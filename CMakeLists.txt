cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ESP-DL still uses the legacy mbedtls/sha256.h + mbedtls/aes.h headers, which
# IDF v6.0 (mbedtls 4.x) relocated to mbedtls/private/. Bridge the public names
# to the private ones globally so esp-dl compiles unchanged. The legacy C API is
# unchanged — only the header path moved.
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}/main/compat_mbedtls" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-I$ENV{IDF_PATH}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include" APPEND)

# Embed the git short SHA as the app version so pull-OTA can tell builds apart.
execute_process(COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} rev-parse --short=7 HEAD
                OUTPUT_VARIABLE GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(GIT_SHA)
  set(PROJECT_VER "${GIT_SHA}")
endif()

project(xiao_s3_webcam)
