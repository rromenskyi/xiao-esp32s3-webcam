cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ESP-DL still uses the legacy mbedtls/sha256.h + mbedtls/aes.h headers, which
# IDF v6.0 (mbedtls 4.x) relocated to mbedtls/private/. Bridge the public names
# to the private ones globally so esp-dl compiles unchanged. The legacy C API is
# unchanged — only the header path moved.
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}/main/compat_mbedtls" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-I$ENV{IDF_PATH}/components/mbedtls/mbedtls/tf-psa-crypto/drivers/builtin/include" APPEND)

project(xiao_s3_webcam)
