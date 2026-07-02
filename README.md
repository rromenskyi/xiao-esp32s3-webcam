# XIAO ESP32S3 Sense Webcam

[![firmware](https://github.com/rromenskyi/xiao-esp32s3-webcam/actions/workflows/firmware.yml/badge.svg)](https://github.com/rromenskyi/xiao-esp32s3-webcam/actions/workflows/firmware.yml)

ESP-IDF firmware for the Seeed Studio XIAO ESP32S3 Sense. It exposes the onboard
OV2640 camera as an MJPEG stream over WiFi with a polished web UI, uses the
onboard PDM microphone and microSD card, and supports firmware updates over the
air — no USB needed after the first flash.

## Features

- **MJPEG camera stream** with a responsive dark web UI (snapshot, fullscreen,
  live controls for resolution / quality / brightness / contrast / saturation /
  mirror / flip).
- **Multiple simultaneous viewers** (up to 3) via async HTTP handlers.
- **First-boot WiFi provisioning**: SoftAP `XIAO-SETUP`, scan + save credentials
  to NVS, auto-reconnect, fallback to setup AP if WiFi is unreachable.
- **mDNS discovery** — reach the board at `http://xiao-cam.local/` without
  knowing its IP.
- **OTA updates over WiFi** — upload a `.bin` from the web UI (or `curl`); the
  board writes the inactive slot and reboots into it.
- **Onboard PDM microphone** — `/audio.wav` records mono 16 kHz PCM with a
  DC-block filter and gain.
- **microSD card** (SDMMC 1-bit) with a format action from the UI.

## Hardware

| | |
|---|---|
| Board | Seeed Studio XIAO ESP32S3 Sense |
| Target | `esp32s3` |
| Camera | OV2640 (LCD_CAM / DVP) |
| Microphone | PDM MSM261D3526H1CPM — CLK=GPIO42, DATA=GPIO41 |
| microSD | SDMMC 1-bit — CLK=GPIO7, CMD=GPIO9, D0=GPIO8 |
| Flash | 8 MB |
| PSRAM | 8 MB octal |

## HTTP Endpoints

| Method | Path | Port | Description |
|---|---|---|---|
| GET | `/` | 80 | Web UI |
| GET | `/stream` | 81 | MJPEG stream (multipart) |
| GET | `/control?<param>=<value>` | 80 | Camera settings |
| GET | `/audio.wav?secs=N&gain=G` | 80 | Record mic to WAV |
| POST | `/ota` | 80 | Upload firmware `.bin` (OTA) |
| POST | `/format_sd` | 80 | Format the SD card |
| GET | `/reboot` | 80 | Reboot |

## Build

ESP-IDF 5.4 needs a `python3` that is 3.11+, but macOS `python3` is 3.9. Put a
3.11 ahead of it on `PATH` before sourcing `export.sh`:

```bash
mkdir -p ~/.espidf-shim
ln -sf "$(command -v python3.11)" ~/.espidf-shim/python3
export PATH="$HOME/.espidf-shim:$PATH"
source /path/to/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

## Flash (first time, over USB)

```bash
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

If the board will not auto-reset into download mode (a known ESP32-S3 native-USB
quirk — esptool prints "No serial data received"), enter it manually: hold the
**B (BOOT)** button, tap **R (RESET)**, release **B**, then flash.

## Update (afterwards, over WiFi)

Use the **Firmware** card in the web UI, or:

```bash
curl --data-binary @build/xiao_s3_webcam.bin http://xiao-cam.local/ota
```

The board flashes the inactive OTA slot and reboots into the new firmware.

## First Boot

1. Connect to WiFi `XIAO-SETUP` (password `setup123`).
2. Open `http://192.168.4.1`, scan, select your network, enter the password, save.
3. The board reboots and connects to your WiFi.
4. Open `http://xiao-cam.local/` (or the IP shown in the serial monitor).

## Partitions

OTA layout (`partitions.csv`): `nvs`, `otadata`, `phy_init`, two 2 MB app slots
(`ota_0` / `ota_1`), and a 3.5 MB FAT `storage` partition for the SD/data.

## Dependencies

Pulled via the ESP-IDF Component Manager (locked in `dependencies.lock`):

```yaml
espressif/esp32-camera: ^2.1.7
espressif/mdns: ^1.2
```
