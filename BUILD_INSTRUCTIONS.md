# Build And Flash

## One-Time Host Setup

Clone ESP-IDF v5.4 (e.g. next to this repo) and export `IDF_PATH` to it.

Host tools required:

- ESP-IDF tools for `esp32s3`
- ESP-IDF Python env on Python 3.11
- CMake
- Ninja

ESP-IDF 5.4 needs a `python3` that is 3.11+, but macOS `python3` is 3.9.
`export.sh` resolves its env from the `python3` on `PATH`, so put a shim dir with
a `python3` symlink to a 3.11 interpreter ahead of the system one. Create it once:

```bash
mkdir -p ~/.espidf-shim
ln -sf "$(command -v python3.11)" ~/.espidf-shim/python3
```

Then activate ESP-IDF with the shim on `PATH`:

```bash
export PATH="$HOME/.espidf-shim:$PATH"
source /path/to/esp-idf/export.sh
```

## Build

```bash
export PATH="$HOME/.espidf-shim:$PATH"
source /path/to/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

## Flash (first time, USB)

```bash
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

Replace `/dev/cu.usbmodemXXXX` with the actual port from `ls /dev/cu.usb*`. If
the board will not auto-reset into download mode ("No serial data received"),
hold **B (BOOT)**, tap **R (RESET)**, release **B**, then flash.

## Update (afterwards, over WiFi)

```bash
curl --data-binary @build/xiao_s3_webcam.bin http://xiao-cam.local/ota
```

## Useful Commands

```bash
idf.py build
idf.py monitor
idf.py fullclean
idf.py erase-flash
```

## Runtime Setup

WiFi credentials are configured only through the setup page and saved to NVS.

On first boot:

1. Join `XIAO-SETUP` / `setup123`.
2. Open `http://192.168.4.1`.
3. Save WiFi credentials.
4. After reboot, open `http://<device-ip>/`.

If saved WiFi is wrong or unavailable, the firmware retries and then returns to setup AP mode.
