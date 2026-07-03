# Hardware & porting

## Target board

**[Seeed Studio XIAO ESP32S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)**
— the *Sense* variant with the camera + microphone + microSD expansion board. The plain
XIAO ESP32S3 lacks all three. Also sold on Amazon as **`B0C69FFVHH`**.

| | |
|---|---|
| SoC | ESP32-S3 @ 240 MHz, dual-core |
| PSRAM | 8 MB, **octal** (`CONFIG_SPIRAM_MODE_OCT`) |
| Flash | 8 MB |
| Camera | OV2640 (via LCD_CAM), `PIXFORMAT_JPEG` |
| Mic | PDM MEMS |
| Storage | microSD (on the expansion board) |

## Pinout (verified)

### microSD — **SDMMC 1-bit** (not SPI!)

| Signal | GPIO |
|--------|------|
| CLK | 7 |
| CMD | 9 |
| D0 | 8 |

> The Sense board wires the SD slot to the SDMMC controller in 1-bit mode. An SPI +
> chip-select setup (a common copy-paste) gives `ESP_ERR_TIMEOUT (0x107)`. Use SDMMC.

### PDM microphone

| Signal | GPIO |
|--------|------|
| CLK | 42 |
| DATA | 41 |

I2S PDM RX, mono, 16-bit, 16 kHz. The raw signal has a large DC offset — the firmware
applies a DC-block filter + fixed gain. The camera (LCD_CAM) and the mic (I2S PDM) run
simultaneously without conflict.

### Camera

The OV2640 uses the dedicated LCD_CAM peripheral with the standard XIAO Sense pin map
(defined in `init_camera()` in `main/webcam.c`).

## Partition table

`partitions.csv` — two 3.5 MB OTA app slots (needed for the ESP-DL person-detection
model), no internal FAT (recordings live on the microSD):

```
nvs      0x9000   0x6000
otadata  0xf000   0x2000
phy_init 0x11000  0x1000
ota_0    0x20000  0x380000   (3.5 MB)
ota_1    0x3a0000 0x380000   (3.5 MB)
```

Changing the partition table requires a **USB flash** (it can't go over OTA). After that,
OTA works normally.

## Porting to another ESP32-S3 camera board

Most of the firmware is board-agnostic. To adapt it:

1. **Camera pins** — update the `camera_config_t` pin map in `init_camera()`.
2. **SD pins** — update `SDMMC` CLK/CMD/D0 (or switch to SPI if your board wires it that
   way; then set the SPI host + CS).
3. **Mic pins** — update the PDM CLK/DATA in `init_mic()`, or disable the mic if absent.
4. **PSRAM** — if your board has quad (not octal) PSRAM, change `CONFIG_SPIRAM_MODE_OCT`
   to `CONFIG_SPIRAM_MODE_QUAD` in `sdkconfig.defaults` and regenerate sdkconfig.
5. **Flash size** — if not 8 MB, adjust `CONFIG_ESPTOOLPY_FLASHSIZE_*` and the partition
   sizes. The app needs ~3.5 MB per OTA slot with ESP-DL enabled.

After editing `sdkconfig.defaults`, remove the stale `sdkconfig` and regenerate:

```bash
rm -f sdkconfig && idf.py set-target esp32s3 && idf.py build
```

> **Tip:** `sdkconfig.defaults` changes are ignored while a `sdkconfig` file exists.
> Always regenerate after changing defaults.
