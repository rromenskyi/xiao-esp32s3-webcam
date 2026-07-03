# 📷 XIAO ESP32S3 Sense — Smart Webcam Firmware

[![firmware](https://github.com/rromenskyi/xiao-esp32s3-webcam/actions/workflows/firmware.yml/badge.svg)](https://github.com/rromenskyi/xiao-esp32s3-webcam/actions/workflows/firmware.yml)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0-red)](https://github.com/espressif/esp-idf)
[![board](https://img.shields.io/badge/board-XIAO%20ESP32S3%20Sense-0aa)](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)
[![license](https://img.shields.io/badge/license-MIT-blue)](#license)

A thumb-sized board turned into a **smart security camera**: live MJPEG stream,
**on-device AI person detection**, motion-gated recording to SD, and **Telegram alerts
with a snapshot** — all configured from a polished web UI, updated over the air, with
zero cloud dependency.

> Everything runs on the **~$14 Seeed XIAO ESP32S3 Sense** (ESP32-S3, 8 MB PSRAM,
> OV2640 camera, PDM mic, microSD). No hub, no subscription, nothing leaves your LAN.

---

## ✨ Features

### 📹 Camera & streaming
- **MJPEG stream** with a responsive dark web UI — snapshot, fullscreen, and live
  controls for resolution, JPEG quality, brightness, contrast, saturation, mirror, flip.
- **Multiple simultaneous viewers** via async HTTP handlers.
- Live **wall-clock overlay** drawn in the browser (zero cost to the video).

### 🧠 Recording & AI
- **Loop recording to microSD** (MJPEG-in-AVI + PCM audio), with a disk budget
  (default 80 %) and time-based clips (~15 min), auto-resuming after power-up.
- **Motion-gated recording** — a background-subtraction heuristic records only when
  something moves; each event becomes its own timestamped clip.
- **On-device ML person detection** (Espressif ESP-DL `pedestrian_detect`) confirms a
  *person* before recording — curtains, light changes and pets stop triggering it.
- **Ignore zones** — paint a mask over the frame to exclude a window or a busy street.
- **Timestamp subtitles** — a per-second `.srt` sidecar next to each clip so any player
  shows the real time (no lossy re-encode; see [docs/TIMESTAMPS.md](docs/TIMESTAMPS.md)).
- **Crash-safe** — the AVI is checkpointed ~every 2 s, so a power cut costs ≤ 2 s.

### 🔔 Alerts
- **Telegram, built in** — on a detection the camera sends the **snapshot with a caption**
  straight to your chat over HTTPS. No relay, no server.
- **Generic webhook** — POSTs `{event, camera, time, score, photo}` to any URL
  (Home Assistant, n8n, ntfy, Discord…). See **[INTEGRATIONS.md](INTEGRATIONS.md)**.
- Snapshot of the triggering moment served at `/event.jpg`.

### 🌐 Connectivity & ops
- **First-boot provisioning** — SoftAP `XIAO-SETUP`, scan + save WiFi, auto-reconnect;
  multiple known networks (main + repeaters), roams to whichever is up.
- **mDNS** — reach the board at `http://xiao-cam.local/`.
- **OTA over WiFi** — flash a `.bin` from the UI or `curl`; dual 3.5 MB slots. No USB
  after the first flash.
- **NTP time** with a human-readable timezone picker (opt-in).
- **Config backup to SD** — save *everything* (WiFi, camera, recording, motion, AI,
  Telegram, timezone) to the card; drop it into a fresh board to auto-provision.
- **System card** — RAM, PSRAM, flash, CPU, free SD space, uptime.

---

## 🚀 Quick start

**Already have a board with this firmware?**
1. Power it up → join the `XIAO-SETUP` WiFi → pick your network → save.
2. Open **`http://xiao-cam.local/`** (or the IP shown in your router).
3. Stream is live — turn on recording / AI / Telegram in the **Recording** card.

**Flashing a fresh board** — see **[BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)**
(one USB flash, then OTA forever).

---

## 🔩 Hardware

Built and tested for the **[Seeed XIAO ESP32S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)**
(the "Sense" variant with the camera + mic + microSD expansion board — the plain XIAO
ESP32S3 lacks these). Also sold on Amazon as `B0C69FFVHH`.

| | |
|---|---|
| **SoC** | ESP32-S3 @ 240 MHz, 8 MB PSRAM (octal), 8 MB flash |
| **Camera** | OV2640 via LCD_CAM, `PIXFORMAT_JPEG` |
| **Mic** | PDM, CLK `42` / DATA `41`, I2S PDM RX mono 16 kHz |
| **microSD** | **SDMMC 1-bit** — CLK `7` / CMD `9` / D0 `8` (not SPI!) |

Porting to another ESP32-S3 camera board? See **[docs/HARDWARE.md](docs/HARDWARE.md)**.

---

## 🗺️ How it works

```
  OV2640 ──JPEG──┐
                 ├──►  MJPEG stream  ──────────►  browser   (:81)
  PDM mic ───────┤
                 ├──►  motion ──►  ESP-DL person?  ──►  record ──►  AVI + .srt ──►  microSD
                 │
                 └──►  Telegram  ·  webhook  ·  /event.jpg

        WiFi (roams)  ·  mDNS xiao-cam.local  ·  OTA-over-WiFi  ·  web UI (:80)
```

- **ESP-IDF v6.0**, single-file app (`main/webcam.c`) + a small C++ wrapper for ESP-DL.
- Two HTTP servers: UI/API on `:80`, the MJPEG stream on `:81`.
- Settings persist to **NVS**; a full copy can be mirrored to the **SD card**.
- Full HTTP API: **[docs/API.md](docs/API.md)**.

---

## 🧭 Roadmap

- [ ] Wake-word ("OK ESP") → stream to your own Whisper + LLM
- [ ] Pull-OTA: check a URL + SHA-256 and self-update (nightly / stable channels)
- [ ] Snapshot-thumbnail timeline in the file browser

---

## 📚 Docs

| | |
|---|---|
| [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) | Toolchain, build, first USB flash, OTA |
| [INTEGRATIONS.md](INTEGRATIONS.md) | Telegram, Home Assistant, ntfy, Discord, n8n |
| [docs/API.md](docs/API.md) | Every HTTP endpoint |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Pinout & porting guide |
| [docs/TIMESTAMPS.md](docs/TIMESTAMPS.md) | Why the timestamp is a subtitle, not burned-in |

---

## License

MIT — see [LICENSE](LICENSE). Built with [ESP-IDF](https://github.com/espressif/esp-idf),
[esp32-camera](https://github.com/espressif/esp32-camera), and
[ESP-DL](https://github.com/espressif/esp-dl).
