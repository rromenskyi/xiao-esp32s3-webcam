# XIAO ESP32-S3 Webcam — HTTP API Reference

The firmware exposes a main web/JSON API on **port 80**, a dedicated MJPEG stream server on **port 81**, and — only while the board is unprovisioned and running its `XIAO-SETUP` SoftAP — a separate **provisioning** server on port 80. Reach the device at `http://xiao-cam.local/` (mDNS) or its DHCP IP. Most write endpoints persist to NVS and mirror to `/sdcard/xiao-config.txt`.

## Endpoint summary

### Main server (port 80)

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/` | HTML dashboard / web UI |
| GET | `/control` | Set camera sensor parameters (framesize, quality, flip, …) |
| GET | `/reboot` | Reboot the device |
| POST | `/format_sd` | Unmount and reformat the microSD card |
| GET | `/audio.wav` | Capture live microphone audio as a WAV stream |
| POST | `/ota` | Push a firmware `.bin` (OTA update) then reboot |
| POST | `/rec` | Toggle / configure loop recording + motion |
| GET | `/rec/list` | List recorded clips + full recorder/motion status |
| POST | `/rec/clear` | Delete all recorded clips |
| GET | `/files` | Browse the SD card (JSON directory listing) |
| GET | `/download` | Download a file from the SD card |
| POST | `/delete` | Delete a file on the SD card |
| POST | `/rename` | Rename/move a file on the SD card |
| POST | `/copy` | Copy a file on the SD card |
| POST | `/config/backup` | Write current config (incl. WiFi creds) to SD |
| GET | `/wifi/list` | List saved networks + current connection info |
| POST | `/wifi/add` | Add/update a saved WiFi network |
| POST | `/wifi/del` | Remove a saved WiFi network |
| GET | `/time` | Get current time / NTP status |
| POST | `/ntp` | Enable/disable NTP and set timezone |
| GET | `/sysinfo` | Chip / memory / SD / uptime info |
| GET | `/detect` | Run the person detector on one live frame (debug) |
| GET | `/event.jpg` | JPEG snapshot of the last detection/alarm event |
| GET | `/webhook` | Read alarm (webhook/Telegram) config |
| POST | `/webhook` | Set webhook URL / Telegram / camera name; fire test |
| GET | `/mask` | Read the motion ignore-mask grid |
| POST | `/mask` | Set the motion ignore-mask grid |

### Stream server (port 81)

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/stream` | MJPEG live video (`multipart/x-mixed-replace`) |

> There is **no** `/snapshot` endpoint — the dashboard "Snapshot" button grabs a frame from the `<img>` stream client-side via canvas. Device-side stills come from `/stream` (live) and `/event.jpg` (last alarm frame).

---

## Camera / Stream

### `GET /stream` (port 81)
Live MJPEG. `Content-Type: multipart/x-mixed-replace; boundary=frame`, each part `image/jpeg`. Limited to 3 concurrent viewers (excess → `503`). `503` if the camera is not initialized.

### `GET /control` (port 80)
Adjusts and persists camera sensor settings. Any single invalid value aborts with `400`.

| Param | Type | Range |
|-------|------|-------|
| `framesize` | string | `QVGA`,`VGA`,`SVGA`,`XGA`,`UXGA`,… |
| `quality` | int | 4–30 (lower = better) |
| `brightness`/`contrast`/`saturation` | int | −2 … 2 |
| `hmirror`/`vflip` | 0/1 | mirror / flip |

`GET /control?framesize=SVGA&vflip=1&quality=10` → `{"ok":true}`

### `GET /audio.wav` (port 80)
Live mic as WAV (mono, 16-bit, 16 kHz), DC-blocked + gained. `503` if the mic is busy (recorder holds it).

| Param | Type | Meaning |
|-------|------|---------|
| `secs` | int | 1–30 (default 5) |
| `gain` | int | 1–32 (default 8) |

---

## Recording (loop / dashcam)

Clips go to `/sdcard/rec` as AVI (MJPEG + optional PCM), with an optional `.srt` time sidecar.

### `POST /rec` (port 80)
Query-string params; only supplied ones change, each validated + persisted.

| Param | Type | Range |
|-------|------|-------|
| `on` | 0/1 | enable loop recording |
| `pct` | int | storage budget % of card, 10–95 |
| `audio` | 0/1 | mux mic audio |
| `seg` | int | clip length minutes, 1–60 |
| `motion` | 0/1 | record only on motion |
| `msens` | int | motion sensitivity (% pixels), 1–30 |
| `ml` | 0/1 | confirm motion with ML person detector |
| `pconf` | int | person-confidence % to trigger, 10–90 |
| `srt` | 0/1 | write `.srt` sidecar |

### `GET /rec/list` (port 80)
Clips + full status:
```json
{
  "clips":[{"name":"clip0001.avi","size":10485760,"mtime":"2026-07-01 14:03"}],
  "enabled":true,"audio":true,"pct":80,"seg":15,
  "motion":false,"msens":5,"mactive":false,"mscore":-1,
  "ml":true,"mlready":true,"pconf":25,"mlscore":-1,
  "srt":true,"webhook":false,
  "used":10485760,"total":31914983424,"active":"clip0002.avi","status":"idle"
}
```
`active` = clip being written; `mscore`/`mlscore` = last motion % / person-confidence % (`-1` = not measured).

### `POST /rec/clear` (port 80)
Deletes every clip except the one being written → `{"deleted":12}`.

---

## Motion + AI (person detection)

### `GET /detect` (port 80)
Runs the ML person detector on one fresh frame (debug/tuning).

| Param | Type | Meaning |
|-------|------|---------|
| `fmt` | int | override detector input pixel format |
| `thr` | int | override trigger threshold % |

`{"ready":true,"person":true,"score":72,"ms":48,"w":640,"h":480}` — or `{"ready":false}` when unavailable.

### `GET /event.jpg` (port 80)
JPEG of the moment the last alarm fired. `Cache-Control: no-store`. `404` if none yet.

### `GET /mask` · `POST /mask` (port 80)
Motion ignore-mask over an 8×6 grid; `'1'` cells are ignored.

| Param | Type | Meaning |
|-------|------|---------|
| `grid` | string | exactly 48 chars of `0`/`1` (8×6) |

→ `{"cols":8,"rows":6,"grid":"0000…"}`

---

## Alerts / Webhook / Telegram

### `GET /webhook` · `POST /webhook` (port 80)
On **POST**, the raw request **body** is stored as the webhook URL. Query params (either method) set the rest; `?test=1` fires a test alert immediately.

| Param | Type | Meaning |
|-------|------|---------|
| `name` | string | camera name used in alerts |
| `tgtok` | string | Telegram bot token |
| `tgchat` | string | Telegram chat id |
| `test` | any | fire a test alert now |

→ `{"url":"…","name":"xiao-cam","chat":"123456789","tg":true}` (`tg`=true when a token is set; the token itself is never returned).

The alert payload POSTed on an event:
```json
{"event":"person","camera":"front-door","time":"2026-07-04 01:00:31","score":33,"photo":"http://<ip>/event.jpg"}
```

---

## Files / SD

All paths must start with `/sdcard` and contain no `..`, else `400 "bad path"`.

### `GET /files` — `?dir=` (default `/sdcard`)
```json
{"dir":"/sdcard","entries":[{"name":"rec","dir":true,"size":0,"mtime":"…"}]}
```

### `GET /download` — `?path=` (required)
Raw response with real `Content-Length`, sent as attachment. Runs on a worker; max 2 concurrent (excess → `503`). `404` if missing.

### `POST /delete` — `?path=` → `{"ok":true}`
### `POST /rename` · `POST /copy` — `?src=&dst=` → `{"ok":true}`
### `POST /format_sd` — reformats the card, returns the SD status string.

---

## WiFi

Up to 5 known networks. A password is valid if empty (open) or 8–63 printable ASCII.

### `GET /wifi/list`
```json
{"networks":["HomeWiFi","Garage"],"count":2,"max":5,
 "current":"192.168.1.42","ap":"HomeWiFi","rssi":-58,"ch":6,"phy":"11n"}
```
### `POST /wifi/add` — `?ssid=&pass=` → `{"ok":true}`
### `POST /wifi/del` — `?ssid=` → `{"ok":true}`

---

## Time / NTP

### `GET /time` → `{"ntp":false,"synced":false,"tz":"UTC0","time":"2026-07-01 14:05:12"}`
### `POST /ntp` — `?on=0/1&tz=<POSIX TZ>` → same shape as `/time`.

---

## System / OTA / Config

### `GET /sysinfo`
```json
{"chip":"ESP32-S3","cores":2,"cpu_mhz":240,"flash_mb":8,
 "heap_free":180000,"heap_total":320000,"psram_free":7000000,"psram_total":8000000,
 "sd_free":31900000000,"sd_total":31914983424,"uptime":3600,"idf":"v6.0"}
```
### `GET /reboot` → `"Rebooting"` then restarts.
### `POST /ota` — **body** = raw `.bin`. Writes the inactive slot, validates, reboots.
```
curl -X POST --data-binary @webcam.bin http://xiao-cam.local/ota
```
### `POST /config/backup` — writes full config to `/sdcard/xiao-config.txt` (drop-in provisioning).

---

## Provisioning server (SoftAP only)

Active before the board has WiFi creds, hosting the `XIAO-SETUP` AP. `GET /` = setup page; `GET /scan` = `[{"ssid":"…","rssi":-58}]`; `POST /save` = form (`ssid`,`pass`) → saves + reboots into station mode.
