# Why the timestamp is a subtitle, not burned into the video

Short version: **on the ESP32-S3 you cannot burn a wall-clock timestamp into a
decent-resolution video at a usable frame rate.** It's a hardware limitation, not a
missing feature — so this firmware puts the time where it belongs: in the filename, an
overlay on the live browser view, and a `.srt` subtitle sidecar you can burn in later.

## The physics

The OV2640 sensor outputs **hardware-compressed JPEG**. To draw text into a frame you
must `decode → draw → re-encode` — and JPEG is a DCT bitstream, so there's no way to
poke a pixel without a full round-trip. On the S3 that round-trip is **PSRAM-bandwidth
bound**: a VGA frame is 900 KB of RGB shuffled through PSRAM per frame.

Measured on this board (VGA, 640×480):

| Path | fps |
|------|-----|
| Raw sensor JPEG (no overlay) | **~8.7** |
| decode → draw → re-encode (built-in codec) | ~3.0 |
| decode → draw → re-encode (SIMD `esp_new_jpeg`, both ways) | ~3.8 |

The SIMD codec barely helped — confirming it's memory bandwidth, not compute. Espressif's
own camera maintainer quotes the same ballpark (~300 ms/frame → 3 fps at QVGA) and
recommends overlaying **in the browser** instead.

Every mainstream security-cam / dashcam SoC (HiSilicon, Ingenic, Ambarella, Novatek) does
timestamp OSD in a **dedicated hardware overlay block** that alpha-blends the text before
the hardware encoder — something the ESP32-S3 simply doesn't have. (The **ESP32-P4** does:
hardware JPEG codec + a PPA blend engine, ~10 ms/frame. That's the upgrade path if you must
burn on-device.)

## What we do instead

1. **Filename** — every clip is `clip-YYYYMMDD-HHMMSS.avi`, so you always know the start.
2. **Live browser overlay** — the web UI paints a ticking clock over the stream (zero cost).
3. **`.srt` sidecar** — a per-second subtitle file next to each clip. VLC (and most players)
   show the real wall-clock time on playback, with **no quality loss and no fps hit**.

This is exactly what the flagship ESP32 recorder ([s60sc/ESP32-CAM_MJPEG2SD](https://github.com/s60sc/ESP32-CAM_MJPEG2SD))
does, and how DJI/GoPro handle telemetry too.

## Want it burned into the file?

Do it once, offline, on a real computer — full quality, any resolution:

```bash
ffmpeg -i clip-20260704-224059.avi -vf subtitles=clip-20260704-224059.srt out.mp4
```
