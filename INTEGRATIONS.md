# Alarm webhook integrations

When motion (or a person, if AI confirm is on) is detected, the camera sends a
`POST` with this JSON body to your configured URL (rate-limited to 1 per 10 s):

```json
{ "event": "person", "camera": "front-door", "time": "2026-07-04 01:00:31", "score": 33,
  "photo": "http://10.0.0.228/event.jpg" }
```

| field    | meaning |
|----------|---------|
| `event`  | `motion` (heuristic only), `person` (ML-confirmed), or `test` |
| `camera` | the name you set (so a shared bot knows which camera fired) |
| `time`   | wall-clock (needs NTP on), else empty |
| `score`  | person-detection confidence %, or `-1` when AI confirm is off |
| `photo`  | URL of a **snapshot of the moment it triggered** (empty on `test`). Also at `/event.jpg`. |

**Set it up:** web UI → **Recording** card → **Camera name** + **Alarm webhook** → **Save** → **Test**.
The camera must be able to reach the webhook URL. **Note:** `photo` is a LAN address —
whatever fetches it (Home Assistant, a relay) must be on the same network as the camera.

---

## 1. Phone push in 60 seconds — ntfy.sh (no server)

1. Install the **ntfy** app (iOS/Android), subscribe to a private topic, e.g. `xiao-cam-8f3k9q2` (make it random).
2. Webhook URL: `https://ntfy.sh/xiao-cam-8f3k9q2`
3. Done — every event is a push notification. (It shows the raw JSON; use a relay below for prettier text.)

---

## 2. Home Assistant (direct — no relay)

1. Settings → Automations → new → trigger **Webhook**, ID `xiao_cam`.
2. Webhook URL: `http://<HA-IP>:8123/api/webhook/xiao_cam`
3. Example — push notification + turn on a light:

```yaml
alias: XIAO cam alarm
trigger:
  - platform: webhook
    webhook_id: xiao_cam
    allowed_methods: [POST]
    local_only: true
action:
  - service: notify.mobile_app_yourphone
    data:
      title: "📷 {{ trigger.json.camera }}"
      message: "{{ trigger.json.event }} at {{ trigger.json.time }} ({{ trigger.json.score }}%)"
      data:
        image: "{{ trigger.json.photo }}"   # attaches the snapshot to the push
  - service: light.turn_on
    target: { entity_id: light.hallway }
```

Swap `notify.mobile_app_*` for `notify.telegram` (with `data.photo: {{ trigger.json.photo }}`),
`tts.speak`, `switch.turn_on`, etc. The `camera` and `photo` fields make one bot handle
many cameras and show the snapshot inline.

---

## 3. Telegram (needs a tiny relay)

Telegram's API wants `chat_id` + `text`, so point the camera at a small relay that
reformats our JSON. Pick one:

### Option A — Home Assistant as the relay
If you already have HA + the Telegram integration, use the automation in §2 but call
`notify.telegram` in the action. Nothing else needed.

### Option B — standalone relay (Python, run on a Pi / always-on box on the LAN)
Fetches the snapshot and posts it **with a caption** so you see who tripped it:
```python
# pip install flask requests
from flask import Flask, request
import requests
TOKEN = "123456:ABC..."   # from @BotFather
CHAT  = "123456789"       # your id, from @userinfobot
app = Flask(__name__)

@app.post("/alert")
def alert():
    d = request.get_json(force=True)
    cap = f"📷 {d.get('camera')}: {d.get('event')} at {d.get('time')} ({d.get('score')}%)"
    photo = d.get("photo")
    if photo:                                  # send the snapshot with a caption
        img = requests.get(photo, timeout=5).content
        requests.post(f"https://api.telegram.org/bot{TOKEN}/sendPhoto",
                      data={"chat_id": CHAT, "caption": cap},
                      files={"photo": ("event.jpg", img)})
    else:                                      # no photo (e.g. a test) → text only
        requests.get(f"https://api.telegram.org/bot{TOKEN}/sendMessage",
                     params={"chat_id": CHAT, "text": cap})
    return "ok"

app.run(host="0.0.0.0", port=8899)
```
Webhook URL: `http://<relay-ip>:8899/alert`. The relay must be on the same LAN as the
camera (it fetches the `photo` URL). This is the recommended setup — **photo in chat**.

### Option C — Cloudflare Worker (free, no server, reachable from anywhere)
```js
export default {
  async fetch(req, env) {
    const d = await req.json();
    const text = `📷 ${d.event} at ${d.time} (${d.score}%)`;
    await fetch(`https://api.telegram.org/bot${env.TOKEN}/sendMessage` +
      `?chat_id=${env.CHAT}&text=${encodeURIComponent(text)}`);
    return new Response("ok");
  }
};
```
Set `TOKEN` / `CHAT` as Worker secrets; use the Worker URL as the webhook.
(For a public URL the camera must have internet — it does, it's on your WiFi.)

---

## 4. Discord

Discord webhooks want `{ "content": "..." }`, so use a one-line relay (same shape as
§3 Option C) that maps our fields into `content`, or route through HA / n8n.

## 5. Everything else

**n8n, Node-RED, Zapier, Make, IFTTT** all accept the raw JSON `POST` and let you build
any action (SMS, email, siren, smart plug) from the `event` / `time` / `score` fields.
