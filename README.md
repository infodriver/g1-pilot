# G1 Pilot

Web console for a Unitree G1: type text → the robot speaks; plus supervised pose,
gesture and walking controls. Runs on the robot's onboard dev PC (Orin NX) and is
reachable from the lab Wi-Fi.

## Open it

```
ssh g1 cat ~/g1-pilot/url.txt
```

That file holds the full URL (with the access key) for both the Wi-Fi address
(`http://<robot-ip>:8130/?k=…`) and the wired/robot-LAN address
(`http://<robot-wired-ip>:8130/?k=…`). The key lives in `~/g1-pilot/config.json`
(mode 600); requests without it are rejected with 401.

## What it does

| Panel | Endpoint | Notes |
| --- | --- | --- |
| Speech | `POST /api/say` | `{"text","engine":"native"\|"local","speaker_id":0-3}`. Native = Unitree `TtsMaker`; local = Piper `en_US-ryan-medium` streamed as 16 kHz PCM (offline). If one engine fails the other is used, and the response reports the voice that actually spoke. |
| Voice selection | `POST /api/tts` | `{"engine":"native"\|"local","speaker_id":0-3}` — **saved** to config.json, so it survives reloads and is used for every reply, including Voice-ask answers. `GET /api/state` reports `tts_engine`, `tts_speaker`, `tts_local_available`, and `tts_voices` (per-language Piper files present). |
| Answer language | `POST /api/tts` / `POST /api/ask_config` | `{"language":"en"\|"ru"\|"zh"}`. Piper voice per language: `piper_voices` in config.json (en Ryan, ru Dmitri, zh Huayan). |
| Pose & balance | `POST /api/loco` | `prepare, sit, stand_up, lie_to_stand, squat, damp, start, high_stand, low_stand, zero_torque, stop_move, wave_hand, wave_hand_turn, shake_hand`. **prepare** = bring-up sequence damp → start (fsm 200) → stand up (706), per-step codes returned in `steps`. |
| Gestures | `POST /api/arm` / `/api/loco` | Unitree arm-action presets: `shake hand, high five, hug, clap, heart, right heart, hands up, face wave, high wave, right hand up, reject, x-ray, left kiss, right kiss, two-hand kiss, release arm`. The panel also puts **Zero torque (limp)** and **Damp (damping)** at the top of the Gestures grid (2026-09-30, at Kai's request) — loco actions, same motion gate. **Prepare robot** (top of Pose & balance) runs `damp → start → stand up` — the bring-up sequence that puts the robot in standing main control, which the arm presets appear to require (they answer `7404` while damped; loco gestures work). Command results are real RPC codes (green accepted / red code X) because the app calls the RPC layer directly — the SDK wrappers return `None`; gate refusals log `motion_blocked`. |
| Walk | `POST /api/move` | `{"vx","vy","vyaw"}`, clamped to `limits` in config, duration 1 s, deadman 0.6 s |
| Volume / LED | `POST /api/volume`, `POST /api/led` | 0-100 volume; RGB LED (`LedControl`) |
| Telemetry | `GET /api/state` | loco FSM id/mode, volume, E-stop, moving/speaking, command log |
| Health | `GET /health` | no key required |
| Voice ask (DJI) | `GET /api/ask/status`, `/api/ask/kb`; `POST /api/ask`, `/api/ask_start`, `/api/ask_stop`, `/api/ask_text`, `/api/ask_config`, `/api/ask/kb`, `/api/ask/kb_update`, `/api/ask/kb_delete` | Proxied to the Mac's `dji-ask` service (mic capture + Whisper + knowledge base + web lookup). The robot app speaks the returned answer locally. |

## Voice ask (DJI microphone)

### Languages (DISABLED 21:0x — English only)

The operator asked to return to the English-only version: `language: "en"`, `language_strict: false`,
`whisper_model: "base.en"`, and the language selector is removed from both UIs. The ru/zh
machinery and the downloaded Piper voices (Dmitri/Huayan) and Whisper `tiny`/`small` models
are left on disk but inactive — re-enabling is a config change plus restoring the selector.

Original design (kept for reference):

- A `tiny` multilingual Whisper pass identifies the spoken language before anything else.
- With `language_strict` on, a question in a different language is **not answered and not
  spoken** — the panel just reports e.g. *"heard Russian — only answering English"*.
- Transcription then uses a per-language model (`whisper_models`: en → `base.en` fast,
  ru/zh → `small`), and the answer is produced in the chosen language.
- Online search follows the language: `ru.wikipedia.org` / `zh.wikipedia.org`, with an
  English-translation search as an extra fallback.
- Speech uses the matching Piper voice (en Ryan, ru Dmitri, zh Huayan); the English voice
  can also be Unitree's native TTS.

The DJI `Wireless Mic Rx` is plugged into the MacBook, so the microphone, Whisper and
answer lookup live in the separate `dji-ask` service (port 8140). G1 Pilot proxies to
it so the whole thing is one app in the browser:

```
G1 Pilot page (robot)  ──►  /api/ask*  ──►  Mac dji-ask :8140
        ▲                                        │  ffmpeg + Whisper + KB + web
        └────────── answer text ─────────────────┘
        └── spoken here by the robot's own speaker (AudioClient.TtsMaker)
```

- Backend address: `config.json` → `ask.url` (empty = auto-detect: the robot uses the
  address of the browser that opened this page, so opening G1 Pilot from the Mac that
  has the DJI receiver just works). `ask.port` defaults to 8140, `ask.key` is the shared
  key the Mac service requires from LAN callers.
- Requires the Mac service running: `dji-ask/start.sh`, and the Mac and robot on the
  same network. `GET /api/ask/status` reports `reachable` plus the backend URL.
- Panel: ask button (silence-detected), hold-to-talk, typed questions, level meter,
  knowledge-base add / **edit** / delete (edit loads the entry into the form; the button becomes
  *Save changes*, Cancel restores add mode), online toggle and mic threshold.
- **Jokes panel** — `data/jokes.jsonl` is a separate bank; "tell me a joke" serves a random
  un-told joke verbatim, and once they are all told (or the bank is empty) it fetches one
  online. The panel shows *N saved, M told* with add / edit / delete / *Reset told*,
  proxied as `/api/ask/jokes*`.

## Safety model

- **Nothing moves at startup.** The server never issues a motion command unless a request arrives.
- Two gates on every motion request: the UI's *motion armed* checkbox (sent as `X-Pilot-Motion: clear`) and a server-side E-stop flag. Both must be clear.
- **E-STOP** (`POST /api/estop`, also the Escape key) sends `StopMove` + `Damp` and latches; `/api/estop_clear` releases it.
- Walk commands are non-continuous (1 s each) and the deadman thread zeroes velocity 0.6 s after the last one; releasing the button also sends an explicit stop.
- Velocities clamped in both the UI (speed slider) and the server.
- `zero_torque` releases the joints — the robot collapses unless supported.

## Ops

```sh
systemctl --user status g1-pilot      # active + enabled at boot
systemctl --user restart g1-pilot
tail -f ~/g1-pilot/pilot.log          # JSON lines, one per command
```

CLI helper (reads the key from config.json, so no secrets in argv):

```sh
~/g1-pilot/pilotctl.py state
~/g1-pilot/pilotctl.py say "Hello" [native|local]
~/g1-pilot/pilotctl.py vol 100
~/g1-pilot/pilotctl.py loco sit --arm        # --arm needed for anything that moves
~/g1-pilot/pilotctl.py arm "high five" --arm
~/g1-pilot/pilotctl.py estop | clear
```

Config: `~/g1-pilot/config.json` (`interface` must be the robot-side NIC, `eth0`;
`key`; `limits`; `deadman_seconds`; Piper paths). Restart after editing.

## Files

- `g1_pilot.py` — HTTP server + DDS/RPC clients (`LocoClient`, `G1ArmActionClient`, `AudioClient`)
- `pilotctl.py` — local CLI (keyless command line, `--arm` for motion)
- `index.html` — single-page UI (no external assets, works offline)
- `g1-pilot.service` — systemd user unit
- `config.json`, `url.txt`, `pilot.log` — runtime state (not in this repo copy)

Verified 2026-09-26: health, state read (fsm 0 = zero torque), native TTS `code 0`,
volume → 100 (max), 401 on keyless requests, page served over Wi-Fi and opened in
the operator's browser. Voice switching: `voice local` then a plain `/api/say` reports
`engine: piper` (and a Voice-ask answer reports `spoken.engine: piper`); `voice native`
returns to Unitree. Motion commands were **not** exercised — standing up needs a human
spotter.
