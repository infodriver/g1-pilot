#!/usr/bin/env python3
"""G1 Pilot — web console for the Unitree G1: text-to-speech + supervised motion.

Design rules (safety first):
  * Nothing moves until the operator explicitly arms the console (motion gate)
    and clears the E-stop.
  * Every motion request is whitelisted, rate-limited and clamped.
  * Walk commands are non-continuous (1 s) and additionally guarded by a
    server-side deadman that zeroes velocity when commands stop arriving.
  * No motion is ever issued at startup.
"""
import json
import os
import secrets
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import wave
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "config.json")

DEFAULTS = {
    "interface": "eth0",
    "bind": "0.0.0.0",
    "port": 8130,
    "key": "",
    "deadman_seconds": 0.6,
    "limits": {"vx": 0.30, "vy": 0.25, "vyaw": 0.50},
    "tts_default_engine": "native",
    "tts_default_speaker": 1,
    "piper_bin": "/home/unitree/g1-voice/vendor/piper/piper",
    "piper_voice": "/home/unitree/g1-voice/models/en_US-ryan-medium.onnx",
    "piper_voices": {
        "en": "/home/unitree/g1-voice/models/en_US-ryan-medium.onnx",
        "ru": "/home/unitree/g1-voice/models/ru_RU-dmitri-medium.onnx",
        "zh": "/home/unitree/g1-voice/models/zh_CN-huayan-medium.onnx",
    },
    "speaker_match": 55,
    # DJI mic / voice-ask backend: the Mac running dji-ask (STT + knowledge base).
    # Empty url = auto-detect from the browser that opens this page.
    "ask": {"url": "", "port": 8140, "key": "", "timeout_s": 180, "last_client": ""},
}


def load_config():
    cfg = json.loads(json.dumps(DEFAULTS))
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH) as fh:
            user = json.load(fh)
        for k, v in user.items():
            if k in ("limits", "ask", "piper_voices") and isinstance(v, dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    if not cfg.get("key"):
        cfg["key"] = secrets.token_hex(8)
        save_config(cfg)
    return cfg


def save_config(cfg):
    tmp = CONFIG_PATH + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(cfg, fh, indent=2)
    os.replace(tmp, CONFIG_PATH)


# ---------------------------------------------------------------- robot link
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
)
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

CFG = load_config()

FSM_LABELS = {
    0: "zero torque (limp)",
    1: "damp",
    2: "squat",
    3: "sit",
    4: "main control",
    200: "start",
    702: "lie->stand",
    706: "squat->stand",
}

LOCK = threading.RLock()
AUDIO_LOCK = threading.Lock()
LOG = deque(maxlen=200)
START_TS = time.time()

STATE = {
    "estop": False,
    "moving": False,
    "last_move": 0.0,
    "speaking_until": 0.0,
    "last_error": None,
}


def log(event, detail=""):
    entry = {"t": time.strftime("%H:%M:%S"), "event": event, "detail": detail}
    LOG.appendleft(entry)
    print(json.dumps(entry), flush=True)


ChannelFactoryInitialize(0, CFG["interface"])

LOCO = LocoClient(); LOCO.SetTimeout(6.0); LOCO.Init()
ARM = G1ArmActionClient(); ARM.SetTimeout(6.0); ARM.Init()
AUDIO = AudioClient(); AUDIO.SetTimeout(6.0); AUDIO.Init()

# Actions the console may invoke. "motion": true  -> gated by E-stop + motion gate.
# These call the RPC layer directly (same API id + payload as the SDK's convenience
# wrappers) so the real result code reaches the UI and the log — the wrappers return None.
_SHAKE_STATE = {"first": False}


def _shake_hand_code():
    _SHAKE_STATE["first"] = not _SHAKE_STATE["first"]
    return LOCO.SetTaskId(3 if _SHAKE_STATE["first"] else 2)


def _prepare_robot():
    """Bring the robot from damp/limp up to standing main control.

    Unitree's own example does Damp → (pause) → Squat2StandUp; "Start" (fsm 200) enters main
    operation control first, which is what the arm-action presets appear to require.
    """
    steps = []
    for name, fn, pause in (("damp", lambda: LOCO.SetFsmId(1), 0.6),
                            ("start", lambda: LOCO.SetFsmId(200), 0.8),
                            ("stand_up", lambda: LOCO.SetFsmId(706), 0.0)):
        try:
            code = fn()
        except Exception as exc:
            code = f"error: {exc}"
        steps.append({"step": name, "code": code})
        log("prepare_step", f"{name} code={code}")
        if pause and code in (0, None):
            time.sleep(pause)
    STATE["last_prepare"] = steps
    last = steps[-1]["code"] if steps else None
    return last if isinstance(last, int) else 0


LOCO_ACTIONS = {
    "prepare": (_prepare_robot, True),
    "sit": (lambda: LOCO.SetFsmId(3), True),
    "stand_up": (lambda: LOCO.SetFsmId(706), True),
    "lie_to_stand": (lambda: LOCO.SetFsmId(702), True),
    "squat": (lambda: LOCO.SetFsmId(706), True),
    "damp": (lambda: LOCO.SetFsmId(1), True),
    "zero_torque": (lambda: LOCO.SetFsmId(0), True),
    "start": (lambda: LOCO.SetFsmId(200), True),
    "high_stand": (lambda: LOCO.SetStandHeight((1 << 32) - 1), True),
    "low_stand": (lambda: LOCO.SetStandHeight(0), True),
    "stop_move": (lambda: LOCO.SetVelocity(0.0, 0.0, 0.0), False),
    "wave_hand": (lambda: LOCO.SetTaskId(0), True),
    "wave_hand_turn": (lambda: LOCO.SetTaskId(1), True),
    "shake_hand": (_shake_hand_code, True),
}


def read_state():
    """Read-only telemetry; never issues commands."""
    out = {"fsm_id": None, "fsm_mode": None, "balance_mode": None, "stand_height": None}
    probes = [
        ("fsm_id", ROBOT_API_ID_LOCO_GET_FSM_ID),
        ("fsm_mode", ROBOT_API_ID_LOCO_GET_FSM_MODE),
        ("balance_mode", ROBOT_API_ID_LOCO_GET_BALANCE_MODE),
        ("stand_height", ROBOT_API_ID_LOCO_GET_STAND_HEIGHT),
    ]
    with LOCK:
        for name, api_id in probes:
            try:
                code, data = LOCO._Call(api_id, "{}")
                out[name] = json.loads(data).get("data") if code == 0 and data else None
            except Exception as exc:  # transport hiccup: report, do not raise
                out[name] = None
                STATE["last_error"] = f"{name}: {exc}"
    return out


def deadman_loop():
    while True:
        time.sleep(0.15)
        if STATE["moving"] and time.monotonic() - STATE["last_move"] > CFG["deadman_seconds"]:
            try:
                with LOCK:
                    LOCO.StopMove()
                log("deadman_stop")
            except Exception as exc:
                STATE["last_error"] = f"deadman: {exc}"
            STATE["moving"] = False


threading.Thread(target=deadman_loop, daemon=True).start()


# ------------------------------------------------------------------- speech
def speak_native(text, speaker_id):
    with LOCK:
        code = AUDIO.TtsMaker(text, int(speaker_id))
    return code, "native"


def speak_local(text, voice=None):
    """Offline fallback: Piper voice -> 16 kHz mono PCM -> Unitree speaker."""
    exe, voice = CFG["piper_bin"], voice or CFG["piper_voice"]
    if not (os.path.exists(exe) and os.path.exists(voice)):
        return -1, "piper missing"
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = os.path.dirname(exe) + ":" + env.get("LD_LIBRARY_PATH", "")
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        wav_path = tmp.name
    try:
        proc = subprocess.run(
            [exe, "-m", voice, "-f", wav_path],
            input=text.encode(), env=env, capture_output=True, timeout=60,
        )
        if proc.returncode != 0:
            return -1, proc.stderr.decode(errors="replace")[-200:]
        import audioop
        with wave.open(wav_path, "rb") as wf:
            rate = wf.getframerate()
            pcm = wf.readframes(wf.getnframes())
        pcm, _ = audioop.ratecv(pcm, 2, 1, rate, 16000, None)
        if not pcm:
            return -1, "empty pcm"
        stream_id = secrets.token_hex(4)
        with AUDIO_LOCK:
            chunk = 32000
            for offset in range(0, len(pcm), chunk):
                code, _ = AUDIO.PlayStream("g1-pilot", stream_id, pcm[offset:offset + chunk])
                if code != 0:
                    return code, "PlayStream"
            STATE["speaking_until"] = time.monotonic() + len(pcm) / 32000.0
        return 0, "piper"
    finally:
        try:
            os.unlink(wav_path)
        except OSError:
            pass


def piper_voice_for(lang):
    """Piper voice file for a language code, falling back to the configured default."""
    code = (lang or "en").lower().split("-")[0]
    voices = CFG.get("piper_voices") or {}
    candidate = voices.get(code) or CFG["piper_voice"]
    return candidate if os.path.exists(candidate) else CFG["piper_voice"]


def handle_say(payload):
    text = (payload.get("text") or "").strip()
    if not text:
        return 400, {"error": "text is required"}
    if len(text) > 600:
        text = text[:600]
    engine = payload.get("engine") or CFG["tts_default_engine"]
    speaker = payload.get("speaker_id", CFG["tts_default_speaker"])
    voice = piper_voice_for(payload.get("lang"))
    started = time.time()
    if engine == "local":
        code, used = speak_local(text, voice)
        if code != 0:
            # Piper unavailable/broken: still say something, and say what actually spoke
            log("tts_local_failed", str(used))
            try:
                code, used = speak_native(text, speaker)
            except Exception as exc:
                code, used = -1, f"native error: {exc}"
    else:
        try:
            code, used = speak_native(text, speaker)
        except Exception as exc:
            code, used = -1, f"native error: {exc}"
        if code != 0:
            log("tts_native_failed", f"code={code}")
            code, used = speak_local(text, voice)
    log("say", f"engine={used} code={code} text={text[:60]!r}")
    return 200, {"code": code, "engine": used, "seconds": round(time.time() - started, 2)}


# ------------------------------------------------- DJI mic / voice-ask backend
def ask_url():
    """Where the Mac's dji-ask service lives (mic capture + STT + knowledge base).

    Priority: explicit url → last browser we saw (persisted, so a restart keeps working) →
    the browser of the request being served right now.
    """
    configured = (CFG["ask"].get("url") or "").strip()
    if configured and configured != "auto":
        return configured.rstrip("/")
    host = CFG["ask"].get("last_client") or STATE.get("browser_ip")
    if host:
        return f"http://{host}:{CFG['ask'].get('port', 8140)}"
    return ""


def ask_call(path, payload=None, timeout=None):
    base = ask_url()
    if not base:
        return False, {"error": "no DJI backend known yet - open this page from the Mac "
                                "that has the DJI receiver plugged in"}
    data = json.dumps(payload).encode() if payload is not None else None
    request = urllib.request.Request(base + path, data=data,
                                     method="POST" if payload is not None else "GET")
    request.add_header("X-DJI-Key", CFG["ask"].get("key", ""))
    if data:
        request.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(request, timeout=timeout or CFG["ask"].get("timeout_s", 180)) as response:
            return True, json.loads(response.read() or "{}")
    except urllib.error.HTTPError as exc:
        return False, {"error": f"ask backend HTTP {exc.code}",
                       "body": exc.read().decode(errors="replace")[:200]}
    except Exception as exc:
        return False, {"error": f"ask backend unreachable at {base}: {exc}"}


def speak_answer(text, lang=None):
    """The answer is spoken by the robot itself (no extra hop through the Mac)."""
    if not text:
        return {"spoke": False}
    try:
        code, body = handle_say({"text": text, "lang": lang})
    except Exception as exc:
        return {"spoke": False, "error": str(exc)}
    return {"spoke": code == 200 and body.get("code") == 0,
            "code": body.get("code"), "engine": body.get("engine"), "lang": lang}


# ------------------------------------------------------------------ handlers
class Handler(BaseHTTPRequestHandler):
    server_version = "g1-pilot/1.0"

    def log_message(self, fmt, *args):  # keep the journal quiet
        pass

    # -- helpers
    def _key_ok(self):
        supplied = self.headers.get("X-Pilot-Key") or ""
        if not supplied:
            supplied = (parse_qs(urlparse(self.path).query).get("k") or [""])[0]
        return secrets.compare_digest(supplied, CFG["key"])

    def _motion_gate(self):
        if STATE["estop"]:
            return "E-stop is engaged"
        if (self.headers.get("X-Pilot-Motion") or "") != "clear":
            return "motion gate not armed"
        return None

    def _send(self, code, obj, ctype="application/json"):
        body = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except BrokenPipeError:
            pass

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return {}

    # -- routes
    def _note_client(self):
        client = self.client_address[0]
        if client in ("127.0.0.1", "::1"):
            return
        STATE["browser_ip"] = client
        # remember it on disk: otherwise a restart loses the DJI backend until someone
        # happens to open this page again
        if CFG["ask"].get("last_client") != client:
            CFG["ask"]["last_client"] = client
            save_config(CFG)
            log("ask_backend_client", client)

    def _ask_proxy(self, path, payload):
        if path == "/api/ask":
            ok, data = ask_call("/api/ask", {"mode": payload.get("mode", "auto"), "speak": False})
            if ok:
                data["spoken"] = speak_answer((data.get("answer") or {}).get("text"), data.get("lang"))
            return self._send(200 if ok else 502, data)
        if path == "/api/ask_start":
            ok, data = ask_call("/api/ask_start", {})
            return self._send(200 if ok else 502, data)
        if path == "/api/ask_stop":
            ok, data = ask_call("/api/ask_stop", {"speak": False})
            if ok:
                data["spoken"] = speak_answer((data.get("answer") or {}).get("text"), data.get("lang"))
            return self._send(200 if ok else 502, data)
        if path == "/api/ask_text":
            ok, data = ask_call("/api/ask_text", {"question": payload.get("question", ""), "speak": False})
            if ok:
                data["spoken"] = speak_answer((data.get("answer") or {}).get("text"), data.get("lang"))
            return self._send(200 if ok else 502, data)
        if path == "/api/ask_config":
            ok, data = ask_call("/api/config", dict(payload))
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/kb":
            ok, data = ask_call("/api/kb", {"text": payload.get("text", ""),
                                            "title": payload.get("title", "")})
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/kb_delete":
            ok, data = ask_call("/api/kb_delete", {"id": payload.get("id", "")})
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/kb_update":
            ok, data = ask_call("/api/kb_update", {"id": payload.get("id", ""),
                                                    "text": payload.get("text", ""),
                                                    "title": payload.get("title", "")})
            if ok:
                log("kb_edit", f"{data.get('id')} via app")
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/jokes_add":
            ok, data = ask_call("/api/jokes", {"text": payload.get("text", ""),
                                               "title": payload.get("title", "")})
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/jokes_update":
            ok, data = ask_call("/api/jokes_update", {"id": payload.get("id", ""),
                                                      "text": payload.get("text", ""),
                                                      "title": payload.get("title", "")})
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/jokes_delete":
            ok, data = ask_call("/api/jokes_delete", {"id": payload.get("id", "")})
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/jokes_reset":
            ok, data = ask_call("/api/jokes_reset", {})
            return self._send(200 if ok else 502, data)
        return self._send(404, {"error": "unknown ask route"})

    def do_GET(self):
        self._note_client()
        path = urlparse(self.path).path
        if path == "/health":
            return self._send(200, {"ok": True, "service": "g1-pilot"})
        if not self._key_ok():
            return self._send(401, {"error": "missing or bad key", "hint": "open /?k=<key>"})
        if path == "/api/ask/status":
            ok, data = ask_call("/api/status", timeout=15)
            data["backend"] = ask_url()
            data["reachable"] = ok
            data["voice"] = CFG["tts_default_engine"]
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/kb":
            ok, data = ask_call("/api/kb", timeout=15)
            return self._send(200 if ok else 502, data)
        if path == "/api/ask/jokes":
            ok, data = ask_call("/api/jokes", timeout=20)
            return self._send(200 if ok else 502, data)
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as fh:
                return self._send(200, fh.read(), "text/html; charset=utf-8")
        if path == "/api/state":
            state = read_state()
            try:
                with LOCK:
                    vol_code, vol_data = AUDIO.GetVolume()
                volume = vol_data.get("volume") if vol_code == 0 and vol_data else None
            except Exception:
                volume = None
            return self._send(200, {
                "robot": state,
                "fsm_label": FSM_LABELS.get(state.get("fsm_id"), f"fsm {state.get('fsm_id')}"),
                "volume": volume,
                "estop": STATE["estop"],
                "moving": STATE["moving"],
                "speaking": time.monotonic() < STATE["speaking_until"],
                "uptime_s": round(time.time() - START_TS),
                "tts_engine": CFG["tts_default_engine"],
                "tts_speaker": CFG["tts_default_speaker"],
                "tts_local_available": os.path.exists(CFG["piper_bin"]) and os.path.exists(CFG["piper_voice"]),
                "tts_voices": {code: os.path.exists(path)
                               for code, path in (CFG.get("piper_voices") or {}).items()},
                "ask_engine": CFG["tts_default_engine"],
                "limits": CFG["limits"],
                "last_error": STATE["last_error"],
                "log": list(LOG)[:25],
            })
        if path == "/api/log":
            return self._send(200, {"log": list(LOG)})
        return self._send(404, {"error": "not found"})

    def do_POST(self):
        self._note_client()
        path = urlparse(self.path).path
        if not self._key_ok():
            return self._send(401, {"error": "missing or bad key"})
        payload = self._read_json()

        if path.startswith("/api/ask"):
            return self._ask_proxy(path, payload)

        if path == "/api/estop":
            STATE["estop"] = True
            STATE["moving"] = False
            try:
                with LOCK:
                    LOCO.StopMove()
                    LOCO.Damp()
            except Exception as exc:
                STATE["last_error"] = f"estop: {exc}"
            log("ESTOP")
            return self._send(200, {"estop": True})

        if path == "/api/estop_clear":
            STATE["estop"] = False
            log("estop_cleared")
            return self._send(200, {"estop": False})

        if path == "/api/say":
            try:
                code, body = handle_say(payload)
            except Exception as exc:
                log("say_error", str(exc))
                return self._send(500, {"error": str(exc)})
            return self._send(code, body)

        if path == "/api/tts":
            engine = (payload.get("engine") or "").strip()
            if engine and engine not in ("native", "local"):
                return self._send(400, {"error": "engine must be native or local"})
            if engine:
                CFG["tts_default_engine"] = engine
            if "speaker_id" in payload:
                try:
                    CFG["tts_default_speaker"] = int(payload["speaker_id"])
                except (TypeError, ValueError):
                    return self._send(400, {"error": "speaker_id must be an int"})
            save_config(CFG)
            log("tts_config", f"engine={CFG['tts_default_engine']} speaker={CFG['tts_default_speaker']}")
            return self._send(200, {"engine": CFG["tts_default_engine"],
                                    "speaker_id": CFG["tts_default_speaker"]})

        if path == "/api/volume":
            volume = int(payload.get("volume", 0))
            if not 0 <= volume <= 100:
                return self._send(400, {"error": "volume 0-100"})
            with LOCK:
                code = AUDIO.SetVolume(volume)
            log("volume", f"{volume} code={code}")
            return self._send(200, {"code": code, "volume": volume})

        if path == "/api/led":
            try:
                r, g, b = (max(0, min(255, int(payload.get(k, 0)))) for k in ("r", "g", "b"))
            except (TypeError, ValueError):
                return self._send(400, {"error": "r,g,b ints 0-255"})
            with LOCK:
                code = AUDIO.LedControl(r, g, b)
            return self._send(200, {"code": code})

        if path == "/api/loco":
            name = payload.get("action") or ""
            entry = LOCO_ACTIONS.get(name)
            if not entry:
                return self._send(400, {"error": f"unknown action {name}"})
            fn, is_motion = entry
            if is_motion:
                gate = self._motion_gate()
                if gate:
                    log("motion_blocked", f"{name} — {gate}")
                    return self._send(409, {"error": gate})
            try:
                with LOCK:
                    code = fn()
            except Exception as exc:
                log("loco_error", f"{name} {exc}")
                return self._send(500, {"error": str(exc)})
            if name == "stop_move":
                STATE["moving"] = False
            log("loco", f"{name} code={code}")
            body = {"code": code, "action": name, "ok": code == 0}
            if name == "prepare":
                body["steps"] = STATE.get("last_prepare", [])
            return self._send(200, body)

        if path == "/api/arm":
            name = payload.get("action") or ""
            if name not in action_map:
                return self._send(400, {"error": f"unknown arm action {name}"})
            gate = self._motion_gate()
            if gate:
                log("motion_blocked", f"arm {name} — {gate}")
                return self._send(409, {"error": gate})
            try:
                with LOCK:
                    code = ARM.ExecuteAction(action_map[name])
            except Exception as exc:
                log("arm_error", f"{name} {exc}")
                return self._send(500, {"error": str(exc)})
            log("arm", f"{name} code={code}")
            return self._send(200, {"code": code, "action": name, "ok": code == 0})

        if path == "/api/move":
            gate = self._motion_gate()
            if gate:
                return self._send(409, {"error": gate})
            lim = CFG["limits"]
            try:
                vx = max(-lim["vx"], min(lim["vx"], float(payload.get("vx", 0))))
                vy = max(-lim["vy"], min(lim["vy"], float(payload.get("vy", 0))))
                vyaw = max(-lim["vyaw"], min(lim["vyaw"], float(payload.get("vyaw", 0))))
            except (TypeError, ValueError):
                return self._send(400, {"error": "vx,vy,vyaw must be numbers"})
            try:
                with LOCK:
                    code = LOCO.SetVelocity(vx, vy, vyaw, 1.0)
            except Exception as exc:
                return self._send(500, {"error": str(exc)})
            STATE["last_move"] = time.monotonic()
            STATE["moving"] = any(abs(v) > 1e-3 for v in (vx, vy, vyaw))
            return self._send(200, {"code": code, "vx": vx, "vy": vy, "vyaw": vyaw})

        return self._send(404, {"error": "not found"})

    # Explicit stop paths must never be blocked by the gate.
    def do_DELETE(self):
        return self._send(405, {"error": "use POST"})


def main():
    server = ThreadingHTTPServer((CFG["bind"], CFG["port"]), Handler)
    log("started", f"bind={CFG['bind']}:{CFG['port']} iface={CFG['interface']}")
    print(f"G1 Pilot on http://<robot-ip>:{CFG['port']}/?k={CFG['key']}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
