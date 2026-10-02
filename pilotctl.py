#!/usr/bin/env python3
"""CLI for the local G1 Pilot service.

Reads the access key from ~/g1-pilot/config.json, so no key ever appears in a
command line. Motion commands need an explicit --arm flag (mirrors the UI gate).

  ./pilotctl.py state
  ./pilotctl.py say "Hello" [native|local]
  ./pilotctl.py voice native|local          # saved on the robot, used for all replies
  ./pilotctl.py speaker 0-3
  ./pilotctl.py vol 100
  ./pilotctl.py led 0 255 0
  ./pilotctl.py loco sit --arm
  ./pilotctl.py arm "high five" --arm
  ./pilotctl.py move 0.2 0 0 --arm
  ./pilotctl.py estop | clear

DJI voice-ask (mic backend on the Mac):
  ./pilotctl.py ask-status
  ./pilotctl.py ask "What is the capital of France?"   # typed question
  ./pilotctl.py ask-mic                               # record from the DJI mic, answer, speak
  ./pilotctl.py ask-kb [status|list]
  ./pilotctl.py ask-kb-add "title" "text"
"""
import json
import os
import sys
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "config.json")) as fh:
    CFG = json.load(fh)
BASE = f"http://127.0.0.1:{CFG.get('port', 8130)}"

USAGE = __doc__


def call(path, body=None, arm=False):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data,
                                method="POST" if body is not None else "GET")
    req.add_header("X-Pilot-Key", CFG["key"])
    if body is not None:
        req.add_header("Content-Type", "application/json")
    if arm:
        req.add_header("X-Pilot-Motion", "clear")
    try:
        with urllib.request.urlopen(req, timeout=35) as resp:
            return json.loads(resp.read() or "{}")
    except urllib.error.HTTPError as exc:
        return {"http_error": exc.code, "body": exc.read().decode(errors="replace")}
    except urllib.error.URLError as exc:
        return {"error": str(exc)}


def main(argv):
    if not argv:
        print(USAGE)
        return 2
    args = [a for a in argv if a != "--arm"]
    arm = "--arm" in argv
    cmd, rest = args[0], args[1:]

    if cmd == "state":
        out = call("/api/state")
        out.pop("log", None)
        print(json.dumps(out, indent=1))
    elif cmd == "say":
        payload = {"text": rest[0]}
        if len(rest) > 1:
            payload["engine"] = rest[1]
        print(call("/api/say", payload))
    elif cmd == "voice":
        print(call("/api/tts", {"engine": rest[0]}))
    elif cmd == "speaker":
        print(call("/api/tts", {"speaker_id": int(rest[0])}))
    elif cmd == "vol":
        print(call("/api/volume", {"volume": int(rest[0])}))
    elif cmd == "led":
        print(call("/api/led", {"r": int(rest[0]), "g": int(rest[1]), "b": int(rest[2])}))
    elif cmd == "loco":
        print(call("/api/loco", {"action": rest[0]}, arm=arm))
    elif cmd == "arm":
        print(call("/api/arm", {"action": rest[0]}, arm=arm))
    elif cmd == "move":
        print(call("/api/move", {"vx": float(rest[0]), "vy": float(rest[1]), "vyaw": float(rest[2])}, arm=arm))
    elif cmd == "estop":
        print(call("/api/estop", {}))
    elif cmd == "clear":
        print(call("/api/estop_clear", {}))
    elif cmd == "ask-status":
        print(json.dumps(call("/api/ask/status"), indent=1))
    elif cmd == "ask":
        print(json.dumps(call("/api/ask_text", {"question": rest[0]}), indent=1))
    elif cmd == "ask-mic":
        print(json.dumps(call("/api/ask", {"mode": "auto"}), indent=1))
    elif cmd == "ask-kb":
        out = call("/api/ask/kb")
        entries = out.get("entries")
        print(json.dumps(entries if entries is not None else out, indent=1))
    elif cmd == "ask-kb-add":
        payload = {"title": rest[0], "text": " ".join(rest[1:])} if len(rest) > 1 else {"title": "", "text": " ".join(rest)}
        print(call("/api/ask/kb", payload))
    else:
        print(USAGE)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
