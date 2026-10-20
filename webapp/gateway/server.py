"""Glaucus I web gateway. Runs on the topside machine; phones connect over Wi-Fi.

    pip install -r webapp/requirements.txt
    python webapp/gateway/server.py --mock          # no robot needed (runs the simulator)
    python webapp/gateway/server.py --ros           # real robot (source ROS 2 first)
    then open http://<topside-ip>:8000 on a phone on the topside Wi-Fi

API (also what a future native app would use):
    GET  /api/status           current status snapshot (JSON)
    GET  /api/hull             hull outline + lane plan, for drawing
    GET  /api/coverage         downsampled cleaned/not-cleaned grid
    POST /api/mission/start
    POST /api/mission/pause    {"paused": true|false}
    POST /api/mission/stop
    POST /api/estop
    WS   /ws                   pushes {"status": ...} at 5 Hz and {"coverage": ...} at 1 Hz

DESIGN CHOICES
  - Plain web app served from here (no app store, works on any phone/laptop,
    one codebase). Wrap it in Capacitor later if you truly need a native app.
  - WebSocket push for live state, plain POST for commands (easy to retry,
    easy to curl from a terminal when debugging).
  - E-stop is its own endpoint and never goes through mission logic.
  - No auth yet. The topside Wi-Fi is the boundary. TODO(glaucus): a shared
    PIN before any public demo where strangers can join the network.
"""
from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

FRONTEND = Path(__file__).resolve().parents[1] / "frontend"


class PauseBody(BaseModel):
    paused: bool


def create_app(backend) -> FastAPI:
    app = FastAPI(title="Glaucus I gateway")

    def reply(result: tuple[bool, str]):
        ok, msg = result
        return JSONResponse({"ok": ok, "message": msg}, status_code=200 if ok else 409)

    @app.get("/api/status")
    def status():
        return backend.status()

    @app.get("/api/hull")
    def hull():
        return backend.hull()

    @app.get("/api/coverage")
    def coverage():
        return backend.coverage()

    @app.post("/api/mission/start")
    def start():
        return reply(backend.start())

    @app.post("/api/mission/pause")
    def pause(body: PauseBody):
        return reply(backend.pause(body.paused))

    @app.post("/api/mission/stop")
    def stop():
        return reply(backend.abort())

    @app.post("/api/estop")
    def estop():
        return reply(backend.estop())

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        tick = 0
        try:
            while True:
                await sock.send_json({"status": backend.status()})
                if tick % 5 == 0:
                    await sock.send_json({"coverage": backend.coverage()})
                tick += 1
                await asyncio.sleep(0.2)
        except (WebSocketDisconnect, RuntimeError):
            pass

    @app.get("/")
    def index():
        return FileResponse(FRONTEND / "index.html")

    app.mount("/", StaticFiles(directory=FRONTEND), name="static")
    return app


def main() -> None:
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--mock", action="store_true", help="drive the app from the simulator (default)")
    mode.add_argument("--ros", action="store_true", help="bridge to the real robot via ROS 2")
    ap.add_argument("--speed", type=float, default=4.0, help="mock: sim speed multiplier")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()

    from backends import MockBackend, RosBackend
    backend = RosBackend() if a.ros else MockBackend(speed=a.speed)
    uvicorn.run(create_app(backend), host=a.host, port=a.port, log_level="info")


if __name__ == "__main__":
    main()
