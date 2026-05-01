"""
drawbot_server.py — WebSocket bridge entre le GUI web et le Drawbot ESP32.

Usage :
    pip install websockets
    python3 pc/drawbot_server.py AA:BB:CC:DD:EE:FF [channel]
    DRAWBOT_BT_ADDR=AA:BB:CC:DD:EE:FF python3 pc/drawbot_server.py

Le serveur expose ws://localhost:8765 et diffuse la télémétrie JSON toutes les 100ms.
Commandes entrantes : {"type":"cmd","cmd":"F:150"}
Télémétrie sortante : {"type":"telemetry","ts":...,"enc_r":...,"ax":...,...}
"""

import asyncio
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drawbot_client import DrawbotClient  # noqa: E402

import websockets  # pip install websockets

WS_HOST          = "localhost"
WS_PORT          = 8765
POLL_INTERVAL_S  = 0.10   # 100 ms → 10 Hz télémétrie
BT_TIMEOUT_S     = 0.30   # timeout par commande BT
RECONNECT_DELAY_S = 3.0

# ── Conversions raw int16 → unités physiques ──────────────────────────────────
ACCEL_SCALE = 0.000598   # m/s²  (0.061 mg/LSB × 9.81 m/s²/g × 1e-3 g/mg)
GYRO_SCALE  = 0.00875    # °/s   (8.75 mdps/LSB × 1e-3)
MAG_SCALE   = 0.1461     # mGauss (1/6842 gauss/LSB × 1000)


class RobotBridge:
    def __init__(self, bt_address: str, bt_channel: int):
        self.bt_address = bt_address
        self.bt_channel = bt_channel
        self.robot: DrawbotClient | None = None
        self.clients: set = set()
        self._bt_lock: asyncio.Lock | None = None  # créé dans le loop asyncio

    def _get_lock(self) -> asyncio.Lock:
        if self._bt_lock is None:
            self._bt_lock = asyncio.Lock()
        return self._bt_lock

    # ── Connexion BT (bloquant, exécuté dans executor) ────────────────────────
    def _connect_sync(self) -> DrawbotClient:
        return DrawbotClient(self.bt_address, self.bt_channel)

    async def connect(self):
        loop = asyncio.get_running_loop()
        self.robot = await loop.run_in_executor(None, self._connect_sync)
        print(f"[Bridge] Connecté à {self.bt_address}")

    # ── Envoi d'une commande BT + attente de la réponse (dans executor) ───────
    def _request_sync(self, cmd: str, prefix: str) -> str | None:
        try:
            self.robot.send(cmd)
            return self.robot.wait_for_response(prefix, timeout=BT_TIMEOUT_S)
        except (TimeoutError, ConnectionError, OSError):
            return None

    async def _bt_request(self, cmd: str, prefix: str) -> str | None:
        loop = asyncio.get_running_loop()
        async with self._get_lock():
            return await loop.run_in_executor(None, self._request_sync, cmd, prefix)

    # ── Envoi sans attente de réponse (commandes de pilotage) ─────────────────
    def _send_sync(self, cmd: str):
        try:
            self.robot.send(cmd)
        except (ConnectionError, OSError):
            pass

    async def _bt_send(self, cmd: str):
        loop = asyncio.get_running_loop()
        async with self._get_lock():
            await loop.run_in_executor(None, self._send_sync, cmd)

    # ── Parseurs ──────────────────────────────────────────────────────────────
    @staticmethod
    def _parse_enc(line: str) -> dict:
        try:
            _, vals = line.split(":", 1)
            r, l = vals.split(",", 1)
            return {"enc_r": int(r), "enc_l": int(l)}
        except Exception:
            return {}

    @staticmethod
    def _parse_imu(line: str) -> dict:
        try:
            _, vals = line.split(":", 1)
            ax, ay, az, gx, gy, gz = [int(v) for v in vals.split(",")]
            return {
                "ax": round(ax * ACCEL_SCALE, 4),
                "ay": round(ay * ACCEL_SCALE, 4),
                "az": round(az * ACCEL_SCALE, 4),
                "gx": round(gx * GYRO_SCALE,  4),
                "gy": round(gy * GYRO_SCALE,  4),
                "gz": round(gz * GYRO_SCALE,  4),
            }
        except Exception:
            return {}

    @staticmethod
    def _parse_mag(line: str) -> dict:
        try:
            _, vals = line.split(":", 1)
            mx, my, mz = [int(v) for v in vals.split(",")]
            return {
                "mx": round(mx * MAG_SCALE, 3),
                "my": round(my * MAG_SCALE, 3),
                "mz": round(mz * MAG_SCALE, 3),
            }
        except Exception:
            return {}

    # ── Poll complet : ENC + IMU + MAG ────────────────────────────────────────
    async def poll_once(self) -> dict:
        data: dict = {"type": "telemetry", "ts": time.time()}

        enc = await self._bt_request("ENC", "ENC:")
        if enc:
            data.update(self._parse_enc(enc))

        imu = await self._bt_request("IMU", "IMU:")
        if imu:
            data.update(self._parse_imu(imu))

        mag = await self._bt_request("MAG", "MAG:")
        if mag:
            data.update(self._parse_mag(mag))

        return data

    # ── Broadcast JSON vers tous les clients WS ───────────────────────────────
    async def broadcast(self, msg: dict):
        if not self.clients:
            return
        payload = json.dumps(msg)
        await asyncio.gather(
            *[ws.send(payload) for ws in self.clients],
            return_exceptions=True,
        )

    # ── Boucle télémétrie principale ──────────────────────────────────────────
    async def telemetry_loop(self):
        while True:
            if self.robot and self.robot.connected:
                try:
                    data = await self.poll_once()
                    await self.broadcast(data)
                except Exception as e:
                    print(f"[Bridge] Erreur poll : {e}")
                    await self.broadcast({"type": "status", "connected": False})
            else:
                await self.broadcast({"type": "status", "connected": False})
                print(f"[Bridge] BT déconnecté — reconnexion dans {RECONNECT_DELAY_S}s")
                await asyncio.sleep(RECONNECT_DELAY_S)
                try:
                    await self.connect()
                    await self.broadcast({"type": "status", "connected": True})
                except Exception as e:
                    print(f"[Bridge] Échec reconnexion : {e}")
                    continue

            await asyncio.sleep(POLL_INTERVAL_S)

    # ── Handler WebSocket par client ──────────────────────────────────────────
    async def handle_client(self, ws):
        self.clients.add(ws)
        await ws.send(json.dumps({
            "type": "status",
            "connected": self.robot is not None and self.robot.connected,
        }))
        print(f"[WS] Client connecté ({len(self.clients)} total)")
        try:
            async for raw in ws:
                await self._handle_ws_message(raw)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            print(f"[WS] Client déconnecté ({len(self.clients)} restants)")

    async def _handle_ws_message(self, raw: str):
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        if msg.get("type") != "cmd":
            return
        cmd_str = str(msg.get("cmd", "")).strip()
        if not cmd_str:
            return
        if self.robot and self.robot.connected:
            await self._bt_send(cmd_str)
        else:
            print(f"[Bridge] Commande ignorée (BT déconnecté) : {cmd_str}")


# ── Résolution de l'adresse BT ────────────────────────────────────────────────
def resolve_args():
    bt_address = os.environ.get("DRAWBOT_BT_ADDR")
    bt_channel = 1
    if len(sys.argv) >= 2 and ":" in sys.argv[1]:
        bt_address = sys.argv[1]
        if len(sys.argv) >= 3:
            bt_channel = int(sys.argv[2])
    elif bt_address and len(sys.argv) >= 2:
        try:
            bt_channel = int(sys.argv[1])
        except ValueError:
            pass
    return bt_address, bt_channel


def main():
    bt_address, bt_channel = resolve_args()
    if not bt_address:
        print("Usage : python3 pc/drawbot_server.py AA:BB:CC:DD:EE:FF [channel]")
        print("        DRAWBOT_BT_ADDR=AA:BB:CC:DD:EE:FF python3 pc/drawbot_server.py")
        sys.exit(1)

    bridge = RobotBridge(bt_address, bt_channel)

    async def run():
        await bridge.connect()
        print(f"[Bridge] Serveur WebSocket sur ws://{WS_HOST}:{WS_PORT}")
        async with websockets.serve(bridge.handle_client, WS_HOST, WS_PORT):
            await bridge.telemetry_loop()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n[Bridge] Arrêt.")


if __name__ == "__main__":
    main()
