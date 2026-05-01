"""
Client de controle Drawbot
Connexion Bluetooth RFCOMM vers l'ESP32 en Bluetooth classique (SPP).

Usage :
  python3 drawbot_client.py <bt_address> [channel]
  DRAWBOT_BT_ADDR=AA:BB:CC:DD:EE:FF python3 drawbot_client.py [channel]

Exemple :
  python3 drawbot_client.py AA:BB:CC:DD:EE:FF 1
"""

import os
import queue
import socket
import sys
import threading
import time
import math

DEFAULT_BT_CHANNEL = 1
WHEEL_DIAMETER_CM = 9.0
TICKS_PER_WHEEL_REV = 2008
WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM
DEFAULT_DISTANCE_SPEED = 120
DEFAULT_DISTANCE_TIMEOUT_S = 15.0


def resolve_cli_args():
    env_address = os.environ.get("DRAWBOT_BT_ADDR")

    if len(sys.argv) > 2:
        return sys.argv[1], int(sys.argv[2])
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if ":" in arg:
            return arg, DEFAULT_BT_CHANNEL
        if env_address:
            return env_address, int(arg)

    if env_address:
        return env_address, DEFAULT_BT_CHANNEL

    return None, DEFAULT_BT_CHANNEL


BT_ADDRESS, BT_CHANNEL = resolve_cli_args()


class DrawbotClient:
    def __init__(self, bt_address: str, channel: int):
        if not hasattr(socket, "AF_BLUETOOTH"):
            raise OSError("Bluetooth sockets are not supported on this Python build")

        self.sock = socket.socket(
            socket.AF_BLUETOOTH,
            socket.SOCK_STREAM,
            socket.BTPROTO_RFCOMM,
        )
        self.sock.settimeout(10)
        self.sock.connect((bt_address, channel))
        self.sock.settimeout(None)
        self._file = self.sock.makefile("r")
        self.connected = True
        self._responses = queue.Queue()

        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()
        print(f"[OK] Connecte a {bt_address} sur le canal RFCOMM {channel}")

    def send(self, cmd: str) -> None:
        """Envoyer une commande (ajoute \n automatiquement)."""
        if not self.connected:
            raise ConnectionError("Connexion Bluetooth deja fermee")

        try:
            self.sock.sendall((cmd.strip() + "\n").encode())
        except OSError as exc:
            self.connected = False
            raise ConnectionError("Connexion Bluetooth perdue") from exc

    def _receive_loop(self):
        try:
            for raw_line in self._file:
                line = raw_line.strip()
                print(f"[ESP32] {line}")
                if line.startswith("OK:MOVE:"):
                    try:
                        parts = line.split(":")
                        ticks = int(parts[4])
                        estimated_cm = (ticks / TICKS_PER_WHEEL_REV) * WHEEL_CIRCUMFERENCE_CM
                        print(f"[INFO] Distance estimee : {estimated_cm:.2f} cm ({ticks} ticks)")
                    except (IndexError, ValueError):
                        pass
                self._responses.put(line)
        except Exception:
            print("[INFO] Connexion Bluetooth fermee.")
        finally:
            self.connected = False

    def close(self):
        try:
            self.send("S")
        except (OSError, ConnectionError):
            pass
        self.sock.close()

    def wait_for_response(self, prefixes, timeout: float = 1.0) -> str:
        if isinstance(prefixes, str):
            prefixes = (prefixes,)

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Aucune reponse recue pour {prefixes}")

            try:
                line = self._responses.get(timeout=remaining)
            except queue.Empty as exc:
                raise TimeoutError(f"Aucune reponse recue pour {prefixes}") from exc

            if any(line.startswith(prefix) for prefix in prefixes):
                return line

    def request_encoders(self, timeout: float = 1.0):
        self.send("ENC")
        response = self.wait_for_response("ENC:", timeout=timeout)
        _, values = response.split(":", 1)
        right_ticks, left_ticks = values.split(",", 1)
        return int(right_ticks), int(left_ticks)

    def run_for_ms(self, speed_d: int, speed_g: int, duration_ms: int):
        speed_d = max(-255, min(int(speed_d), 255))
        speed_g = max(-255, min(int(speed_g), 255))
        duration_ms = max(0, int(duration_ms))
        self.send(f"RUN_MS:{speed_d}:{speed_g}:{duration_ms}")

    def run_for_ticks(self, speed_d: int, speed_g: int, target_ticks: int):
        speed_d = max(-255, min(int(speed_d), 255))
        speed_g = max(-255, min(int(speed_g), 255))
        target_ticks = max(0, int(target_ticks))
        self.send(f"RUN_TICKS:{speed_d}:{speed_g}:{target_ticks}")

    def timed_forward(self, speed: int, duration_ms: int):
        self.run_for_ms(speed, speed, duration_ms)

    def timed_backward(self, speed: int, duration_ms: int):
        self.run_for_ms(-speed, -speed, duration_ms)

    def timed_left(self, speed: int, duration_ms: int):
        self.run_for_ms(speed, -speed, duration_ms)

    def timed_right(self, speed: int, duration_ms: int):
        self.run_for_ms(-speed, speed, duration_ms)

    def drive_distance(self, distance_cm: float, ticks_per_wheel_rev: float, speed: int = DEFAULT_DISTANCE_SPEED):
        wheel_circumference_cm = math.pi * WHEEL_DIAMETER_CM
        target_ticks = abs(distance_cm) * (ticks_per_wheel_rev / wheel_circumference_cm)
        if target_ticks == 0:
            self.stop()
            return

        speed = max(0, min(abs(speed), 255))
        direction = 1 if distance_cm >= 0 else -1

        print(
            f"[INFO] Distance cible: {distance_cm:.2f} cm | "
            f"circonference roue: {wheel_circumference_cm:.2f} cm | "
            f"objectif: {target_ticks:.1f} ticks"
        )
        if direction > 0:
            self.run_for_ticks(speed, speed, round(target_ticks))
        else:
            self.run_for_ticks(-speed, -speed, round(target_ticks))

        timeout_s = max(DEFAULT_DISTANCE_TIMEOUT_S, abs(distance_cm) * 0.6)
        self.wait_for_response("OK:RUN_TICKS_DONE", timeout=timeout_s)

        final_right, final_left = self.request_encoders(timeout=1.0)
        final_ticks = (abs(final_right) + abs(final_left)) / 2.0
        final_distance_cm = (final_ticks / ticks_per_wheel_rev) * wheel_circumference_cm

        print(
            f"[OK] Deplacement termine | ticks moyens: {final_ticks:.1f} | "
            f"distance estimee: {final_distance_cm:.2f} cm"
        )

    def corner_left(self, speed: int = DEFAULT_DISTANCE_SPEED):
        """Angle droit gauche parfait — appeler quand le stylo est au coin."""
        self.send(f"CORNER_LEFT:{speed}")
        self.wait_for_response("OK:CORNER_DONE", timeout=20.0)

    def corner_right(self, speed: int = DEFAULT_DISTANCE_SPEED):
        """Angle droit droite parfait — appeler quand le stylo est au coin."""
        self.send(f"CORNER_RIGHT:{speed}")
        self.wait_for_response("OK:CORNER_DONE", timeout=20.0)

    # ── Commandes haut niveau ────────────────────────────────────────────────
    def forward(self, speed: int = 80):   self.send(f"F:{speed}")
    def backward(self, speed: int = 80):  self.send(f"B:{speed}")
    def left(self, speed: int = 80):      self.send(f"L:{speed}")
    def right(self, speed: int = 80):     self.send(f"R:{speed}")
    def stop(self):                        self.send("S")
    def ping(self):                        self.send("PING")
    def read_encoders(self):               self.send("ENC")
    def reset_encoders(self):              self.send("RESET_ENC")


HELP = """
Commandes disponibles :
  f <speed>   -> avancer          (ex: f 200)
  b <speed>   -> reculer
  l <speed>   -> tourner gauche
  r <speed>   -> tourner droite
  tm <f|b|l|r> <speed> <ms> -> mouvement temporise gere par l'ESP32
  run <speed_d> <speed_g> <ms> -> version brute roue droite / roue gauche
  dist <cm> [speed] -> distance executee par l'ESP32 avec encodeurs
  cl [speed]  -> angle droit gauche (stylo au coin avant d'appeler)
  cr [speed]  -> angle droit droite
  s           -> stop
  enc         -> lire encodeurs
  reset       -> reset encodeurs
  ping        -> ping
  q           -> quitter
  (toute autre ligne est envoyee telle quelle a l'ESP32)
"""


def print_usage():
    print("Usage :")
    print("  python3 pc/drawbot_client.py <bt_address> [channel]")
    print("  DRAWBOT_BT_ADDR=AA:BB:CC:DD:EE:FF python3 pc/drawbot_client.py [channel]")
    print("Exemple : python3 pc/drawbot_client.py AA:BB:CC:DD:EE:FF 1")


def resolve_ticks_per_wheel_rev():
    env_value = os.environ.get("DRAWBOT_TICKS_PER_WHEEL_REV")
    if env_value:
        return float(env_value)
    return None


def main():
    if not BT_ADDRESS:
        print("[ERREUR] Adresse Bluetooth manquante.")
        print_usage()
        print("Astuce : utilisez 'bluetoothctl devices' pour retrouver l'adresse MAC du robot apres appairage.")
        return

    try:
        bot = DrawbotClient(BT_ADDRESS, BT_CHANNEL)
    except (ConnectionRefusedError, OSError) as e:
        print(f"[ERREUR] Impossible de se connecter a {BT_ADDRESS} sur le canal {BT_CHANNEL} -> {e}")
        print("Verifiez que :")
        print("  1. Le robot est alimente et le firmware Bluetooth est televerse")
        print("  2. Le PC est appaire avec l'ESP32 'Drawbot'")
        print("  3. L'adresse MAC et le canal RFCOMM sont corrects (canal 1 par defaut)")
        return

    print(HELP)

    ticks_per_wheel_rev = resolve_ticks_per_wheel_rev()
    if ticks_per_wheel_rev is None:
        print("[INFO] Commande 'dist' inactive tant que DRAWBOT_TICKS_PER_WHEEL_REV n'est pas defini.")
    else:
        print(f"[INFO] Calibration distance activee avec {ticks_per_wheel_rev} ticks/tour de roue.")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd == "q":
                break
            elif cmd == "f":
                bot.forward(int(parts[1]) if len(parts) > 1 else 80)
            elif cmd == "b":
                bot.backward(int(parts[1]) if len(parts) > 1 else 80)
            elif cmd == "l":
                bot.left(int(parts[1]) if len(parts) > 1 else 80)
            elif cmd == "r":
                bot.right(int(parts[1]) if len(parts) > 1 else 80)
            elif cmd == "dist":
                if ticks_per_wheel_rev is None:
                    print("[ERREUR] Definis DRAWBOT_TICKS_PER_WHEEL_REV pour utiliser 'dist'.")
                    continue
                if len(parts) < 2:
                    print("[ERREUR] Usage : dist <cm> [speed]")
                    continue
                distance_cm = float(parts[1])
                speed = int(parts[2]) if len(parts) > 2 else DEFAULT_DISTANCE_SPEED
                bot.drive_distance(distance_cm, ticks_per_wheel_rev, speed=speed)
            elif cmd == "tm":
                if len(parts) != 4:
                    print("[ERREUR] Usage : tm <f|b|l|r> <speed> <ms>")
                    continue
                direction = parts[1].lower()
                speed = int(parts[2])
                duration_ms = int(parts[3])
                if direction == "f":
                    bot.timed_forward(speed, duration_ms)
                elif direction == "b":
                    bot.timed_backward(speed, duration_ms)
                elif direction == "l":
                    bot.timed_left(speed, duration_ms)
                elif direction == "r":
                    bot.timed_right(speed, duration_ms)
                else:
                    print("[ERREUR] Direction invalide. Utilise f, b, l ou r.")
            elif cmd == "cl":
                speed = int(parts[1]) if len(parts) > 1 else DEFAULT_DISTANCE_SPEED
                bot.corner_left(speed)
            elif cmd == "cr":
                speed = int(parts[1]) if len(parts) > 1 else DEFAULT_DISTANCE_SPEED
                bot.corner_right(speed)
            elif cmd == "run":
                if len(parts) != 4:
                    print("[ERREUR] Usage : run <speed_d> <speed_g> <ms>")
                    continue
                bot.run_for_ms(int(parts[1]), int(parts[2]), int(parts[3]))
            elif cmd == "s":
                bot.stop()
            elif cmd == "enc":
                bot.read_encoders()
            elif cmd == "reset":
                bot.reset_encoders()
            elif cmd == "ping":
                bot.ping()
            else:
                bot.send(line)

    except (KeyboardInterrupt, ConnectionError):
        pass
    finally:
        bot.close()
        print("Deconnecte.")


if __name__ == "__main__":
    main()
