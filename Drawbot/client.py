import socket
import time

def connecter(ip: str, port: int = 8266):
    print(f"Connexion sur {ip}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    sock.settimeout(2)
    print("Connecté !\n")
    return sock

def envoyer_commande(sock, cmd: str):
    sock.sendall((cmd + '\n').encode())
    time.sleep(0.1)
    try:
        response = sock.recv(1024).decode('utf-8', errors='ignore').strip()
        if response:
            print(f"ESP32 : {response}")
    except (socket.timeout, ConnectionResetError, OSError):
        pass

def menu():
    print("\n--- DRAWBOT CONTROLLER ---")
    print("  F <cm>  : Avancer")
    print("  B <cm>  : Reculer")
    print("  L <deg> : Tourner gauche")
    print("  R <deg> : Tourner droite")
    print("  S       : Stop")
    print("  E       : Lire encodeurs")
    print("  Q       : Quitter")
    print("--------------------------")

def main():
    ip = input("IP de l'ESP32 : ").strip()
    try:
        sock = connecter(ip)
    except Exception as e:
        print(f"Erreur : {e}")
        return

    menu()

    try:
        while True:
            entree = input("Commande > ").strip().split()
            if not entree:
                continue

            cmd = entree[0].upper()
            args = entree[1:]

            if cmd == 'Q':
                print("Déconnexion.")
                try: envoyer_commande(sock, 'S')
                except: pass
                break
            elif cmd == 'F':
                val = float(args[0]) if args else 20.0
                envoyer_commande(sock, f"F:{val}")
            elif cmd == 'B':
                val = float(args[0]) if args else 20.0
                envoyer_commande(sock, f"B:{val}")
            elif cmd == 'L':
                val = float(args[0]) if args else 90.0
                envoyer_commande(sock, f"L:{val}")
            elif cmd == 'R':
                val = float(args[0]) if args else 90.0
                envoyer_commande(sock, f"R:{val}")
            elif cmd == 'S':
                envoyer_commande(sock, 'S')
            elif cmd == 'E':
                envoyer_commande(sock, 'E')
            else:
                print("Commande inconnue.")

    except KeyboardInterrupt:
        print("\nArrêt.")
        try: envoyer_commande(sock, 'S')
        except: pass
    finally:
        try: sock.close()
        except: pass

if __name__ == "__main__":
    main()