import socket
import threading
import tkinter as tk
from tkinter import ttk, messagebox

TCP_PORT = 8266
SEQ1_TOTAL_TIMEOUT_S = 30   # duree max de toute la sequence escalier (cote ESP32)
SEQ2_TOTAL_TIMEOUT_S = 20   # duree max pour le cercle (T=10s + marge)


class DrawbotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drawbot Controller")
        self.root.resizable(False, False)

        self.sock = None
        self.connected = False

        # Synchronisation de la sequence : l'evenement est declenche
        # quand l'ESP32 envoie ">> DONE" (fin d'un mouvement asservi).
        self._done_event = threading.Event()
        self._seq_running = False
        self._seq_abort = False

        self._build_ui()
        self._poll_encoders()

    # ------------------------------------------------------------------
    # Construction de l'interface
    # ------------------------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 10, "pady": 6}

        conn_frame = ttk.LabelFrame(self.root, text="Connexion WiFi")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", **pad)

        ttk.Label(conn_frame, text="IP de l'ESP32 :").grid(row=0, column=0, padx=6, pady=4)
        self.ip_var = tk.StringVar()
        self.ip_entry = ttk.Entry(conn_frame, textvariable=self.ip_var, width=18)
        self.ip_entry.grid(row=0, column=1, padx=6, pady=4)

        self.conn_btn = ttk.Button(conn_frame, text="Connecter", command=self._toggle_connection)
        self.conn_btn.grid(row=0, column=2, padx=6, pady=4)

        self.status_lbl = ttk.Label(conn_frame, text="Déconnecté", foreground="gray")
        self.status_lbl.grid(row=0, column=3, padx=10)

        move_frame = ttk.LabelFrame(self.root, text="Déplacement")
        move_frame.grid(row=1, column=0, sticky="nsew", **pad)

        ttk.Label(move_frame, text="Distance (cm) / angle (°) :").grid(row=0, column=0, columnspan=3, pady=(6, 2))
        self.val_var = tk.DoubleVar(value=20.0)
        ttk.Spinbox(move_frame, from_=1, to=360, textvariable=self.val_var,
                    width=8, format="%.1f", increment=1).grid(row=1, column=0, columnspan=3, pady=2)

        btn_cfg = {"width": 10}
        ttk.Button(move_frame, text="↑  Avant",   command=lambda: self._move("F"), **btn_cfg).grid(row=2, column=1, pady=2)
        ttk.Button(move_frame, text="←  Gauche",  command=lambda: self._move("L"), **btn_cfg).grid(row=3, column=0, padx=4)
        ttk.Button(move_frame, text="■  STOP",    command=self._stop,              **btn_cfg).grid(row=3, column=1, padx=4)
        ttk.Button(move_frame, text="→  Droite",  command=lambda: self._move("R"), **btn_cfg).grid(row=3, column=2, padx=4)
        ttk.Button(move_frame, text="↓  Arrière", command=lambda: self._move("B"), **btn_cfg).grid(row=4, column=1, pady=2)

        self.root.bind("<Up>",    lambda e: self._move("F"))
        self.root.bind("<Down>",  lambda e: self._move("B"))
        self.root.bind("<Left>",  lambda e: self._move("L"))
        self.root.bind("<Right>", lambda e: self._move("R"))
        self.root.bind("<space>", lambda e: self._stop())

        seq_frame = ttk.LabelFrame(self.root, text="Séquences")
        seq_frame.grid(row=1, column=1, sticky="nsew", **pad)

        # Bouton Séquence 1
        self.seq1_btn = ttk.Button(seq_frame, text="Séq. 1 — Escalier", width=22,
                                   command=self._seq_escalier)
        self.seq1_btn.grid(row=0, column=0, pady=(6, 2), padx=8, sticky="ew")

        # Sous-cadre pour Séquence 2 (Cercle avec paramètre rayon)
        seq2_frame = ttk.Frame(seq_frame)
        seq2_frame.grid(row=1, column=0, pady=(2, 6), padx=8, sticky="ew")
        
        self.seq2_btn = ttk.Button(seq2_frame, text="Séq. 2 — Cercle", width=13,
                                   command=self._seq_cercle)
        self.seq2_btn.pack(side="left", padx=(0, 4))
        
        ttk.Label(seq2_frame, text="R=").pack(side="left")
        self.rayon_var = tk.DoubleVar(value=2.0)
        ttk.Spinbox(seq2_frame, from_=0.5, to=20.0, textvariable=self.rayon_var,
                    width=4, format="%.1f", increment=0.5).pack(side="left")
        ttk.Label(seq2_frame, text="cm").pack(side="left")

        enc_frame = ttk.LabelFrame(self.root, text="Encodeurs")
        enc_frame.grid(row=2, column=0, sticky="ew", **pad)

        ttk.Label(enc_frame, text="Gauche :").grid(row=0, column=0, padx=6, pady=4)
        self.enc_g_lbl = ttk.Label(enc_frame, text="—", width=8, anchor="e")
        self.enc_g_lbl.grid(row=0, column=1)

        ttk.Label(enc_frame, text="Droit :").grid(row=0, column=2, padx=6)
        self.enc_d_lbl = ttk.Label(enc_frame, text="—", width=8, anchor="e")
        self.enc_d_lbl.grid(row=0, column=3)

        ttk.Button(enc_frame, text="Lire", command=self._read_enc, width=6).grid(row=0, column=4, padx=6)

        # --- Réglage du PID de virage (gyro), modifiable en direct ---
        tune_frame = ttk.LabelFrame(self.root, text="Réglage virage (gyro / PID)")
        tune_frame.grid(row=2, column=1, sticky="nsew", **pad)

        self.kp_var = tk.DoubleVar(value=4.0)
        self.ki_var = tk.DoubleVar(value=0.10)
        self.kd_var = tk.DoubleVar(value=0.20)

        ttk.Label(tune_frame, text="Kp").grid(row=0, column=0, padx=2, pady=2)
        ttk.Entry(tune_frame, textvariable=self.kp_var, width=6).grid(row=0, column=1, padx=2)
        ttk.Label(tune_frame, text="Ki").grid(row=0, column=2, padx=2)
        ttk.Entry(tune_frame, textvariable=self.ki_var, width=6).grid(row=0, column=3, padx=2)
        ttk.Label(tune_frame, text="Kd").grid(row=0, column=4, padx=2)
        ttk.Entry(tune_frame, textvariable=self.kd_var, width=6).grid(row=0, column=5, padx=2)

        ttk.Button(tune_frame, text="Appliquer PID", command=self._apply_pid).grid(
            row=1, column=0, columnspan=2, pady=4, padx=2, sticky="ew")
        ttk.Button(tune_frame, text="Calibrer gyro", command=lambda: self._send("CAL")).grid(
            row=1, column=2, columnspan=2, pady=4, padx=2, sticky="ew")
        ttk.Button(tune_frame, text="Lire gyro", command=lambda: self._send("G")).grid(
            row=1, column=4, columnspan=2, pady=4, padx=2, sticky="ew")

        ttk.Button(tune_frame, text="Test 90° ←", command=lambda: self._send("L:90.0")).grid(
            row=2, column=0, columnspan=3, pady=(0, 4), padx=2, sticky="ew")
        ttk.Button(tune_frame, text="Test 90° →", command=lambda: self._send("R:90.0")).grid(
            row=2, column=3, columnspan=3, pady=(0, 4), padx=2, sticky="ew")

        log_frame = ttk.LabelFrame(self.root, text="Console")
        log_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=10, pady=(0, 6))
        log_frame.grid_columnconfigure(0, weight=1)

        self.log = tk.Text(log_frame, height=9, state="disabled",
                           font=("Courier", 10), wrap="word")
        self.log.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=4)

        scroll = ttk.Scrollbar(log_frame, command=self.log.yview)
        scroll.grid(row=0, column=2, sticky="ns")
        self.log["yscrollcommand"] = scroll.set

        # Champ de commande brute (utile pour les tests / la demo)
        self.raw_var = tk.StringVar()
        raw_entry = ttk.Entry(log_frame, textvariable=self.raw_var)
        raw_entry.grid(row=1, column=0, sticky="ew", padx=4, pady=(0, 4))
        raw_entry.bind("<Return>", lambda e: self._send_raw())
        ttk.Button(log_frame, text="Envoyer", command=self._send_raw,
                   width=8).grid(row=1, column=1, padx=4, pady=(0, 4))

    # ------------------------------------------------------------------
    # Connexion
    # ------------------------------------------------------------------
    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        ip = self.ip_var.get().strip()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect((ip, TCP_PORT))
            s.settimeout(2)
            self.sock = s
            self.connected = True
            self.status_lbl.config(text=f"Connecté — {ip}", foreground="green")
            self.conn_btn.config(text="Déconnecter")
            self._set_controls_state("normal")
            self._log(f"[OK] Connecté à {ip}:{TCP_PORT}")
            threading.Thread(target=self._rx_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Connexion", f"Impossible de se connecter :\n{e}")

    def _disconnect(self):
        self.connected = False
        self._seq_abort = True
        self._done_event.set()      # libere une eventuelle sequence en attente
        try:
            self._send_raw_cmd("S")
            self.sock.close()
        except Exception:
            pass
        self.sock = None
        self.status_lbl.config(text="Déconnecté", foreground="gray")
        self.conn_btn.config(text="Connecter")
        self._set_controls_state("disabled")
        self._log("[INFO] Déconnecté.")

    # ------------------------------------------------------------------
    # Reception
    # ------------------------------------------------------------------
    def _rx_loop(self):
        buf = ""
        while self.connected:
            try:
                data = self.sock.recv(1024).decode("utf-8", errors="ignore")
                if not data:
                    break
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        self.root.after(0, self._handle_rx, line)
            except socket.timeout:
                continue
            except Exception as e:
                self.root.after(0, self._log, f"[RX ERR] {e}")
                break
        self.root.after(0, self._on_disconnect)

    def _handle_rx(self, line):
        self._log(f"ESP32 : {line}")

        # Fin d'un mouvement asservi : debloque la sequence en cours
        if ">> DONE" in line:
            self._done_event.set()

        # Parse encodeurs "Enc G=xxx Enc D=xxx"
        if "Enc G=" in line and "Enc D=" in line:
            try:
                g = line.split("Enc G=")[1].split()[0]
                d = line.split("Enc D=")[1].split()[0]
                self.enc_g_lbl.config(text=g)
                self.enc_d_lbl.config(text=d)
            except Exception:
                pass

    def _on_disconnect(self):
        if self.connected:
            self._disconnect()

    # ------------------------------------------------------------------
    # Envoi
    # ------------------------------------------------------------------
    def _send_raw_cmd(self, cmd: str):
        if not self.connected or self.sock is None:
            return
        try:
            self.sock.sendall((cmd.strip() + "\n").encode())
        except Exception as e:
            self.root.after(0, self._log, f"[ERR] {e}")

    def _send(self, cmd: str):
        self._log(f"→ {cmd}")
        self._send_raw_cmd(cmd)

    def _move(self, direction: str):
        val = self.val_var.get()
        self._send(f"{direction}:{val:.1f}")

    def _stop(self):
        # Interrompt aussi une eventuelle sequence en cours
        self._seq_abort = True
        self._done_event.set()
        self._send("S")

    def _read_enc(self):
        self._send("E")

    def _poll_encoders(self):
        if self.connected:
            self._send_raw_cmd("E")
        self.root.after(2000, self._poll_encoders)

    def _apply_pid(self):
        # Envoie les gains PID du virage a l'ESP32 (sans recompiler)
        self._send(f"PIDT:{self.kp_var.get():.3f},{self.ki_var.get():.3f},{self.kd_var.get():.3f}")

    def _send_raw(self):
        cmd = self.raw_var.get().strip()
        if cmd:
            self._send(cmd)
            self.raw_var.set("")

    # ------------------------------------------------------------------
    # Sequence n°1 : l'escalier
    # ------------------------------------------------------------------
    def _seq_escalier(self):
        if self._seq_running:
            self._log("[SEQ1] Déjà en cours.")
            return
        self._seq_running = True
        self._seq_abort = False

        def run():
            try:
                if not self.connected:
                    self.root.after(0, self._log, "[SEQ1] Non connecté.")
                    return
                self.root.after(0, self._log,
                                "[SEQ1] Escalier (firmware : 20cm fermé → tractrice)")
                self._done_event.clear()
                self._send_raw_cmd("SEQ:1")
                # La sequence complete dure ~15 s : on laisse une marge.
                if not self._done_event.wait(timeout=SEQ1_TOTAL_TIMEOUT_S):
                    self.root.after(0, self._log, "[SEQ1] Timeout — abandon.")
                    self._send_raw_cmd("S")
                    return
                if self._seq_abort:
                    self.root.after(0, self._log, "[SEQ1] Interrompue.")
                    return
                self.root.after(0, self._log, "[SEQ1] Terminé.")
            finally:
                self._seq_running = False

        threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------------------------------
    # Sequence n°2 : le cercle (Généré dynamiquement par l'ESP32)
    # ------------------------------------------------------------------
    def _seq_cercle(self):
        if self._seq_running:
            self._log("[SEQ2] Déjà en cours.")
            return
        self._seq_running = True
        self._seq_abort = False
        
        rayon = self.rayon_var.get()

        def run():
            try:
                if not self.connected:
                    self.root.after(0, self._log, "[SEQ2] Non connecté.")
                    return
                self.root.after(0, self._log, f"[SEQ2] Cercle dynamique (Rayon={rayon:.1f} cm)")
                self._done_event.clear()
                
                # Envoi de la commande avec le rayon en argument
                self._send_raw_cmd(f"SEQ:2:{rayon:.1f}")
                
                # Le cercle met environ 10 secondes : on laisse une marge.
                if not self._done_event.wait(timeout=SEQ2_TOTAL_TIMEOUT_S):
                    self.root.after(0, self._log, "[SEQ2] Timeout — abandon.")
                    self._send_raw_cmd("S")
                    return
                if self._seq_abort:
                    self.root.after(0, self._log, "[SEQ2] Interrompue.")
                    return
                self.root.after(0, self._log, "[SEQ2] Terminé.")
            finally:
                self._seq_running = False

        threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------------------------------
    # Divers
    # ------------------------------------------------------------------
    def _log(self, msg: str):
        self.log.config(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _set_controls_state(self, state: str):
        for w in self.root.winfo_children():
            self._set_widget_state(w, state)

    def _set_widget_state(self, widget, state):
        if isinstance(widget, ttk.LabelFrame) and widget.cget("text") == "Connexion WiFi":
            return
        try:
            if isinstance(widget, (ttk.Button, ttk.Entry, ttk.Spinbox)):
                widget.config(state=state)
        except Exception:
            pass
        for child in widget.winfo_children():
            self._set_widget_state(child, state)


def main():
    root = tk.Tk()
    DrawbotGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()