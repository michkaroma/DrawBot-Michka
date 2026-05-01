#include <WiFi.h>

// ── Réseau WiFi (mode Access Point) ──────────────────────────────────────────
const char* AP_SSID     = "Drawbot";
const char* AP_PASSWORD = "drawbot123";
const int   TCP_PORT    = 8080;

WiFiServer server(TCP_PORT);
WiFiClient client;

// ── Pinout ───────────────────────────────────────────────────────────────────
#define IN1_D  19
#define IN2_D  18
#define EN_D   23
#define IN1_G  17
#define IN2_G  16
#define EN_G    4

#define ENC_D_A 27
#define ENC_D_B 14
#define ENC_G_A 32
#define ENC_G_B 33

#define SDA_PIN 21
#define SCL_PIN 22

// ── PWM ──────────────────────────────────────────────────────────────────────
#define PWM_FREQ   1000   // Hz
#define PWM_RES    8      // bits → 0..255
#define CH_IN1_D   0
#define CH_IN2_D   1
#define CH_IN1_G   2
#define CH_IN2_G   3

// ── Encodeurs (volatile = modifié par interruption) ──────────────────────────
volatile long enc_D = 0;
volatile long enc_G = 0;

void IRAM_ATTR isr_enc_D() {
  enc_D += (digitalRead(ENC_D_B) == digitalRead(ENC_D_A)) ? -1 : 1;
}
void IRAM_ATTR isr_enc_G() {
  enc_G += (digitalRead(ENC_G_B) == digitalRead(ENC_G_A)) ? 1 : -1;
}

// ── Fonctions moteurs ─────────────────────────────────────────────────────────
void motor_setup() {
  ledcSetup(CH_IN1_D, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN2_D, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN1_G, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN2_G, PWM_FREQ, PWM_RES);

  ledcAttachPin(IN1_D, CH_IN1_D);
  ledcAttachPin(IN2_D, CH_IN2_D);
  ledcAttachPin(IN1_G, CH_IN1_G);
  ledcAttachPin(IN2_G, CH_IN2_G);

  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);
}

// speed : -255 (arrière) .. 0 (stop) .. +255 (avant)
void set_motor_D(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(CH_IN1_D, speed);
    ledcWrite(CH_IN2_D, 0);
  } else if (speed < 0) {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, -speed);
  } else {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, 0);
  }
}

void set_motor_G(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(CH_IN1_G, speed);
    ledcWrite(CH_IN2_G, 0);
  } else if (speed < 0) {
    ledcWrite(CH_IN1_G, 0);
    ledcWrite(CH_IN2_G, -speed);
  } else {
    ledcWrite(CH_IN1_G, 0);
    ledcWrite(CH_IN2_G, 0);
  }
}

void stop_motors() {
  set_motor_D(0);
  set_motor_G(0);
}

// ── Traitement des commandes ──────────────────────────────────────────────────
// Protocole texte (une commande par ligne) :
//   F:<speed>     avancer  (speed 0..255)
//   B:<speed>     reculer
//   L:<speed>     tourner sur place à gauche
//   R:<speed>     tourner sur place à droite
//   S             stop
//   ENC           lire encodeurs → réponse "ENC:<D>,<G>\n"

void handle_command(const String& cmd) {
  if (cmd.startsWith("F:")) {
    int speed = cmd.substring(2).toInt();
    set_motor_D(speed);
    set_motor_G(speed);
    client.println("OK:FORWARD:" + String(speed));

  } else if (cmd.startsWith("B:")) {
    int speed = cmd.substring(2).toInt();
    set_motor_D(-speed);
    set_motor_G(-speed);
    client.println("OK:BACKWARD:" + String(speed));

  } else if (cmd.startsWith("L:")) {
    int speed = cmd.substring(2).toInt();
    set_motor_D(speed);
    set_motor_G(-speed);
    client.println("OK:LEFT:" + String(speed));

  } else if (cmd.startsWith("R:")) {
    int speed = cmd.substring(2).toInt();
    set_motor_D(-speed);
    set_motor_G(speed);
    client.println("OK:RIGHT:" + String(speed));

  } else if (cmd == "S") {
    stop_motors();
    client.println("OK:STOP");

  } else if (cmd == "ENC") {
    client.println("ENC:" + String(enc_D) + "," + String(enc_G));

  } else if (cmd == "RESET_ENC") {
    enc_D = 0;
    enc_G = 0;
    client.println("OK:RESET_ENC");

  } else if (cmd == "PING") {
    client.println("PONG");

  } else {
    client.println("ERR:UNKNOWN:" + cmd);
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Drawbot ===");

  // Moteurs
  motor_setup();
  stop_motors();

  // Encodeurs
  pinMode(ENC_D_A, INPUT_PULLUP);
  pinMode(ENC_D_B, INPUT_PULLUP);
  pinMode(ENC_G_A, INPUT_PULLUP);
  pinMode(ENC_G_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), isr_enc_D, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_G_A), isr_enc_G, CHANGE);

  // WiFi Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP IP : ");
  Serial.println(WiFi.softAPIP());  // généralement 192.168.4.1

  // Serveur TCP
  server.begin();
  Serial.println("Serveur TCP démarré sur port " + String(TCP_PORT));
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Accepter nouveau client si pas de client connecté
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connecté : " + client.remoteIP().toString());
      stop_motors();
    }
  }

  // Lire et traiter les commandes
  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();  // enlever \r\n
    if (cmd.length() > 0) {
      Serial.println("CMD: " + cmd);
      handle_command(cmd);
    }
  }
}
