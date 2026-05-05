#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define LEDU1 25
#define LEDU2 26
#define EN_D 23
#define EN_G 4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define ENC_G_CH_A 32
#define ENC_G_CH_B 33
#define ENC_D_CH_A 27
#define ENC_D_CH_B 14
#define LEDC_FREQ 20000
#define LEDC_RES 8
#define CH_IN1_D 0
#define CH_IN2_D 1
#define CH_IN1_G 2
#define CH_IN2_G 3

const char* WIFI_SSID = "i.t.WORKS N300";
const int TCP_PORT = 8266;

WiFiServer server(TCP_PORT);
WiFiClient client;

void envoyer(const String& msg) {
  if (client && client.connected()) client.println(msg);
  Serial.println(msg);
}

volatile long enc_gauche = 0;
volatile long enc_droit = 0;

void IRAM_ATTR isr_enc_g() {
  enc_gauche += (digitalRead(ENC_G_CH_B) == HIGH) ? 1 : -1;
}
void IRAM_ATTR isr_enc_d() {
  enc_droit += (digitalRead(ENC_D_CH_B) == HIGH) ? 1 : -1;
}
#define TICKS_PER_REV 993
#define WHEEL_DIAM_MM 90.0f
#define WHEEL_CIRCUM_CM (PI * WHEEL_DIAM_MM / 10.0f)
#define TRACK_WIDTH_CM 8.5f
#define TURN_CORRECTION 0.77f
#define PWM_MIN 200

bool tick_motion_active = false;
unsigned long tick_target = 0;
int tick_dir_D = 1;
int tick_dir_G = 1;

void setMoteurDroit(int vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse > 0) {
    vitesse = max(vitesse, PWM_MIN);
    ledcWrite(CH_IN1_D, 0); ledcWrite(CH_IN2_D, vitesse);
  } else if (vitesse < 0) {
    vitesse = min(vitesse, -PWM_MIN);
    ledcWrite(CH_IN1_D, -vitesse); ledcWrite(CH_IN2_D, 0);
  } else {
    ledcWrite(CH_IN1_D, 0); ledcWrite(CH_IN2_D, 0);
  }
}

void setMoteurGauche(int vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse > 0) {
    vitesse = max(vitesse, PWM_MIN);
    ledcWrite(CH_IN1_G, vitesse); ledcWrite(CH_IN2_G, 0);
  } else if (vitesse < 0) {
    vitesse = min(vitesse, -PWM_MIN);
    ledcWrite(CH_IN1_G, 0); ledcWrite(CH_IN2_G, -vitesse);
  } else {
    ledcWrite(CH_IN1_G, 0); ledcWrite(CH_IN2_G, 0);
  }
}

void stopMoteurs() {
  setMoteurDroit(0);
  setMoteurGauche(0);
}

void startMouvement(int vD, int vG, unsigned long ticks) {
  noInterrupts();
  enc_gauche = 0;
  enc_droit = 0;
  interrupts();
  tick_dir_D = (vD >= 0) ? 1 : -1;
  tick_dir_G = (vG >= 0) ? 1 : -1;
  tick_target = ticks;
  tick_motion_active = true;
  setMoteurDroit(vD);
  setMoteurGauche(vG);
}

void traiterCommande(const String& cmd) {
  digitalWrite(LEDU2, !digitalRead(LEDU2));

  if (cmd.startsWith("F:")) {
    float cm = cmd.substring(2).toFloat();
    unsigned long ticks = (unsigned long)(cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(220, 220, ticks);
    envoyer(">> F:" + String(cm) + "cm = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("B:")) {
    float cm = cmd.substring(2).toFloat();
    unsigned long ticks = (unsigned long)(cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(-220, -220, ticks);
    envoyer(">> B:" + String(cm) + "cm = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("L:")) {
    float deg = cmd.substring(2).toFloat();
    float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
    unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(220, -220, ticks);
    envoyer(">> L:" + String(deg) + "deg = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("R:")) {
    float deg = cmd.substring(2).toFloat();
    float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
    unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(-220, 220, ticks);
    envoyer(">> R:" + String(deg) + "deg = " + String(ticks) + " ticks");

  } else if (cmd == "S" || cmd == "s") {
    stopMoteurs();
    tick_motion_active = false;
    envoyer(">> Stop");

  } else if (cmd == "E" || cmd == "e") {
    envoyer("Enc G=" + String(enc_gauche) + " Enc D=" + String(enc_droit));

  } else {
    envoyer("ERR:UNKNOWN:" + cmd);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID);
  Serial.print("Connexion WiFi");
  unsigned long debut = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - debut > 10000) { Serial.println("\nEchec WiFi !"); break; }
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connecte ! IP : " + WiFi.localIP().toString());
    server.begin();
  } else {
    Serial.println("Mode sans WiFi - USB uniquement.");
  }

  pinMode(EN_D, OUTPUT); digitalWrite(EN_D, HIGH);
  pinMode(EN_G, OUTPUT); digitalWrite(EN_G, HIGH);
  pinMode(LEDU1, OUTPUT); digitalWrite(LEDU1, LOW);
  pinMode(LEDU2, OUTPUT); digitalWrite(LEDU2, LOW);

  ledcSetup(CH_IN1_D, LEDC_FREQ, LEDC_RES);
  ledcSetup(CH_IN2_D, LEDC_FREQ, LEDC_RES);
  ledcSetup(CH_IN1_G, LEDC_FREQ, LEDC_RES);
  ledcSetup(CH_IN2_G, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(IN_1_D, CH_IN1_D);
  ledcAttachPin(IN_2_D, CH_IN2_D);
  ledcAttachPin(IN_1_G, CH_IN1_G);
  ledcAttachPin(IN_2_G, CH_IN2_G);

  pinMode(ENC_G_CH_A, INPUT_PULLUP); pinMode(ENC_G_CH_B, INPUT_PULLUP);
  pinMode(ENC_D_CH_A, INPUT_PULLUP); pinMode(ENC_D_CH_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), isr_enc_g, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), isr_enc_d, RISING);

  stopMoteurs();
  digitalWrite(LEDU1, HIGH);
}

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) traiterCommande(cmd);
  }

  if (!client || !client.connected()) {
    client = server.accept();
    if (client) Serial.println("Client connecte : " + client.remoteIP().toString());
  }

  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) traiterCommande(cmd);
  }

  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage >= 500) {
    Serial.print("Enc G: "); Serial.print(enc_gauche);
    Serial.print(" | Enc D: "); Serial.println(enc_droit);
    dernierAffichage = millis();
  }

  if (tick_motion_active) {
    noInterrupts();
    long tg = abs(enc_gauche);
    long td = abs(enc_droit);
    interrupts();
    unsigned long avg = ((unsigned long)tg + (unsigned long)td) / 2;
    if (avg >= tick_target) {
      stopMoteurs();
      tick_motion_active = false;
      envoyer(">> DONE");
    }
  }

  static unsigned long dernierIP = 0;
  if (millis() - dernierIP >= 3000) {
    if (WiFi.status() == WL_CONNECTED)
      Serial.println("IP : " + WiFi.localIP().toString());
    dernierIP = millis();
  }
}