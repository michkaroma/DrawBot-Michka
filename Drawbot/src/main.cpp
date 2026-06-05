#include <WiFi.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
//  DRAWBOT — Firmware ESP32
//  - Trajets droits (F/B) : asservissement en position des
//    roues (boucle fermee P) + synchro G/D + freinage actif.
//  - Virages (L/R)        : boucle fermee PID sur l'angle REEL
//    mesure par le gyroscope de l'IMU (LSM6DS3). Insensible au
//    glissement des roues => angles precis. Repli sur encodeur
//    si l'IMU n'est pas detectee.
// ============================================================

// ---------- Brochage (cf. kick-off slide 6) ----------
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
#define PIN_SDA 21
#define PIN_SCL 22

// ---------- PWM (LEDC) ----------
#define LEDC_FREQ 20000
#define LEDC_RES 8
#define CH_IN1_D 0
#define CH_IN2_D 1
#define CH_IN1_G 2
#define CH_IN2_G 3

// ---------- Calibration mecanique ----------
#define TICKS_PER_REV 993
#define WHEEL_DIAM_MM 90.0f
#define WHEEL_CIRCUM_CM (PI * WHEEL_DIAM_MM / 10.0f)
#define TRACK_WIDTH_CM 8.5f

// !!! A RECALIBRER !!! (utilise uniquement par le repli encodeur)
// L'ancienne valeur (0.77) compensait le depassement du a l'arret
// en boucle ouverte. Avec l'asservissement P + freinage actif, ce
// depassement disparait : partir de 1.0, commander L:90, mesurer
// l'angle reel au rapporteur, puis ajuster :
//   TURN_CORRECTION = angle_voulu / angle_mesure
#define TURN_CORRECTION 1.0f

// ---------- Parametres de l'asservissement (trajets droits) ----------
#define PWM_MIN 200            // seuil de friction statique (mesure)
#define PWM_MAX 240
#define KP 1.8f                // gain proportionnel (ticks -> PWM)
#define KSYNC 1.2f             // gain de synchro entre les deux roues
#define TOL_TICKS 4            // tolerance d'arret (~1 mm de roue)
#define CONTROL_PERIOD_MS 10   // periode d'echantillonnage : 100 Hz
#define BRAKE_MS 80            // duree du freinage actif

// ---------- Commande V : vitesse roue en boucle OUVERTE (cm/s -> PWM) ----------
// Conversion lineaire issue du calibrage : a V_MAX_CMS le PWM vaut PWM_V_MAX.
// PWM_MIN_V = PWM le plus bas ou la roue tourne encore (frottement statique).
// Il est volontairement plus BAS que PWM_MIN (200) : ce dernier est le plancher
// de DEMARRAGE fiable en charge pour l'asservissement ferme, alors que la
// sequence tractrice a besoin de vitesses lentes (jusqu'a ~1.7 cm/s) pour
// tracer les virages. La commande V ecrit donc le PWM directement (ecrirePWM*),
// sans repasser par le plancher 200 de setMoteur*.
// !!! A CALIBRER sur le robot (voir procedure dans le resume) !!!
#define V_MAX_CMS  5.5f        // vitesse (cm/s) atteinte a PWM_V_MAX  -- a mesurer
#define PWM_V_MAX  245         // PWM correspondant a V_MAX_CMS
#define PWM_MIN_V  70          // PWM mini ou la roue tourne encore (deadband bas)

// ---------- IMU LSM6DS3 (gyroscope, I2C) ----------
#define ADDR_IMU     0x6B
#define LSM_WHO_AM_I 0x0F
#define LSM_CTRL2_G  0x11
#define LSM_CTRL3_C  0x12
#define LSM_OUTZ_L_G 0x26
#define GYRO_SENS_DPS 0.070f   // 70 mdps/LSB a pleine echelle +/-2000 dps
// !!! A VERIFIER selon le montage de l'IMU : envoyer "G", tourner le
// robot a la MAIN vers la GAUCHE. Si "rate" est positif -> laisser +1.
// Si "rate" est negatif -> mettre -1.0f.
#define GYRO_Z_SIGN  +1.0f

// ---------- PID du virage (boucle fermee sur le gyroscope) ----------
// Modifiables en direct (sans recompiler) via la commande "PIDT:kp,ki,kd".
float KP_TURN = 4.0f;          // reactivite (deg d'erreur -> PWM)
float KI_TURN = 0.10f;         // rattrape l'erreur residuelle (frottement)
float KD_TURN = 0.20f;         // amortit / anticipe pour eviter le depassement
#define TURN_PWM_MAX 230       // PWM maxi pendant le virage (plus bas = plus lent = plus precis)
#define TURN_INTEGRAL_MAX 200.0f
// Le robot stoppe TURN_BRAKE_LEAD_DEG avant la cible pour compenser
// l'inertie residuelle apres freinage. A calibrer : si L:90 finit a
// 94 deg -> augmenter ; si finit a 87 deg -> diminuer.
#define TURN_BRAKE_LEAD_DEG 2.0f
#define TURN_TIMEOUT_MS 8000   // securite : abandon si la cible n'est jamais atteinte

// ---------- Reseau ----------
const char* WIFI_SSID = "i.t.WORKS N300";
const int TCP_PORT = 8266;

WiFiServer server(TCP_PORT);
WiFiClient client;

void envoyer(const String& msg) {
  if (client && client.connected()) client.println(msg);
  Serial.println(msg);
}

// ---------- Encodeurs ----------
volatile long enc_gauche = 0;
volatile long enc_droit = 0;

void IRAM_ATTR isr_enc_g() {
  enc_gauche += (digitalRead(ENC_G_CH_B) == HIGH) ? 1 : -1;
}
void IRAM_ATTR isr_enc_d() {
  enc_droit += (digitalRead(ENC_D_CH_B) == HIGH) ? 1 : -1;
}

// ---------- Etat du mouvement ----------
enum MotionMode { MODE_IDLE, MODE_TICKS, MODE_TURN, MODE_VSEQ };
MotionMode motionMode = MODE_IDLE;

// Trajets droits (et repli virage encodeur)
long tick_target = 0;     // cible en ticks, identique pour chaque roue
int tick_dir_D = 1;       // sens de rotation roue droite (+1 / -1)
int tick_dir_G = 1;       // sens de rotation roue gauche (+1 / -1)

// Virage gyro
bool  imuOk = false;
float gyroZbias_dps   = 0.0f;   // biais (offset) du gyro au repos
float turn_target_deg = 0.0f;   // consigne signee : gauche > 0, droite < 0
float gyro_angle_deg  = 0.0f;   // angle reel integre depuis le gyro
float turn_integral   = 0.0f;
float turn_prev_error = 0.0f;
unsigned long turn_last_us  = 0;
unsigned long turn_start_ms = 0;

// Boucle ouverte temporisee : commande V et sequence escalier (tractrice)
struct CmdV { float vg; float vd; int dur_ms; };   // vitesses en cm/s, duree en ms
const CmdV* vseq_ptr = nullptr;     // table de paliers en cours d'execution
int  vseq_len = 0;                  // nombre de paliers
int  vseq_idx = 0;                  // palier courant
unsigned long vseq_step_start = 0;  // millis() au debut du palier courant
CmdV vseq_single;                   // tampon pour une commande V isolee
bool escalierPending = false;       // F:20 (ferme) en cours avant la tractrice

// ---------- Commande bas niveau des moteurs ----------
// Ecriture PWM "brute" : sens + saturation uniquement, SANS plancher de
// friction. Le moteur gauche a IN1/IN2 cables a l'envers du droit :
// l'inversion de sens est geree ICI, de facon centralisee. v=0 => roue libre.
void ecrirePWMDroit(int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { ledcWrite(CH_IN1_D, 0);  ledcWrite(CH_IN2_D, v);  }
  else        { ledcWrite(CH_IN1_D, -v); ledcWrite(CH_IN2_D, 0);  }
}
void ecrirePWMGauche(int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { ledcWrite(CH_IN1_G, v);  ledcWrite(CH_IN2_G, 0);  }
  else        { ledcWrite(CH_IN1_G, 0);  ledcWrite(CH_IN2_G, -v); }
}

// Conversion vitesse (cm/s) -> PWM signe (-255..255) pour la commande V.
// Relation lineaire + deadband bas (PWM_MIN_V) pour vaincre le frottement.
// Le signe est conserve tel quel : c'est ecrirePWMGauche qui gere l'inversion
// du moteur gauche, donc on NE compense PAS le sens ici.
int vitesseToPWM(float v_cms) {
  if (fabsf(v_cms) < 0.1f) return 0;
  int pwm = (int)(v_cms / V_MAX_CMS * PWM_V_MAX);
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0 && pwm <  PWM_MIN_V) pwm =  PWM_MIN_V;
  if (pwm < 0 && pwm > -PWM_MIN_V) pwm = -PWM_MIN_V;
  return pwm;
}

// Commande "haut niveau" pour l'asservissement FERME (F/B/L/R) : applique en
// plus le plancher de DEMARRAGE PWM_MIN (200), necessaire pour partir en charge.
void setMoteurDroit(int vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse > 0)      vitesse = max(vitesse, PWM_MIN);
  else if (vitesse < 0) vitesse = min(vitesse, -PWM_MIN);
  ecrirePWMDroit(vitesse);
}

void setMoteurGauche(int vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  if (vitesse > 0)      vitesse = max(vitesse, PWM_MIN);
  else if (vitesse < 0) vitesse = min(vitesse, -PWM_MIN);
  ecrirePWMGauche(vitesse);
}

void stopMoteurs() {
  setMoteurDroit(0);
  setMoteurGauche(0);
}

// Freinage actif : sur le DRV8837, les deux entrees a l'etat haut
// court-circuitent le moteur (mode "brake") au lieu de le laisser
// en roue libre. C'est ce qui empeche le robot de glisser sur son
// inertie au moment de l'arret.
void brakeMoteurs() {
  ledcWrite(CH_IN1_D, 255); ledcWrite(CH_IN2_D, 255);
  ledcWrite(CH_IN1_G, 255); ledcWrite(CH_IN2_G, 255);
  delay(BRAKE_MS);
  stopMoteurs();
}

// ============================================================
//  IMU LSM6DS3 : configuration et lecture du gyroscope
// ============================================================
void imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t imuReadReg(uint8_t reg) {
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADDR_IMU, 1);
  return Wire.available() ? Wire.read() : 0;
}

// Vitesse de rotation autour de la verticale, en deg/s.
// Biais retire et signe applique : positif = rotation vers la gauche.
float lireGyroZ() {
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(LSM_OUTZ_L_G);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADDR_IMU, 2);
  if (Wire.available() < 2) return 0.0f;
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  int16_t raw = (int16_t)((hi << 8) | lo);     // signe, little-endian
  return ((float)raw * GYRO_SENS_DPS - gyroZbias_dps) * GYRO_Z_SIGN;
}

bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  delay(20);                                   // boot LSM6DS3 (~15 ms)
  uint8_t who = imuReadReg(LSM_WHO_AM_I);
  if (who != 0x69 && who != 0x6A) return false; // 0x69=LSM6DS3, 0x6A=LSM6DS3TR-C
  imuWriteReg(LSM_CTRL3_C, 0x44);              // BDU=1 + auto-increment
  imuWriteReg(LSM_CTRL2_G, 0x5C);              // gyro 208 Hz, +/-2000 dps
  delay(50);
  return true;
}

// Mesure du biais du gyro : moyenne sur ~1 s, ROBOT IMMOBILE.
void calibrerGyro(int n = 400) {
  double somme = 0;
  for (int i = 0; i < n; i++) {
    Wire.beginTransmission(ADDR_IMU);
    Wire.write(LSM_OUTZ_L_G);
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR_IMU, 2);
    if (Wire.available() >= 2) {
      uint8_t lo = Wire.read();
      uint8_t hi = Wire.read();
      int16_t raw = (int16_t)((hi << 8) | lo);
      somme += (double)raw * GYRO_SENS_DPS;
    }
    delay(3);
  }
  gyroZbias_dps = (float)(somme / n);
}

// ---------- Demarrage d'un mouvement asservi (ticks : F/B) ----------
// On ne fixe plus de PWM ici : on definit seulement la cible et les
// sens. C'est la boucle d'asservissement qui calcule le PWM a chaque
// periode d'echantillonnage.
void startMouvement(int dirD, int dirG, unsigned long ticks) {
  noInterrupts();
  enc_gauche = 0;
  enc_droit = 0;
  interrupts();
  tick_dir_D = (dirD >= 0) ? 1 : -1;
  tick_dir_G = (dirG >= 0) ? 1 : -1;
  tick_target = (long)ticks;
  escalierPending = false;        // un F/B ordinaire n'enchaine pas la tractrice
  motionMode = MODE_TICKS;
}

// ---------- Demarrage d'un virage asservi sur le gyro (L/R) ----------
void startTurnGyro(float deg) {       // deg > 0 : gauche, deg < 0 : droite
  gyro_angle_deg  = 0.0f;
  turn_target_deg = deg;
  turn_integral   = 0.0f;
  turn_prev_error = deg;
  turn_last_us    = micros();
  turn_start_ms   = millis();
  escalierPending = false;
  motionMode      = MODE_TURN;
}

// ============================================================
//  Commande V et sequence escalier (boucle OUVERTE, tractrice)
// ============================================================
// Profil de vitesses pre-calcule (cinematique de la tractrice) qui trace les
// DEUX virages de l'escalier : virage gauche 90°, puis virage droit 90° fondu
// dans le segment droit final de 40 cm. Le segment initial de 20 cm est fait
// AVANT, en boucle FERMEE (commande F), pour tenir la tolerance de distance
// (+/-1 cm). NB : le segment de 10 cm entre les deux virages n'apparait pas
// dans cette table -> a verifier / completer (voir resume).
static const CmdV ESCALIER[] = {
  // --- virage gauche 90° (tractrice) ---
  {-1.69f, 2.00f, 110}, {-1.37f, 2.31f, 110}, {-1.05f, 2.61f, 110},
  {-0.72f, 2.91f, 120}, {-0.39f, 3.20f, 120}, {-0.07f, 3.47f, 120},
  { 0.26f, 3.74f, 120}, { 0.59f, 4.00f, 120}, { 0.92f, 4.25f, 130},
  { 1.24f, 4.49f, 130}, { 1.56f, 4.71f, 130}, { 1.88f, 4.92f, 140},
  { 2.19f, 5.12f, 140}, { 2.49f, 5.30f, 100},
  // --- virage droit 90° + segment final de 40 cm (tractrice) ---
  { 4.79f, 2.77f, 200}, { 4.92f, 3.06f, 200}, { 5.00f, 3.27f, 200},
  { 5.06f, 3.45f, 200}, { 5.11f, 3.60f, 200}, { 5.15f, 3.74f, 200},
  { 5.18f, 3.86f, 200}, { 5.20f, 3.97f, 200}, { 5.21f, 4.07f, 200},
  { 5.22f, 4.16f, 200}, { 5.23f, 4.23f, 200}, { 5.23f, 4.31f, 200},
  { 5.23f, 4.38f, 200}, { 5.23f, 4.43f, 200}, { 5.22f, 4.49f, 200},
  { 5.22f, 4.53f, 200}, { 5.21f, 4.57f, 200}, { 5.20f, 4.61f, 200},
  { 5.19f, 4.65f, 200}, { 5.19f, 4.68f, 200}, { 5.18f, 4.71f, 200},
  { 5.17f, 4.73f, 200}, { 5.16f, 4.76f, 200}, { 5.15f, 4.77f, 200},
  { 5.14f, 4.80f, 200}, { 5.13f, 4.81f, 200}, { 5.13f, 4.83f, 200},
  { 5.12f, 4.84f, 200}, { 5.11f, 4.86f, 200}, { 5.10f, 4.86f, 200},
  { 5.10f, 4.88f, 200}, { 5.09f, 4.89f, 200}, { 5.09f, 4.89f, 200},
  { 5.08f, 4.91f, 200}, { 5.07f, 4.91f, 200}, { 5.07f, 4.92f, 200},
  { 5.06f, 4.93f, 200}, { 5.06f, 4.93f, 200}, { 5.06f, 4.93f, 200},
  { 5.05f, 4.94f, 200},
};
const int N_ESCALIER = sizeof(ESCALIER) / sizeof(ESCALIER[0]);

// Demarre l'execution d'une table de paliers de vitesse (boucle ouverte).
void startVSeq(const CmdV* seq, int len) {
  vseq_ptr = seq;
  vseq_len = len;
  vseq_idx = 0;
  vseq_step_start = millis();
  motionMode = MODE_VSEQ;
}

// ---------- Boucle d'execution des paliers V (100 Hz) ----------
// A chaque periode : applique la consigne de vitesse du palier courant ; quand
// sa duree est ecoulee, passe au suivant ; en fin de table, freine -> IDLE.
// L'inversion du moteur gauche est geree par ecrirePWMGauche (pas ici).
void controlVSeq() {
  if (vseq_ptr == nullptr || vseq_idx >= vseq_len) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> DONE VSEQ paliers=" + String(vseq_idx));
    return;
  }
  const CmdV& s = vseq_ptr[vseq_idx];
  ecrirePWMGauche(vitesseToPWM(s.vg));
  ecrirePWMDroit (vitesseToPWM(s.vd));
  if (millis() - vseq_step_start >= (unsigned long)s.dur_ms) {
    vseq_idx++;
    vseq_step_start = millis();
  }
}

// ---------- Boucle d'asservissement des trajets droits (100 Hz) ----------
// Pour CHAQUE roue : erreur = ticks restants, PWM = KP * erreur,
// borne entre PWM_MIN (friction) et PWM_MAX. Le robot ralentit donc
// naturellement en approchant de la cible.
// Le terme KSYNC penalise la roue en avance et booste la roue en
// retard, ce qui garantit une ligne bien droite.
void controlTicks() {
  noInterrupts();
  long tg = abs(enc_gauche);
  long td = abs(enc_droit);
  interrupts();

  long errG = tick_target - tg;
  long errD = tick_target - td;

  if (errG <= TOL_TICKS && errD <= TOL_TICKS) {
    brakeMoteurs();
    if (escalierPending) {
      // Segment initial de 20 cm termine -> enchaine la partie tractrice.
      // Pas de ">> DONE" ici : le DONE final viendra de controlVSeq.
      escalierPending = false;
      envoyer(">> SEQ:1 20cm OK -> tractrice");
      startVSeq(ESCALIER, N_ESCALIER);
    } else {
      motionMode = MODE_IDLE;
      envoyer(">> DONE G=" + String(tg) + " D=" + String(td));
    }
    return;
  }

  long diff = tg - td;   // >0 : la gauche est en avance sur la droite

  int pwmG = 0, pwmD = 0;
  if (errG > TOL_TICKS)
    pwmG = constrain((int)(KP * errG - KSYNC * diff), PWM_MIN, PWM_MAX);
  if (errD > TOL_TICKS)
    pwmD = constrain((int)(KP * errD + KSYNC * diff), PWM_MIN, PWM_MAX);

  setMoteurGauche(pwmG * tick_dir_G);
  setMoteurDroit(pwmD * tick_dir_D);
}

// ---------- Boucle PID du virage sur le gyro (100 Hz) ----------
// consigne = angle voulu (deg), mesure = angle reel integre depuis le
// gyro. erreur = consigne - mesure. La commande PWM est appliquee de
// facon differentielle (une roue avant, l'autre arriere).
void controlTurnGyro() {
  unsigned long now_us = micros();
  float dt = (now_us - turn_last_us) / 1.0e6f;
  turn_last_us = now_us;
  if (dt <= 0.0f || dt > 0.2f) dt = CONTROL_PERIOD_MS / 1000.0f;   // garde-fou

  // 1) MESURE : integration de la vitesse gyro -> angle reel
  float rate = lireGyroZ();
  gyro_angle_deg += rate * dt;

  float error = turn_target_deg - gyro_angle_deg;

  // 2) ARRET : cible atteinte (avec avance pour compenser l'inertie)
  if (fabs(gyro_angle_deg) >= fabs(turn_target_deg) - TURN_BRAKE_LEAD_DEG) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> DONE TURN angle=" + String(gyro_angle_deg, 1) +
            " cible=" + String(turn_target_deg, 1));
    return;
  }

  // 2bis) SECURITE : signe du gyro inverse ? (le robot tourne mais
  // l'angle part dans le mauvais sens) -> on stoppe et on previent.
  if (millis() - turn_start_ms > 400 &&
      fabs(gyro_angle_deg) > 5.0f &&
      (gyro_angle_deg * turn_target_deg) < 0.0f) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> ERR signe gyro inverse ? angle=" + String(gyro_angle_deg, 1) +
            " cible=" + String(turn_target_deg, 1) + " -> inverser GYRO_Z_SIGN");
    return;
  }

  // 2ter) SECURITE : timeout
  if (millis() - turn_start_ms > TURN_TIMEOUT_MS) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> DONE TURN TIMEOUT angle=" + String(gyro_angle_deg, 1));
    return;
  }

  // 3) PID
  turn_integral += error * dt;
  turn_integral = constrain(turn_integral, -TURN_INTEGRAL_MAX, TURN_INTEGRAL_MAX);
  float deriv = (error - turn_prev_error) / dt;
  turn_prev_error = error;

  float out = KP_TURN * error + KI_TURN * turn_integral + KD_TURN * deriv;

  // 4) Application : saturation (le plancher de friction PWM_MIN est
  //    assure par setMoteur*). cmd > 0 => rotation vers la gauche :
  //    roue droite en avant, roue gauche en arriere.
  int cmd = constrain((int)out, -TURN_PWM_MAX, TURN_PWM_MAX);
  setMoteurDroit(cmd);
  setMoteurGauche(-cmd);
}

// ---------- Traitement des commandes ----------
void traiterCommande(const String& cmd) {
  digitalWrite(LEDU2, !digitalRead(LEDU2));

  if (cmd.startsWith("F:")) {
    float cm = cmd.substring(2).toFloat();
    unsigned long ticks = (unsigned long)(cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(+1, +1, ticks);
    envoyer(">> F:" + String(cm) + "cm = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("B:")) {
    float cm = cmd.substring(2).toFloat();
    unsigned long ticks = (unsigned long)(cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(-1, -1, ticks);
    envoyer(">> B:" + String(cm) + "cm = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("L:")) {
    float deg = cmd.substring(2).toFloat();
    if (imuOk) {
      startTurnGyro(+deg);                       // gauche = positif
      envoyer(">> L:" + String(deg) + "deg (gyro PID)");
    } else {
      float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
      unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(+1, -1, ticks);             // droite avant, gauche arriere
      envoyer(">> L:" + String(deg) + "deg = " + String(ticks) + " ticks (encodeur)");
    }

  } else if (cmd.startsWith("R:")) {
    float deg = cmd.substring(2).toFloat();
    if (imuOk) {
      startTurnGyro(-deg);                       // droite = negatif
      envoyer(">> R:" + String(deg) + "deg (gyro PID)");
    } else {
      float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
      unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(-1, +1, ticks);             // gauche avant, droite arriere
      envoyer(">> R:" + String(deg) + "deg = " + String(ticks) + " ticks (encodeur)");
    }

  } else if (cmd.startsWith("V:")) {
    // Vitesse roue independante, boucle ouverte temporisee : V:vG:vD:duree_ms
    String s = cmd.substring(2);
    int c1 = s.indexOf(':');
    int c2 = s.indexOf(':', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      vseq_single.vg     = s.substring(0, c1).toFloat();
      vseq_single.vd     = s.substring(c1 + 1, c2).toFloat();
      vseq_single.dur_ms = s.substring(c2 + 1).toInt();
      escalierPending = false;
      startVSeq(&vseq_single, 1);
      envoyer(">> V g=" + String(vseq_single.vg, 2) + " d=" +
              String(vseq_single.vd, 2) + " t=" + String(vseq_single.dur_ms) + "ms");
    } else {
      envoyer("ERR:V attend vG:vD:duree_ms");
    }

  } else if (cmd.startsWith("SEQ:")) {
    int n = cmd.substring(4).toInt();
    if (n == 1) {
      // Escalier : 20 cm en boucle FERMEE (F) puis virages en boucle OUVERTE.
      unsigned long ticks = (unsigned long)(20.0f / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(+1, +1, ticks);   // remet escalierPending a false...
      escalierPending = true;          // ...donc on le re-arme juste apres
      envoyer(">> SEQ:1 escalier (20cm ferme -> tractrice ouverte)");
    } else {
      envoyer("ERR:SEQ inconnue:" + String(n));
    }

  } else if (cmd == "S" || cmd == "s") {
    motionMode = MODE_IDLE;
    escalierPending = false;
    brakeMoteurs();
    envoyer(">> Stop");

  } else if (cmd == "E" || cmd == "e") {
    envoyer("Enc G=" + String(enc_gauche) + " Enc D=" + String(enc_droit));

  } else if (cmd == "G" || cmd == "g") {
    // Lecture gyro : utile pour verifier le signe et pour le reglage
    envoyer("Gyro rate=" + String(lireGyroZ(), 2) + " dps | angle=" +
            String(gyro_angle_deg, 2) + " deg | imu=" + String(imuOk ? 1 : 0));

  } else if (cmd == "CAL" || cmd == "cal") {
    if (imuOk) {
      calibrerGyro();
      envoyer(">> Biais gyro = " + String(gyroZbias_dps, 3) + " dps");
    } else {
      envoyer("ERR:IMU absente");
    }

  } else if (cmd.startsWith("PIDT:")) {
    // Reglage en direct des gains du PID de virage : "PIDT:kp,ki,kd"
    String s = cmd.substring(5);
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      KP_TURN = s.substring(0, c1).toFloat();
      KI_TURN = s.substring(c1 + 1, c2).toFloat();
      KD_TURN = s.substring(c2 + 1).toFloat();
      envoyer(">> PID virage Kp=" + String(KP_TURN, 3) +
              " Ki=" + String(KI_TURN, 3) + " Kd=" + String(KD_TURN, 3));
    } else {
      envoyer("ERR:PIDT attend Kp,Ki,Kd");
    }

  } else {
    envoyer("ERR:UNKNOWN:" + cmd);
  }
}

// ---------- Setup ----------
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

  // IMU : gyroscope pour les virages en boucle fermee
  imuOk = imuInit();
  if (imuOk) {
    Serial.println("IMU LSM6DS3 OK - calibration du gyro (NE PAS BOUGER le robot)...");
    calibrerGyro();
    Serial.println("Biais gyro = " + String(gyroZbias_dps, 3) + " dps");
  } else {
    Serial.println("IMU non detectee - virages en repli ENCODEUR.");
  }

  stopMoteurs();
  digitalWrite(LEDU1, HIGH);
}

// ---------- Boucle principale ----------
void loop() {

  // Commandes via USB (debug)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) traiterCommande(cmd);
  }

  // Acceptation d'un client TCP
  if (!client || !client.connected()) {
    client = server.accept();
    if (client) Serial.println("Client connecte : " + client.remoteIP().toString());
  }

  // Commandes via WiFi
  if (client && client.connected() && client.available()) {
    String cmd = client.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) traiterCommande(cmd);
  }

  // Boucle de controle, cadencee a 100 Hz
  static unsigned long dernierControle = 0;
  if (motionMode != MODE_IDLE && millis() - dernierControle >= CONTROL_PERIOD_MS) {
    dernierControle = millis();
    if (motionMode == MODE_TICKS)      controlTicks();
    else if (motionMode == MODE_TURN)  controlTurnGyro();
    else if (motionMode == MODE_VSEQ)  controlVSeq();
  }

  // Affichage periodique des encodeurs (debug USB)
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage >= 500) {
    Serial.print("Enc G: "); Serial.print(enc_gauche);
    Serial.print(" | Enc D: "); Serial.println(enc_droit);
    dernierAffichage = millis();
  }

  // Rappel periodique de l'IP (debug USB)
  static unsigned long dernierIP = 0;
  if (millis() - dernierIP >= 3000) {
    if (WiFi.status() == WL_CONNECTED)
      Serial.println("IP : " + WiFi.localIP().toString());
    dernierIP = millis();
  }
}
