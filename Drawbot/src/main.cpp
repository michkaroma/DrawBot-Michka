#include <WiFi.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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
#define TICKS_PER_CM (TICKS_PER_REV / WHEEL_CIRCUM_CM)   // ~35.1 ticks/cm
#define TRACK_WIDTH_CM 8.5f

// !!! A RECALIBRER !!! (utilise uniquement par le repli encodeur)
#define TURN_CORRECTION 1.0f

// ---------- Parametres de l'asservissement (trajets droits) ----------
#define PWM_MIN 200            // seuil de friction statique (mesure)
#define PWM_MAX 240
#define KP 1.8f                // gain proportionnel (ticks -> PWM)
#define KSYNC 1.2f             // gain de synchro entre les deux roues
#define TOL_TICKS 4            // tolerance d'arret (~1 mm de roue)
#define CONTROL_PERIOD_MS 10   // periode d'echantillonnage : 100 Hz
#define BRAKE_MS 80            // duree du freinage actif

// ---------- Commande V : vitesse de roue ASSERVIE (boucle fermee PI) ----------
// vitesseToPWM() ne sert plus que de FEEDFORWARD : PWM theorique pour une
// vitesse donnee (calibrage lineaire V_MAX_CMS <-> PWM_V_MAX, mesure :
// V:5:5:2000 -> 53 cm -> 26.5 cm/s a PWM 229 -> V_MAX ~ 29 cm/s a PWM 245).
// La boucle PI (controlVitesse) corrige ensuite avec la vitesse REELLE
// mesuree par les encodeurs : friction, retard moteur et piles faibles
// sont compenses automatiquement. Un "kick" anti-friction statique force
// PWM_KICK quand une roue commandee est encore immobile, car le PWM
// theorique des vitesses lentes (~1.7 cm/s -> PWM ~80) est tres en dessous
// du seuil de demarrage (PWM_MIN = 200).
#define V_MAX_CMS  29.0f       // vitesse (cm/s) atteinte a PWM_V_MAX (mesure)
#define PWM_V_MAX  245         // PWM correspondant a V_MAX_CMS
#define PWM_MIN_V  70          // PWM mini ou la roue tourne encore (deadband bas)
#define PWM_KICK   200         // coup de demarrage anti-friction statique

// Gains du PI de vitesse, reglables en direct via "PIDV:kp,ki"
float KP_V = 8.0f;             // (cm/s d'erreur) -> PWM
float KI_V = 300.0f;           // rattrape la friction ; anti-windup integre

// ---------- IMU LSM6DS3 (gyroscope, I2C) ----------
#define ADDR_IMU     0x6B
#define LSM_WHO_AM_I 0x0F
#define LSM_CTRL2_G  0x11
#define LSM_CTRL3_C  0x12
#define LSM_OUTZ_L_G 0x26
#define GYRO_SENS_DPS 0.070f   // 70 mdps/LSB a pleine echelle +/-2000 dps
#define GYRO_Z_SIGN  +1.0f

// ---------- PID du virage (boucle fermee sur le gyroscope) ----------
float KP_TURN = 4.0f;
float KI_TURN = 0.10f;
float KD_TURN = 0.20f;
#define TURN_PWM_MAX 230
#define TURN_INTEGRAL_MAX 200.0f
#define TURN_BRAKE_LEAD_DEG 2.0f
#define TURN_TIMEOUT_MS 8000

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
long tick_target = 0;
int tick_dir_D = 1;
int tick_dir_G = 1;

// Virage gyro
bool  imuOk = false;
float gyroZbias_dps   = 0.0f;
float turn_target_deg = 0.0f;
float gyro_angle_deg  = 0.0f;
float turn_integral   = 0.0f;
float turn_prev_error = 0.0f;
unsigned long turn_last_us  = 0;
unsigned long turn_start_ms = 0;

// Sequence V : tractrice et commandes V isolees (boucle fermee de vitesse)
struct CmdV { float vg; float vd; int dur_ms; };   // vitesses en cm/s, duree en ms
const CmdV* vseq_ptr = nullptr;
int  vseq_len = 0;
int  vseq_idx = 0;
unsigned long vseq_step_start = 0;
CmdV vseq_single;
bool escalierPending = false;

// Etat du PI de vitesse (commande V)
float vint_g = 0.0f, vint_d = 0.0f;     // integrales d'erreur
float vmes_g_f = 0.0f, vmes_d_f = 0.0f; // vitesses mesurees filtrees (cm/s)
long  vprev_eg = 0, vprev_ed = 0;       // derniers compteurs encodeurs
unsigned long vprev_us = 0;

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

// Conversion vitesse (cm/s) -> PWM signe : FEEDFORWARD du PI de vitesse.
int vitesseToPWM(float v_cms) {
  if (fabsf(v_cms) < 0.1f) return 0;
  int pwm_theorique = (int)((fabsf(v_cms) / V_MAX_CMS) * (PWM_V_MAX - PWM_MIN_V));
  int pwm_final = pwm_theorique + PWM_MIN_V;
  pwm_final = constrain(pwm_final, PWM_MIN_V, PWM_V_MAX);
  return (v_cms < 0) ? -pwm_final : pwm_final;
}

// Commande "haut niveau" pour l'asservissement FERME (F/B/L/R)
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

// Freinage actif : les deux entrees du DRV8837 a l'etat haut = mode "brake".
void brakeMoteurs() {
  ledcWrite(CH_IN1_D, 255); ledcWrite(CH_IN2_D, 255);
  ledcWrite(CH_IN1_G, 255); ledcWrite(CH_IN2_G, 255);
  delay(BRAKE_MS);
  stopMoteurs();
}

// ============================================================
//  Asservissement de VITESSE par roue (commandes V / tractrice)
// ============================================================
// Le coeur de la sequence 1 : chaque roue est asservie en vitesse
// (cm/s) avec un PI + feedforward. Schema bloc par roue :
//
//   consigne ->(+)-> PI -->(+)--> PWM --> moteur --> roue
//              (-)         (+)                        |
//               |       feedforward                   |
//               +---- vitesse mesuree (encodeur) <----+
//
void resetVitessePI() {
  vint_g = vint_d = 0.0f;
  vmes_g_f = vmes_d_f = 0.0f;
  noInterrupts(); vprev_eg = enc_gauche; vprev_ed = enc_droit; interrupts();
  vprev_us = micros();
}

void controlVitesse(float vg_cible, float vd_cible) {
  unsigned long now = micros();
  float dt = (now - vprev_us) / 1.0e6f;
  vprev_us = now;
  if (dt <= 0.0f || dt > 0.1f) dt = CONTROL_PERIOD_MS / 1000.0f;

  // 1) MESURE : vitesse reelle de chaque roue depuis les encodeurs.
  noInterrupts(); long eg = enc_gauche, ed = enc_droit; interrupts();
  // L'encodeur gauche compte NEGATIF en marche avant (cable a l'envers,
  // verifie par les logs : Enc G=-736 apres un F:20). On inverse ici.
  float vg_mes = -(float)(eg - vprev_eg) / TICKS_PER_CM / dt;
  float vd_mes =  (float)(ed - vprev_ed) / TICKS_PER_CM / dt;
  vprev_eg = eg; vprev_ed = ed;

  // Filtre passe-bas : a 1.7 cm/s on ne compte que ~0.6 tick par periode
  // de 10 ms, la mesure brute est tres quantifiee.
  vmes_g_f = 0.75f * vmes_g_f + 0.25f * vg_mes;
  vmes_d_f = 0.75f * vmes_d_f + 0.25f * vd_mes;

  // 2) PI + anti-windup (l'integrale est bornee pour que sa contribution
  //    en PWM ne depasse jamais ~220, sinon gros depassements).
  float err_g = vg_cible - vmes_g_f;
  float err_d = vd_cible - vmes_d_f;
  float imax = 220.0f / fmaxf(KI_V, 1.0f);
  vint_g = constrain(vint_g + err_g * dt, -imax, imax);
  vint_d = constrain(vint_d + err_d * dt, -imax, imax);

  int pwm_g = vitesseToPWM(vg_cible) + (int)(KP_V * err_g + KI_V * vint_g);
  int pwm_d = vitesseToPWM(vd_cible) + (int)(KP_V * err_d + KI_V * vint_d);

  // 3) KICK anti-friction statique : une roue commandee mais immobile
  //    recoit au moins PWM_KICK le temps de decoller. Sans ca, les
  //    vitesses lentes de la tractrice (PWM theorique ~80-130) ne
  //    franchissent jamais le seuil de demarrage (~200).
  if (fabsf(vg_cible) > 0.3f && fabsf(vmes_g_f) < 0.5f) {
    if (pwm_g > 0 && pwm_g <  PWM_KICK) pwm_g =  PWM_KICK;
    if (pwm_g < 0 && pwm_g > -PWM_KICK) pwm_g = -PWM_KICK;
  }
  if (fabsf(vd_cible) > 0.3f && fabsf(vmes_d_f) < 0.5f) {
    if (pwm_d > 0 && pwm_d <  PWM_KICK) pwm_d =  PWM_KICK;
    if (pwm_d < 0 && pwm_d > -PWM_KICK) pwm_d = -PWM_KICK;
  }

  ecrirePWMGauche(constrain(pwm_g, -255, 255));
  ecrirePWMDroit (constrain(pwm_d, -255, 255));
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

float lireGyroZ() {
  Wire.beginTransmission(ADDR_IMU);
  Wire.write(LSM_OUTZ_L_G);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADDR_IMU, 2);
  if (Wire.available() < 2) return 0.0f;
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  int16_t raw = (int16_t)((hi << 8) | lo);
  return ((float)raw * GYRO_SENS_DPS - gyroZbias_dps) * GYRO_Z_SIGN;
}

bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  delay(20);
  uint8_t who = imuReadReg(LSM_WHO_AM_I);
  if (who != 0x69 && who != 0x6A) return false;
  imuWriteReg(LSM_CTRL3_C, 0x44);
  imuWriteReg(LSM_CTRL2_G, 0x5C);
  delay(50);
  return true;
}

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
void startMouvement(int dirD, int dirG, unsigned long ticks) {
  noInterrupts();
  enc_gauche = 0;
  enc_droit = 0;
  interrupts();
  tick_dir_D = (dirD >= 0) ? 1 : -1;
  tick_dir_G = (dirG >= 0) ? 1 : -1;
  tick_target = (long)ticks;
  escalierPending = false;
  motionMode = MODE_TICKS;
}

// ---------- Demarrage d'un virage asservi sur le gyro (L/R) ----------
void startTurnGyro(float deg) {
  gyro_angle_deg  = 0.0f;
  turn_target_deg = deg;
  turn_integral   = 0.0f;
  turn_prev_error = deg;
  turn_last_us    = micros();
  turn_start_ms   = millis();
  escalierPending = false;
  motionMode      = MODE_TURN;
}

// ---------- Sequence escalier : tractrice originale, Vp ~ 5 cm/s ----------
// Geometrie validee en simulation (deviation max < 0.5 mm). Avec
// l'asservissement de vitesse, ces valeurs lentes sont enfin executables :
// le PI monte le PWM jusqu'a ce que la roue tourne VRAIMENT a la consigne.
static const CmdV ESCALIER[] = {
  // virage gauche (90 deg, le stylo trace l'angle, le chassis pivote)
  {-1.69f, 2.00f, 110}, {-1.37f, 2.31f, 110}, {-1.05f, 2.61f, 110},
  {-0.72f, 2.91f, 120}, {-0.39f, 3.20f, 120}, {-0.07f, 3.47f, 120},
  { 0.26f, 3.74f, 120}, { 0.59f, 4.00f, 120}, { 0.92f, 4.25f, 130},
  { 1.24f, 4.49f, 130}, { 1.56f, 4.71f, 130}, { 1.88f, 4.92f, 140},
  { 2.19f, 5.12f, 140}, { 2.49f, 5.30f, 100},
  // pause : consigne 0 = freinage asservi (le PI ramene les roues a 0)
  { 0.00f, 0.00f, 200},
  // virage droit + segment de 40 cm
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

// ---------- Séquence dynamique : Cercle (Cinématique inverse) ----------
#define N_CERCLE 40
CmdV CERCLE_BUFFER[N_CERCLE]; // Buffer global pour stocker la séquence générée

void genererCercle(float Rc) {
  float d = 13.0f;           // Offset du stylo d (cm)
  float e = TRACK_WIDTH_CM;  // Entraxe des roues (déjà défini à 8.5f)
  float T = 10.0f;           // Temps total pour faire le cercle (secondes)
  float dt = T / (float)N_CERCLE;
  
  float theta = PI / 2.0f;   // Le robot commence orienté à 90° (vers le haut)

  for(int i = 0; i < N_CERCLE; i++) {
    float t = i * dt;
    
    // 1. Vitesse du stylo requise (vitesses tangentielles)
    float vpx = -(2.0f * PI * Rc / T) * sin(2.0f * PI * t / T);
    float vpy =  (2.0f * PI * Rc / T) * cos(2.0f * PI * t / T);

    // 2. Cinématique inverse (reculer/avancer pour compenser l'offset)
    float v = vpx * cos(theta) + vpy * sin(theta);
    float w = (-vpx * sin(theta) + vpy * cos(theta)) / d;

    // 3. Calcul des vitesses de roues
    float wg = v - (e * w) / 2.0f;
    float wd = v + (e * w) / 2.0f;

    // 4. Enregistrement dans le buffer
    CERCLE_BUFFER[i].vg = wg;
    CERCLE_BUFFER[i].vd = wd;
    CERCLE_BUFFER[i].dur_ms = (int)(dt * 1000.0f);

    // Mise à jour de l'orientation virtuelle
    theta += w * dt;
  }
}

void startVSeq(const CmdV* seq, int len) {
  vseq_ptr = seq;
  vseq_len = len;
  vseq_idx = 0;
  vseq_step_start = millis();
  resetVitessePI();              // repart d'un etat propre (integrale, mesure)
  motionMode = MODE_VSEQ;
}

void controlVSeq() {
  if (vseq_ptr == nullptr || vseq_idx >= vseq_len) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> DONE VSEQ paliers=" + String(vseq_idx));
    return;
  }
  const CmdV& s = vseq_ptr[vseq_idx];
  controlVitesse(s.vg, s.vd);    // boucle FERMEE : la vitesse reelle suit la consigne
  if (millis() - vseq_step_start >= (unsigned long)s.dur_ms) {
    vseq_idx++;
    vseq_step_start = millis();
  }
}

void controlTicks() {
  noInterrupts();
  long tg = abs(enc_gauche);
  long td = abs(enc_droit);
  interrupts();

  long errG = tick_target - tg;
  long errD = tick_target - td;

  if (errG <= TOL_TICKS && errD <= TOL_TICKS) {
    brakeMoteurs();   // arret net au coin : la tractrice repart de l'arret
                      // (le PI + kick gere le redemarrage, plus besoin d'elan)
    if (escalierPending) {
      escalierPending = false;
      envoyer(">> SEQ:1 20cm OK -> tractrice");
      startVSeq(ESCALIER, N_ESCALIER);
    } else {
      motionMode = MODE_IDLE;
      envoyer(">> DONE G=" + String(tg) + " D=" + String(td));
    }
    return;
  }

  long diff = tg - td;

  int pwmG = 0, pwmD = 0;
  if (errG > TOL_TICKS)
    pwmG = constrain((int)(KP * errG - KSYNC * diff), PWM_MIN, PWM_MAX);
  if (errD > TOL_TICKS)
    pwmD = constrain((int)(KP * errD + KSYNC * diff), PWM_MIN, PWM_MAX);

  setMoteurGauche(pwmG * tick_dir_G);
  setMoteurDroit(pwmD * tick_dir_D);
}

void controlTurnGyro() {
  unsigned long now_us = micros();
  float dt = (now_us - turn_last_us) / 1.0e6f;
  turn_last_us = now_us;
  if (dt <= 0.0f || dt > 0.2f) dt = CONTROL_PERIOD_MS / 1000.0f;

  float rate = lireGyroZ();
  gyro_angle_deg += rate * dt;

  float error = turn_target_deg - gyro_angle_deg;

  if (fabs(gyro_angle_deg) >= fabs(turn_target_deg) - TURN_BRAKE_LEAD_DEG) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> DONE TURN angle=" + String(gyro_angle_deg, 1) +
            " cible=" + String(turn_target_deg, 1));
    return;
  }

  if (millis() - turn_start_ms > 400 &&
      fabs(gyro_angle_deg) > 5.0f &&
      (gyro_angle_deg * turn_target_deg) < 0.0f) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> ERR signe gyro inverse ? angle=" + String(gyro_angle_deg, 1) +
            " cible=" + String(turn_target_deg, 1) + " -> inverser GYRO_Z_SIGN");
    return;
  }

  if (millis() - turn_start_ms > TURN_TIMEOUT_MS) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    envoyer(">> DONE TURN TIMEOUT angle=" + String(gyro_angle_deg, 1));
    return;
  }

  turn_integral += error * dt;
  turn_integral = constrain(turn_integral, -TURN_INTEGRAL_MAX, TURN_INTEGRAL_MAX);
  float deriv = (error - turn_prev_error) / dt;
  turn_prev_error = error;

  float out = KP_TURN * error + KI_TURN * turn_integral + KD_TURN * deriv;

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
      startTurnGyro(+deg);
      envoyer(">> L:" + String(deg) + "deg (gyro PID)");
    } else {
      float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
      unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(+1, -1, ticks);
      envoyer(">> L:" + String(deg) + "deg = " + String(ticks) + " ticks (encodeur)");
    }

  } else if (cmd.startsWith("R:")) {
    float deg = cmd.substring(2).toFloat();
    if (imuOk) {
      startTurnGyro(-deg);
      envoyer(">> R:" + String(deg) + "deg (gyro PID)");
    } else {
      float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
      unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(-1, +1, ticks);
      envoyer(">> R:" + String(deg) + "deg = " + String(ticks) + " ticks (encodeur)");
    }

  } else if (cmd.startsWith("V:")) {
    // Vitesse roue independante, ASSERVIE : V:vG:vD:duree_ms
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
    // Extraction des paramètres. Format attendu : SEQ:num ou SEQ:num:rayon
    String s = cmd.substring(4);
    int idx = s.indexOf(':');
    int n = 0;
    float rayon = 2.0f; // Rayon par défaut si non spécifié
    
    if (idx > 0) {
      n = s.substring(0, idx).toInt();
      rayon = s.substring(idx + 1).toFloat();
    } else {
      n = s.toInt();
    }

    if (n == 1) {
      // Séquence 1 : Escalier original (précédé d'un F:20)
      unsigned long ticks = (unsigned long)(20.0f / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(+1, +1, ticks);
      escalierPending = true;
      envoyer(">> SEQ:1 escalier (20cm ferme -> tractrice asservie)");
      
    } else if (n == 2) {
      // Séquence 2 : Cercle dynamique
      genererCercle(rayon);
      escalierPending = false; // Le cercle se lance tout de suite
      startVSeq(CERCLE_BUFFER, N_CERCLE);
      envoyer(">> SEQ:2 cercle dynamique (Rayon=" + String(rayon) + "cm)");
      
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

  } else if (cmd == "W" || cmd == "w") {
    // Vitesses mesurees (mises a jour pendant une sequence V uniquement)
    envoyer("Vmes G=" + String(vmes_g_f, 2) + " D=" + String(vmes_d_f, 2) + " cm/s");

  } else if (cmd == "G" || cmd == "g") {
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

  } else if (cmd.startsWith("PIDV:")) {
    // Reglage en direct du PI de vitesse : "PIDV:kp,ki"
    String s = cmd.substring(5);
    int c1 = s.indexOf(',');
    if (c1 > 0) {
      KP_V = s.substring(0, c1).toFloat();
      KI_V = s.substring(c1 + 1).toFloat();
      envoyer(">> PI vitesse Kp=" + String(KP_V, 2) + " Ki=" + String(KI_V, 1));
    } else {
      envoyer("ERR:PIDV attend Kp,Ki");
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