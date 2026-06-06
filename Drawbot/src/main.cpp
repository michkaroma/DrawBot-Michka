#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

/*
  ═══════════════════════════════════════════════════════════════
  DRAWBOT — Firmware unifié (fusion projets "Michka" + "Gregoire")
  ═══════════════════════════════════════════════════════════════

  Repris du projet MICHKA (référence pour tout ce qui touche au robot) :
  - Constantes physiques : 993 ticks/tour, roues Ø 90 mm, entraxe 8,5 cm
  - Bas niveau moteurs : PWM 20 kHz sur IN1/IN2, EN maintenu à l'état haut
  - Asservissement en ticks (F/B), virage gyro PID (L/R)
  - Asservissement de vitesse par roue (PI + feedforward + kick anti-friction)
  - SEQ:1 escalier (20 cm asservi en ticks puis tractrice)
  - SEQ:2 cercle (cinématique inverse, rayon paramétrable)
  - IMU LSM6DS3 (gyroscope) + calibration du biais
  - Protocole TCP texte (port 8266) -> l'interface Tkinter reste compatible

  Repris du projet Gregoire :
  - WiFi + serveur HTTP (port 80) + interface HTML servie par LittleFS
  - Odométrie x / y / theta + position du feutre (alimente la carte du HTML)
  - Magnétomètre LIS3MDL : calibration, sauvegarde EEPROM, Définir Nord,
    Orienter Nord, flèche Nord (navigation multi-points PID dist + angle)
  - Routes /data, /cmd, /manual, /pid

  Supprimé (doublons devenus inutiles) :
  - Escalier "3 points de passage" et cercle par waypoints du Gregoire
    (remplacés par les séquences Michka)

  WiFi : mode AP + STA simultané.
  - AP  : réseau "Drawbot" (mdp 12345678), interface sur http://192.168.4.1
  - STA : rejoint "i.t.WORKS N300" si dispo, IP locale affichée dans le HTML

  ⚠ PlatformIO : le HTML va dans le dossier  data/index.html  du projet,
    à téléverser avec  pio run -t uploadfs.
    Ajouter dans platformio.ini :  board_build.filesystem = littlefs

  ⚠ À RE-RÉGLER sur le robot (hérités du Gregoire, le bas niveau a changé) :
    - gains PID distance/angle de la flèche Nord (KP_DIST..., KP_ANGLE...)
    - vitesses de rotation de faceNorth() et calibrateMagnetometer()
*/

// ═══════════════════════════════════════════════════════════════
// Réseau
// ═══════════════════════════════════════════════════════════════
const char* AP_SSID  = "Drawbot";
const char* AP_PASS  = "12345678";
const char* STA_SSID = "i.t.WORKS N300";
const char* STA_PASS = "";              // réseau ouvert ; renseigner si besoin

WebServer server(80);                   // interface HTML
WiFiServer tcpServer(8266);             // interface Tkinter (protocole texte)
WiFiClient tcpClient;

// ═══════════════════════════════════════════════════════════════
// Brochage (cf. kick-off slide 6)
// ═══════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════
// PWM (LEDC) — schéma Michka : PWM sur IN1/IN2 à 20 kHz, EN à 1
// ═══════════════════════════════════════════════════════════════
#define LEDC_FREQ 20000
#define LEDC_RES 8
#define CH_IN1_D 0
#define CH_IN2_D 1
#define CH_IN1_G 2
#define CH_IN2_G 3

// ═══════════════════════════════════════════════════════════════
// Calibration mécanique (valeurs Michka, validées par les séquences)
// ═══════════════════════════════════════════════════════════════
#define TICKS_PER_REV 993
#define WHEEL_DIAM_MM 90.0f
#define WHEEL_CIRCUM_CM (PI * WHEEL_DIAM_MM / 10.0f)
#define TICKS_PER_CM (TICKS_PER_REV / WHEEL_CIRCUM_CM)   // ~35.1 ticks/cm
#define TRACK_WIDTH_CM 8.5f

// !!! A RECALIBRER !!! (utilisé uniquement par le repli encodeur des virages)
#define TURN_CORRECTION 1.0f

// Mêmes grandeurs en unités SI pour l'odométrie (carte du HTML)
const float RAYON_ROUE_M   = (WHEEL_DIAM_MM / 2.0f) / 1000.0f;  // 0.045 m
const float ENTRAXE_M      = TRACK_WIDTH_CM / 100.0f;           // 0.085 m
const float PERIMETRE_M    = 2.0f * PI * RAYON_ROUE_M;          // ~0.2827 m
const float OFFSET_STYLO_M = 0.13f;                             // feutre 13 cm devant l'axe

// ═══════════════════════════════════════════════════════════════
// Asservissement en ticks (trajets droits F/B) — Michka
// ═══════════════════════════════════════════════════════════════
#define PWM_MIN 200            // seuil de friction statique (mesuré)
#define PWM_MAX 240
#define KP 1.8f                // gain proportionnel (ticks -> PWM)
#define KSYNC 1.2f             // gain de synchro entre les deux roues
#define TOL_TICKS 4            // tolérance d'arrêt (~1 mm de roue)
#define CONTROL_PERIOD_MS 10   // période d'échantillonnage : 100 Hz
#define BRAKE_MS 80            // durée du freinage actif

// ═══════════════════════════════════════════════════════════════
// Asservissement de VITESSE par roue (PI + feedforward) — Michka
// ═══════════════════════════════════════════════════════════════
// vitesseToPWM() ne sert que de FEEDFORWARD : PWM théorique pour une
// vitesse donnée (calibrage linéaire V_MAX_CMS <-> PWM_V_MAX, mesuré).
// La boucle PI (controlVitesse) corrige ensuite avec la vitesse RÉELLE
// mesurée par les encodeurs. Un "kick" anti-friction statique force
// PWM_KICK quand une roue commandée est encore immobile.
#define V_MAX_CMS  29.0f       // vitesse (cm/s) atteinte à PWM_V_MAX (mesuré)
#define PWM_V_MAX  245         // PWM correspondant à V_MAX_CMS
#define PWM_MIN_V  70          // PWM mini où la roue tourne encore
#define PWM_KICK   200         // coup de démarrage anti-friction statique

// Gains du PI de vitesse, réglables en direct via "PIDV:kp,ki" (TCP)
float KP_V = 8.0f;             // (cm/s d'erreur) -> PWM
float KI_V = 300.0f;           // rattrape la friction ; anti-windup intégré

// ═══════════════════════════════════════════════════════════════
// IMU LSM6DS3 (gyroscope, I2C) — Michka
// ═══════════════════════════════════════════════════════════════
#define ADDR_IMU     0x6B
#define LSM_WHO_AM_I 0x0F
#define LSM_CTRL2_G  0x11
#define LSM_CTRL3_C  0x12
#define LSM_OUTZ_L_G 0x26
#define GYRO_SENS_DPS 0.070f   // 70 mdps/LSB à pleine échelle +/-2000 dps
#define GYRO_Z_SIGN  +1.0f

// PID du virage (boucle fermée sur le gyroscope), réglable via "PIDT:kp,ki,kd"
float KP_TURN = 4.0f;
float KI_TURN = 0.10f;
float KD_TURN = 0.20f;
#define TURN_PWM_MAX 230
#define TURN_INTEGRAL_MAX 200.0f
#define TURN_BRAKE_LEAD_DEG 2.0f
#define TURN_TIMEOUT_MS 8000

// ═══════════════════════════════════════════════════════════════
// Magnétomètre LIS3MDL — Gregoire
// ═══════════════════════════════════════════════════════════════
#define ADDR_MAG 0x1E
#define EEPROM_SIZE 64
#define AUTO_CALIB_FLAG 0xCC

float mag_offset_x = 0.0f;
float mag_offset_y = 0.0f;
float magnetic_north_angle = 0.0f;
bool  auto_calibrated = false;

int16_t mag_x_raw = 0;
int16_t mag_y_raw = 0;
int16_t mag_z_raw = 0;
float   mag_heading = -1.0f;

// ═══════════════════════════════════════════════════════════════
// Navigation flèche Nord (PID distance + angle) — Gregoire
// ⚠ Gains hérités du Gregoire : à re-régler avec le bas niveau Michka.
// ═══════════════════════════════════════════════════════════════
#define MAX_POINTS 160

float VITESSE_MAX = 240.0f;
float VITESSE_MIN = 200.0f;        // = seuil de friction du bas niveau Michka
float DISTANCE_TOLERANCE = 0.012f; // m
float ANGLE_TOLERANCE = 0.05f;     // rad

float KP_DIST = 900.0f;
float KI_DIST = 50.0f;
float KD_DIST = 150.0f;

float KP_ANGLE = 145.0f;
float KI_ANGLE = 15.0f;
float KD_ANGLE = 40.0f;

float erreurDist_precedente = 0.0f;
float integrale_dist = 0.0f;
float erreurAngle_precedente = 0.0f;
float integrale_angle = 0.0f;
unsigned long temps_precedent = 0;

struct Point {
  float x;
  float y;
};

Point PARCOURS[MAX_POINTS];
int  NB_POINTS = 0;
int  pointActuel = 0;
bool parcoursCharge = false;
bool parcoursFini = false;
bool objectifAtteint = false;
bool estModeFlecheNord = false;
unsigned long tempsArrivePoint = 0;
const unsigned long PAUSE_ENTRE_POINTS = 1000;

float vG_prev = 0.0f;
float vD_prev = 0.0f;

// ═══════════════════════════════════════════════════════════════
// Odométrie — Gregoire (mais avec les constantes Michka)
// ═══════════════════════════════════════════════════════════════
// Convention encodeurs : marche AVANT = ticks POSITIFS pour les deux
// roues (le câblage inversé de la roue gauche est corrigé dans l'ISR).
volatile long ticksG = 0;
volatile long ticksD = 0;
long lastTicksG = 0;
long lastTicksD = 0;

double x_robot = -OFFSET_STYLO_M;   // feutre en (0,0) au départ
double y_robot = 0.0;
double theta_robot = 0.0;

// ═══════════════════════════════════════════════════════════════
// État du mouvement (modes Michka)
// ═══════════════════════════════════════════════════════════════
enum MotionMode { MODE_IDLE, MODE_TICKS, MODE_TURN, MODE_VSEQ };
MotionMode motionMode = MODE_IDLE;

// Trajets droits asservis en ticks (et repli virage encodeur).
// Plus de remise à zéro des compteurs (ça casserait l'odométrie) :
// on mémorise la valeur de départ et on vise un écart relatif.
long tick_target = 0;
long tick_ref_G = 0;
long tick_ref_D = 0;
int  tick_dir_D = 1;
int  tick_dir_G = 1;

// Virage gyro
bool  imuOk = false;
float gyroZbias_dps   = 0.0f;
float turn_target_deg = 0.0f;
float gyro_angle_deg  = 0.0f;
float turn_integral   = 0.0f;
float turn_prev_error = 0.0f;
unsigned long turn_last_us  = 0;
unsigned long turn_start_ms = 0;

// Séquence V : tractrice / cercle (boucle fermée de vitesse)
struct CmdV { float vg; float vd; int dur_ms; };   // cm/s, durée en ms
const CmdV* vseq_ptr = nullptr;
int  vseq_len = 0;
int  vseq_idx = 0;
unsigned long vseq_step_start = 0;
CmdV vseq_single;
bool escalierPending = false;

// État du PI de vitesse
float vint_g = 0.0f, vint_d = 0.0f;     // intégrales d'erreur
float vmes_g_f = 0.0f, vmes_d_f = 0.0f; // vitesses mesurées filtrées (cm/s)
long  vprev_eg = 0, vprev_ed = 0;       // derniers compteurs encodeurs
unsigned long vprev_us = 0;

// ═══════════════════════════════════════════════════════════════
// État global
// ═══════════════════════════════════════════════════════════════
String etatRobot = "idle";
String lastMessage = "Pret";
bool   stopRequested = false;   // interrompt les boucles bloquantes (faceNorth, calib)
bool   seq2Active = false;      // un cercle Michka est chargé / en cours
float  rayonCercle = 0.0f;

// ═══════════════════════════════════════════════════════════════
// Prototypes
// ═══════════════════════════════════════════════════════════════
void   envoyer(const String& msg);
void   logMsg(const String& msg);
double normaliserAngle(double angle);
float  normalizeDeg360(float angle);
float  corrigerEstOuest(float heading);
void   getPositionStylo(double& x_stylo, double& y_stylo);
void   copyTicks(long& d, long& g);
void   ecrirePWMDroit(int v);
void   ecrirePWMGauche(int v);
int    vitesseToPWM(float v_cms);
void   setMoteurDroit(int vitesse);
void   setMoteurGauche(int vitesse);
void   setMoteurs(float vitesseGauche, float vitesseDroite);
void   stopMoteurs();
void   brakeMoteurs();
void   resetVitessePI();
void   controlVitesse(float vg_cible, float vd_cible);
void   imuWriteReg(uint8_t reg, uint8_t val);
uint8_t imuReadReg(uint8_t reg);
float  lireGyroZ();
bool   imuInit();
void   calibrerGyro(int n = 400);
void   initMagnetometer();
bool   readRawMagnetometer(int16_t& x, int16_t& y, int16_t& z);
bool   isValidMagReading(int16_t x, int16_t y);
float  calculateHeading(int16_t x, int16_t y);
float  headingFromRaw(int16_t x, int16_t y);
float  getHeadingFromNorth();
void   saveMagCalibration();
bool   loadMagCalibration();
void   calibrateMagnetometer();
void   setCurrentAsNorth();
void   faceNorth();
void   mettreAJourOdometrie();
void   resetParcours();
void   startMouvement(int dirD, int dirG, unsigned long ticks);
void   startTurnGyro(float deg);
void   genererCercle(float Rc);
void   startVSeq(const CmdV* seq, int len);
void   controlVSeq();
void   controlTicks();
void   controlTurnGyro();
void   lancerSeq1();
void   lancerSeq2(float rayon);
void   stopTout();
float  calculerPID_Distance(float erreur, float dt);
float  calculerPID_Angle(float erreur, float dt);
void   loadNorthArrowSimplePath();
void   drawNorthArrowFixed();
void   navigationMultiPoints();
void   traiterCommande(const String& cmd);
void   gererTcp();
void   gererSerial();
String jsonEscape(String s);
String mimeType(const String& path);
void   handleIndex();
void   handleData();
void   handleCommand();
void   handleManual();
void   handlePID();

// ═══════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════
// envoyer() : sortie "console" -> client TCP (Tkinter) + port série.
void envoyer(const String& msg) {
  if (tcpClient && tcpClient.connected()) tcpClient.println(msg);
  Serial.println(msg);
}

// logMsg() : comme envoyer(), mais mémorise aussi le message pour /data
// (champ "Message" de l'interface HTML). À utiliser pour les événements ;
// les simples réponses aux requêtes (E, W, G) passent par envoyer().
void logMsg(const String& msg) {
  lastMessage = msg;
  envoyer(msg);
}

double normaliserAngle(double angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

float normalizeDeg360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

// Après "Définir Nord", Nord et Sud sont bons mais Est/Ouest inversés.
// On inverse donc le sens angulaire : 0 reste 0, 90 devient 270, etc.
float corrigerEstOuest(float heading) {
  if (heading < 0.0f) return heading;
  return normalizeDeg360(360.0f - heading);
}

void getPositionStylo(double& x_stylo, double& y_stylo) {
  x_stylo = x_robot + OFFSET_STYLO_M * cos(theta_robot);
  y_stylo = y_robot + OFFSET_STYLO_M * sin(theta_robot);
}

// ═══════════════════════════════════════════════════════════════
// Encodeurs
// ═══════════════════════════════════════════════════════════════
// Convention : marche avant = +1 des deux côtés. La roue gauche compte
// "à l'envers" à cause du câblage (vérifié sur Michka : Enc G négatif
// après un F:20) : la correction de signe est faite ICI, une fois pour
// toutes. Tout le reste du code (PI vitesse, odométrie) suppose des
// ticks positifs en marche avant.
void IRAM_ATTR isr_enc_g() {
  ticksG += (digitalRead(ENC_G_CH_B) == HIGH) ? -1 : 1;
}
void IRAM_ATTR isr_enc_d() {
  ticksD += (digitalRead(ENC_D_CH_B) == HIGH) ? 1 : -1;
}

void copyTicks(long& d, long& g) {
  noInterrupts();
  d = ticksD;
  g = ticksG;
  interrupts();
}

// ═══════════════════════════════════════════════════════════════
// Bas niveau moteurs (schéma Michka)
// ═══════════════════════════════════════════════════════════════
// Écriture PWM "brute" : sens + saturation uniquement, SANS plancher de
// friction. Le moteur gauche a IN1/IN2 câblés à l'envers du droit :
// l'inversion de sens est gérée ICI, de façon centralisée. v=0 => roue libre.
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

// Conversion vitesse (cm/s) -> PWM signé : FEEDFORWARD du PI de vitesse.
int vitesseToPWM(float v_cms) {
  if (fabsf(v_cms) < 0.1f) return 0;
  int pwm_theorique = (int)((fabsf(v_cms) / V_MAX_CMS) * (PWM_V_MAX - PWM_MIN_V));
  int pwm_final = pwm_theorique + PWM_MIN_V;
  pwm_final = constrain(pwm_final, PWM_MIN_V, PWM_V_MAX);
  return (v_cms < 0) ? -pwm_final : pwm_final;
}

// Commande "haut niveau" avec plancher anti-friction (asservissements F/B,
// virages, navigation flèche Nord, manuel) : toute consigne non nulle est
// remontée au seuil de démarrage mesuré.
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

// Wrapper pour le code hérité du Gregoire (navigation flèche Nord, faceNorth,
// calibration magnétomètre, mode manuel) qui travaille en float.
void setMoteurs(float vitesseGauche, float vitesseDroite) {
  setMoteurGauche((int)vitesseGauche);
  setMoteurDroit((int)vitesseDroite);
}

void stopMoteurs() {
  setMoteurDroit(0);
  setMoteurGauche(0);
}

// Freinage actif : les deux entrées du DRV8837 à l'état haut = mode "brake".
void brakeMoteurs() {
  ledcWrite(CH_IN1_D, 255); ledcWrite(CH_IN2_D, 255);
  ledcWrite(CH_IN1_G, 255); ledcWrite(CH_IN2_G, 255);
  delay(BRAKE_MS);
  stopMoteurs();
}

// ═══════════════════════════════════════════════════════════════
//  Asservissement de VITESSE par roue (séquences V / tractrice / cercle)
// ═══════════════════════════════════════════════════════════════
// Le cœur des séquences 1 et 2 : chaque roue est asservie en vitesse
// (cm/s) avec un PI + feedforward. Schéma bloc par roue :
//
//   consigne ->(+)-> PI -->(+)--> PWM --> moteur --> roue
//              (-)         (+)                        |
//               |       feedforward                   |
//               +---- vitesse mesurée (encodeur) <----+
//
void resetVitessePI() {
  vint_g = vint_d = 0.0f;
  vmes_g_f = vmes_d_f = 0.0f;
  copyTicks(vprev_ed, vprev_eg);
  vprev_us = micros();
}

void controlVitesse(float vg_cible, float vd_cible) {
  unsigned long now = micros();
  float dt = (now - vprev_us) / 1.0e6f;
  vprev_us = now;
  if (dt <= 0.0f || dt > 0.1f) dt = CONTROL_PERIOD_MS / 1000.0f;

  // 1) MESURE : vitesse réelle de chaque roue depuis les encodeurs.
  //    (Les signes sont déjà corrigés dans les ISR : avant = positif.)
  long eg, ed;
  copyTicks(ed, eg);
  float vg_mes = (float)(eg - vprev_eg) / TICKS_PER_CM / dt;
  float vd_mes = (float)(ed - vprev_ed) / TICKS_PER_CM / dt;
  vprev_eg = eg; vprev_ed = ed;

  // Filtre passe-bas : à 1.7 cm/s on ne compte que ~0.6 tick par période
  // de 10 ms, la mesure brute est très quantifiée.
  vmes_g_f = 0.75f * vmes_g_f + 0.25f * vg_mes;
  vmes_d_f = 0.75f * vmes_d_f + 0.25f * vd_mes;

  // 2) PI + anti-windup (l'intégrale est bornée pour que sa contribution
  //    en PWM ne dépasse jamais ~220, sinon gros dépassements).
  float err_g = vg_cible - vmes_g_f;
  float err_d = vd_cible - vmes_d_f;
  float imax = 220.0f / fmaxf(KI_V, 1.0f);
  vint_g = constrain(vint_g + err_g * dt, -imax, imax);
  vint_d = constrain(vint_d + err_d * dt, -imax, imax);

  int pwm_g = vitesseToPWM(vg_cible) + (int)(KP_V * err_g + KI_V * vint_g);
  int pwm_d = vitesseToPWM(vd_cible) + (int)(KP_V * err_d + KI_V * vint_d);

  // 3) KICK anti-friction statique : une roue commandée mais immobile
  //    reçoit au moins PWM_KICK le temps de décoller.
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

// ═══════════════════════════════════════════════════════════════
//  IMU LSM6DS3 : configuration et lecture du gyroscope
// ═══════════════════════════════════════════════════════════════
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

// Le bus I2C est initialisé une seule fois dans setup() (Wire.begin),
// l'IMU et le magnétomètre ne font que leur configuration de registres.
bool imuInit() {
  uint8_t who = imuReadReg(LSM_WHO_AM_I);
  if (who != 0x69 && who != 0x6A) return false;
  imuWriteReg(LSM_CTRL3_C, 0x44);
  imuWriteReg(LSM_CTRL2_G, 0x5C);
  delay(50);
  return true;
}

void calibrerGyro(int n) {
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

// ═══════════════════════════════════════════════════════════════
// Magnétomètre LIS3MDL (chaîne boussole du Gregoire)
// ═══════════════════════════════════════════════════════════════
void initMagnetometer() {
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x20);
  Wire.write(0x90); // Medium performance, 10 Hz
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x21);
  Wire.write(0x00); // ±4 gauss
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x22);
  Wire.write(0x00); // Continuous mode
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x23);
  Wire.write(0x08); // Z high performance
  Wire.endTransmission();

  logMsg("Magnetometre initialise");
}

bool readRawMagnetometer(int16_t& x, int16_t& y, int16_t& z) {
  Wire.beginTransmission(ADDR_MAG);
  Wire.write(0x28);
  int error = Wire.endTransmission(false);

  if (error != 0) return false;

  Wire.requestFrom(ADDR_MAG, 6);
  if (Wire.available() < 6) return false;

  uint8_t xLow = Wire.read();
  uint8_t xHigh = Wire.read();
  uint8_t yLow = Wire.read();
  uint8_t yHigh = Wire.read();
  uint8_t zLow = Wire.read();
  uint8_t zHigh = Wire.read();

  x = (int16_t)((xHigh << 8) | xLow);
  y = (int16_t)((yHigh << 8) | yLow);
  z = (int16_t)((zHigh << 8) | zLow);

  mag_x_raw = x;
  mag_y_raw = y;
  mag_z_raw = z;

  return true;
}

bool isValidMagReading(int16_t x, int16_t y) {
  return (abs(x) < 30000 && abs(y) < 30000 && (abs(x) > 10 || abs(y) > 10));
}

float calculateHeading(int16_t x, int16_t y) {
  float x_cal = (float)x - mag_offset_x;
  float y_cal = (float)y - mag_offset_y;

  float heading = atan2(y_cal, x_cal) * 180.0f / PI;
  if (heading < 0.0f) heading += 360.0f;
  return heading;
}

// Cap par rapport au Nord défini, à partir d'une mesure brute déjà lue.
float headingFromRaw(int16_t x, int16_t y) {
  if (!isValidMagReading(x, y)) return -1.0f;

  float current_heading = calculateHeading(x, y);
  float heading_from_north = current_heading - magnetic_north_angle;

  while (heading_from_north < 0.0f) heading_from_north += 360.0f;
  while (heading_from_north >= 360.0f) heading_from_north -= 360.0f;

  heading_from_north = corrigerEstOuest(heading_from_north);

  mag_heading = heading_from_north;
  return heading_from_north;
}

float getHeadingFromNorth() {
  int16_t x, y, z;
  if (!readRawMagnetometer(x, y, z)) return -1.0f;
  return headingFromRaw(x, y);
}

void saveMagCalibration() {
  EEPROM.write(0, AUTO_CALIB_FLAG);
  EEPROM.put(4, mag_offset_x);
  EEPROM.put(8, mag_offset_y);
  EEPROM.put(12, magnetic_north_angle);
  EEPROM.write(16, auto_calibrated ? 1 : 0);
  EEPROM.commit();
  logMsg("Calibration magnetometre sauvegardee");
}

bool loadMagCalibration() {
  if (EEPROM.read(0) != AUTO_CALIB_FLAG) {
    return false;
  }

  EEPROM.get(4, mag_offset_x);
  EEPROM.get(8, mag_offset_y);
  EEPROM.get(12, magnetic_north_angle);
  auto_calibrated = (EEPROM.read(16) == 1);

  return auto_calibrated;
}

void calibrateMagnetometer() {
  logMsg("Calibration magnetometre : rotation du robot");
  etatRobot = "mag_calibration";
  stopRequested = false;

  const int max_samples = 180;
  int sample_count = 0;

  int16_t x_min = 32767, x_max = -32768;
  int16_t y_min = 32767, y_max = -32768;

  // ⚠ A AJUSTER : 160 avec l'ancien bas niveau ; avec le PWM Michka le
  // seuil de friction est ~200, on tourne donc un peu au-dessus.
  float rotation_speed = 215.0f;
  unsigned long start_time = millis();
  unsigned long last_sample_time = 0;

  // Rotation pendant 20 s max. Plus robuste que dépendre de theta_robot.
  while (sample_count < max_samples && (millis() - start_time) < 20000) {
    server.handleClient();
    gererTcp();
    mettreAJourOdometrie();
    if (stopRequested) break;

    if ((millis() - last_sample_time) > 100) {
      int16_t x, y, z;
      if (readRawMagnetometer(x, y, z) && isValidMagReading(x, y)) {
        x_min = min(x_min, x);
        x_max = max(x_max, x);
        y_min = min(y_min, y);
        y_max = max(y_max, y);
        sample_count++;
        last_sample_time = millis();
      }
    }

    setMoteurs(-rotation_speed, rotation_speed);
    delay(20);
  }

  brakeMoteurs();

  if (stopRequested) {
    etatRobot = "mag_calib_aborted";
    logMsg("Calibration interrompue");
    return;
  }

  if (sample_count > 50) {
    mag_offset_x = (float)(x_max + x_min) / 2.0f;
    mag_offset_y = (float)(y_max + y_min) / 2.0f;

    // Comme dans le projet d'origine : la rotation a pollue la pose,
    // on remet le repere a zero une fois la calibration terminee.
    resetParcours();

    // La calibration ne définit pas le Nord : elle calcule seulement les
    // offsets X/Y. Ensuite : placer le robot vers le vrai Nord puis
    // cliquer "Définir Nord".
    auto_calibrated = true;
    saveMagCalibration();
    etatRobot = "mag_calibrated_offsets_only";
    logMsg("Calibration offsets OK. Place le robot vers le vrai Nord puis clique Definir Nord.");
  } else {
    auto_calibrated = false;
    etatRobot = "mag_calib_failed";
    logMsg("Calibration echouee : pas assez d'echantillons");
  }
}

void setCurrentAsNorth() {
  int16_t x, y, z;
  if (!readRawMagnetometer(x, y, z) || !isValidMagReading(x, y)) {
    etatRobot = "set_north_failed";
    logMsg("Impossible de lire le magnetometre pour definir le Nord");
    return;
  }

  // On sauvegarde le cap brut courant comme référence Nord.
  // getHeadingFromNorth() appliquera ensuite la correction Est/Ouest.
  magnetic_north_angle = calculateHeading(x, y);
  auto_calibrated = true;
  saveMagCalibration();

  mag_heading = 0.0f;
  etatRobot = "north_defined";
  logMsg("Nord defini = position actuelle");
}

void faceNorth() {
  if (!auto_calibrated) {
    logMsg("Magnetometre non calibre : impossible de s'orienter Nord");
    return;
  }

  etatRobot = "face_north";
  logMsg("Orientation vers le Nord...");
  stopRequested = false;

  unsigned long start_time = millis();
  unsigned long timeout = 18000;
  int stableCount = 0;

  while ((millis() - start_time) < timeout) {
    server.handleClient();
    gererTcp();
    mettreAJourOdometrie();
    if (stopRequested) break;

    float current = getHeadingFromNorth();
    if (current < 0) {
      stopMoteurs();
      delay(40);
      continue;
    }

    // Erreur courte vers 0° : plage -180° à +180°.
    float error = 0.0f - current;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    if (fabs(error) <= 4.0f) {
      stopMoteurs();
      stableCount++;
      if (stableCount >= 10) break; // stable environ 0,5 s
      delay(50);
      continue;
    }

    stableCount = 0;

    // ⚠ A AJUSTER : avec le bas niveau Michka, en dessous de ~200 de PWM
    // le robot ne tourne pas (plancher appliqué par setMoteurs de toute
    // façon). Plage volontairement resserrée juste au-dessus du seuil.
    float turn_speed = constrain(fabs(error) * 1.5f, 205.0f, 235.0f);

    // Si le robot part du mauvais côté, inverser simplement les deux
    // lignes suivantes.
    if (error > 0.0f) {
      setMoteurs(turn_speed, -turn_speed);
    } else {
      setMoteurs(-turn_speed, turn_speed);
    }

    delay(50);
  }

  brakeMoteurs();
  delay(400);

  if (stopRequested) {
    etatRobot = "face_north_aborted";
    logMsg("Orientation Nord interrompue");
    return;
  }

  float final_heading = getHeadingFromNorth();
  etatRobot = "face_north_done";
  logMsg("Orientation Nord terminee. Cap=" + String(final_heading, 1) + " deg");
}

// ═══════════════════════════════════════════════════════════════
// Odométrie (carte de l'interface HTML) — constantes Michka
// ═══════════════════════════════════════════════════════════════
// Tourne en permanence dans loop(), quel que soit le mode de mouvement :
// la carte trace donc aussi l'escalier et le cercle (séquences Michka).
void mettreAJourOdometrie() {
  long currentG, currentD;
  copyTicks(currentD, currentG);

  long dG = currentG - lastTicksG;
  long dD = currentD - lastTicksD;

  if (dG == 0 && dD == 0) return;

  lastTicksG = currentG;
  lastTicksD = currentD;

  double distG = ((double)dG * PERIMETRE_M) / TICKS_PER_REV;
  double distD = ((double)dD * PERIMETRE_M) / TICKS_PER_REV;

  double dDist = (distG + distD) / 2.0;
  double dTheta = (distD - distG) / ENTRAXE_M;

  if (fabs(dTheta) > 0.0001) {
    double R = dDist / dTheta;
    double dx = R * (sin(theta_robot + dTheta) - sin(theta_robot));
    double dy = R * (cos(theta_robot) - cos(theta_robot + dTheta));
    x_robot += dx;
    y_robot += dy;
  } else {
    x_robot += dDist * cos(theta_robot);
    y_robot += dDist * sin(theta_robot);
  }

  theta_robot = normaliserAngle(theta_robot + dTheta);
}

// Remise à zéro du repère : stylo en (0,0), orientation 0.
// Arrête aussi tout mouvement en cours (les cibles Michka sont relatives
// aux compteurs, on ne remet pas les ticks à zéro pendant un mouvement).
void resetParcours() {
  motionMode = MODE_IDLE;
  escalierPending = false;

  noInterrupts();
  ticksG = 0;
  ticksD = 0;
  interrupts();
  lastTicksG = 0;
  lastTicksD = 0;

  x_robot = -OFFSET_STYLO_M;
  y_robot = 0.0;
  theta_robot = 0.0;

  pointActuel = 0;
  objectifAtteint = false;
  parcoursFini = false;
  estModeFlecheNord = false;

  integrale_dist = 0.0f;
  integrale_angle = 0.0f;
  erreurDist_precedente = 0.0f;
  erreurAngle_precedente = 0.0f;

  vG_prev = 0.0f;
  vD_prev = 0.0f;

  etatRobot = "reset";
  logMsg("Reset : stylo en (0,0)");
}

// ═══════════════════════════════════════════════════════════════
// Mouvements asservis Michka
// ═══════════════════════════════════════════════════════════════
// Démarrage d'un mouvement asservi en ticks (F/B).
// Cibles RELATIVES : on mémorise les compteurs au départ au lieu de les
// remettre à zéro, pour ne pas perturber l'odométrie de la carte.
void startMouvement(int dirD, int dirG, unsigned long ticks) {
  copyTicks(tick_ref_D, tick_ref_G);
  tick_dir_D = (dirD >= 0) ? 1 : -1;
  tick_dir_G = (dirG >= 0) ? 1 : -1;
  tick_target = (long)ticks;
  escalierPending = false;
  parcoursCharge = false;          // coupe une éventuelle flèche Nord
  estModeFlecheNord = false;
  motionMode = MODE_TICKS;
  etatRobot = "move_ticks";
}

// Démarrage d'un virage asservi sur le gyro (L/R)
void startTurnGyro(float deg) {
  gyro_angle_deg  = 0.0f;
  turn_target_deg = deg;
  turn_integral   = 0.0f;
  turn_prev_error = deg;
  turn_last_us    = micros();
  turn_start_ms   = millis();
  escalierPending = false;
  parcoursCharge  = false;
  estModeFlecheNord = false;
  motionMode      = MODE_TURN;
  etatRobot = "turn_gyro";
}

// ---------- Séquence 1 : escalier, tractrice originale, Vp ~ 5 cm/s ----------
// Géométrie validée en simulation (déviation max < 0.5 mm). Avec
// l'asservissement de vitesse, ces valeurs lentes sont exécutables :
// le PI monte le PWM jusqu'à ce que la roue tourne VRAIMENT à la consigne.
static const CmdV ESCALIER[] = {
  // virage gauche (90 deg, le stylo trace l'angle, le châssis pivote)
  {-1.69f, 2.00f, 110}, {-1.37f, 2.31f, 110}, {-1.05f, 2.61f, 110},
  {-0.72f, 2.91f, 120}, {-0.39f, 3.20f, 120}, {-0.07f, 3.47f, 120},
  { 0.26f, 3.74f, 120}, { 0.59f, 4.00f, 120}, { 0.92f, 4.25f, 130},
  { 1.24f, 4.49f, 130}, { 1.56f, 4.71f, 130}, { 1.88f, 4.92f, 140},
  { 2.19f, 5.12f, 140}, { 2.49f, 5.30f, 100},
  // pause : consigne 0 = freinage asservi (le PI ramène les roues à 0)
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

// ---------- Séquence 2 : cercle dynamique (cinématique inverse) ----------
#define N_CERCLE 40
CmdV CERCLE_BUFFER[N_CERCLE]; // Buffer global pour la séquence générée

void genererCercle(float Rc) {
  float d = OFFSET_STYLO_M * 100.0f;  // offset du stylo (cm) : 13 cm
  float e = TRACK_WIDTH_CM;           // entraxe des roues (cm)
  float T = 10.0f;                    // temps total pour faire le cercle (s)
  float dt = T / (float)N_CERCLE;

  float theta = PI / 2.0f;   // repère interne du calcul (robot "vers le haut")

  for (int i = 0; i < N_CERCLE; i++) {
    float t = i * dt;

    // 1. Vitesse du stylo requise (vitesses tangentielles)
    float vpx = -(2.0f * PI * Rc / T) * sin(2.0f * PI * t / T);
    float vpy =  (2.0f * PI * Rc / T) * cos(2.0f * PI * t / T);

    // 2. Cinématique inverse (compense l'offset du stylo)
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
  parcoursCharge = false;
  estModeFlecheNord = false;
  resetVitessePI();              // repart d'un état propre (intégrale, mesure)
  motionMode = MODE_VSEQ;
}

void controlVSeq() {
  if (vseq_ptr == nullptr || vseq_idx >= vseq_len) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    seq2Active = false;
    etatRobot = "done";
    logMsg(">> DONE VSEQ paliers=" + String(vseq_idx));
    return;
  }
  const CmdV& s = vseq_ptr[vseq_idx];
  controlVitesse(s.vg, s.vd);    // boucle FERMÉE : la vitesse réelle suit la consigne
  if (millis() - vseq_step_start >= (unsigned long)s.dur_ms) {
    vseq_idx++;
    vseq_step_start = millis();
  }
}

void controlTicks() {
  long g, d;
  copyTicks(d, g);
  long tg = labs(g - tick_ref_G);
  long td = labs(d - tick_ref_D);

  long errG = tick_target - tg;
  long errD = tick_target - td;

  if (errG <= TOL_TICKS && errD <= TOL_TICKS) {
    brakeMoteurs();   // arrêt net au coin : la tractrice repart de l'arrêt
                      // (le PI + kick gère le redémarrage, plus besoin d'élan)
    if (escalierPending) {
      escalierPending = false;
      logMsg(">> SEQ:1 20cm OK -> tractrice");
      etatRobot = "seq1_tractrice";
      startVSeq(ESCALIER, N_ESCALIER);
    } else {
      motionMode = MODE_IDLE;
      etatRobot = "done";
      logMsg(">> DONE G=" + String(tg) + " D=" + String(td));
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
    etatRobot = "done";
    logMsg(">> DONE TURN angle=" + String(gyro_angle_deg, 1) +
           " cible=" + String(turn_target_deg, 1));
    return;
  }

  if (millis() - turn_start_ms > 400 &&
      fabs(gyro_angle_deg) > 5.0f &&
      (gyro_angle_deg * turn_target_deg) < 0.0f) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    etatRobot = "error";
    logMsg(">> ERR signe gyro inverse ? angle=" + String(gyro_angle_deg, 1) +
           " cible=" + String(turn_target_deg, 1) + " -> inverser GYRO_Z_SIGN");
    return;
  }

  if (millis() - turn_start_ms > TURN_TIMEOUT_MS) {
    brakeMoteurs();
    motionMode = MODE_IDLE;
    etatRobot = "done";
    logMsg(">> DONE TURN TIMEOUT angle=" + String(gyro_angle_deg, 1));
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

// ---------- Lancement des séquences (partagé HTML + Tkinter) ----------
void lancerSeq1() {
  // Séquence 1 : escalier (20 cm asservi en ticks puis tractrice)
  seq2Active = false;
  unsigned long ticks = (unsigned long)(20.0f / WHEEL_CIRCUM_CM * TICKS_PER_REV);
  startMouvement(+1, +1, ticks);
  escalierPending = true;
  etatRobot = "seq1_running";
  logMsg(">> SEQ:1 escalier (20cm ferme -> tractrice asservie)");
}

void lancerSeq2(float rayon) {
  // Séquence 2 : cercle dynamique, rayon paramétrable (2 à 20 cm)
  rayon = constrain(rayon, 2.0f, 20.0f);
  rayonCercle = rayon;
  seq2Active = true;
  genererCercle(rayon);
  startVSeq(CERCLE_BUFFER, N_CERCLE);
  etatRobot = "seq2_running";
  logMsg(">> SEQ:2 cercle dynamique (Rayon=" + String(rayon) + "cm)");
}

// Arrêt général : coupe les modes Michka, la flèche Nord et les boucles
// bloquantes (faceNorth, calibration magnétomètre).
void stopTout() {
  motionMode = MODE_IDLE;
  escalierPending = false;
  seq2Active = false;
  parcoursCharge = false;
  parcoursFini = true;
  estModeFlecheNord = false;
  stopRequested = true;
  brakeMoteurs();
  etatRobot = "stopped";
  logMsg(">> Stop");
}

// ═══════════════════════════════════════════════════════════════
// PID de navigation (flèche Nord) — Gregoire
// ═══════════════════════════════════════════════════════════════
float calculerPID_Distance(float erreur, float dt) {
  float P = KP_DIST * erreur;

  integrale_dist += erreur * dt;
  integrale_dist = constrain(integrale_dist, -0.5f, 0.5f);
  float I = KI_DIST * integrale_dist;

  float derivee = (erreur - erreurDist_precedente) / dt;
  float D = KD_DIST * derivee;

  erreurDist_precedente = erreur;
  return P + I + D;
}

float calculerPID_Angle(float erreur, float dt) {
  erreur = normaliserAngle(erreur);

  float P = KP_ANGLE * erreur;

  integrale_angle += erreur * dt;
  integrale_angle = constrain(integrale_angle, -0.3f, 0.3f);
  float I = KI_ANGLE * integrale_angle;

  float derivee = normaliserAngle(erreur - erreurAngle_precedente) / dt;
  float D = KD_ANGLE * derivee;

  erreurAngle_precedente = erreur;
  return P + I + D;
}

// ═══════════════════════════════════════════════════════════════
// Flèche Nord (séquence 3) — Gregoire
// ═══════════════════════════════════════════════════════════════
void loadNorthArrowSimplePath() {
  // Flèche Nord simple, sans zigzag.
  // Repère après alignement Nord :
  //   x positif = direction Nord / avant robot, y positif = gauche.
  //
  // Séquence : tige -> pointe -> base gauche -> base droite -> pointe,
  // puis triangles internes pour le remplissage (ET3.4 : >= 80 % colorié).
  NB_POINTS = 0;

  auto addPoint = [](float x, float y) {
    if (NB_POINTS < MAX_POINTS) {
      PARCOURS[NB_POINTS].x = x;
      PARCOURS[NB_POINTS].y = y;
      NB_POINTS++;
    }
  };

  const float shaftEndX = 0.120f;  // 12 cm : tige
  const float tipX      = 0.215f;  // pointe à 21,5 cm
  const float baseTop   = 0.045f;  // coin haut/gauche
  const float baseBot   = 0.029f;  // coin bas/droit raccourci (base plus droite)

  // Départ stylo = (0,0)
  // 1) Contour extérieur.
  addPoint(shaftEndX, 0.000f);      // tige jusqu'au centre de la base
  addPoint(tipX,      0.000f);      // trait central jusqu'à la pointe
  addPoint(shaftEndX, baseTop);     // diagonale vers base gauche/haut
  addPoint(shaftEndX, -baseBot);    // base vers base droite/bas
  addPoint(tipX,      0.000f);      // diagonale retour pointe

  // 2) Remplissage : triangles internes proportionnels, décalés à gauche
  //    pour éviter le débordement à droite.
  const int nbTrianglesInternes = 5;
  const float echelles[nbTrianglesInternes] = {0.82f, 0.66f, 0.52f, 0.38f, 0.25f};

  const float centreX = (tipX + shaftEndX + shaftEndX) / 3.0f - 0.010f;

  for (int i = 0; i < nbTrianglesInternes; i++) {
    float s = echelles[i];

    float baseX_i = centreX + s * (shaftEndX - centreX);
    float tipX_i  = centreX + s * (tipX - centreX) - 0.012f;
    float top_i = baseTop * s * 0.90f;

    // Le premier triangle interne descend plus bas, puis diminue.
    float botBoost = (i == 0) ? 1.35f : ((i == 1) ? 1.20f : 1.05f);
    float bot_i = baseBot * s * botBoost;

    addPoint(baseX_i, 0.000f);
    addPoint(tipX_i,  0.000f);
    addPoint(baseX_i, top_i);
    addPoint(baseX_i, -bot_i);
    addPoint(tipX_i,  0.000f);
  }

  pointActuel = 0;
  objectifAtteint = false;
  parcoursFini = false;
  estModeFlecheNord = true;
  parcoursCharge = true;

  integrale_dist = 0.0f;
  integrale_angle = 0.0f;
  erreurDist_precedente = 0.0f;
  erreurAngle_precedente = 0.0f;
  vG_prev = 0.0f;
  vD_prev = 0.0f;
  temps_precedent = millis();

  etatRobot = "north_arrow_loaded";
  logMsg("Fleche Nord chargee : tige + triangle");
}

void drawNorthArrowFixed() {
  // Étape 0 : on coupe tout mouvement en cours.
  motionMode = MODE_IDLE;
  escalierPending = false;
  seq2Active = false;
  parcoursCharge = false;
  parcoursFini = true;
  objectifAtteint = false;
  stopMoteurs();

  if (!auto_calibrated) {
    etatRobot = "north_arrow_failed";
    logMsg("Magnetometre non calibre : impossible de faire la fleche Nord");
    return;
  }

  logMsg("Fleche Nord : alignement a 0 deg...");

  // Étape 1 : alignement Nord (bloquant, interruptible par Stop).
  faceNorth();
  if (stopRequested) return;

  // Étape 2 : une fois aligné, on redéfinit le repère local :
  // stylo en (0,0), x positif = axe Nord.
  resetParcours();

  // Étape 3 : on charge la flèche : tige -> pointe -> base -> remplissage.
  loadNorthArrowSimplePath();

  etatRobot = "north_arrow_running";
  logMsg("Fleche Nord : dessin tige + triangle lance");
}

// Navigation multi-points au PID (utilisée uniquement par la flèche Nord ;
// l'escalier et le cercle par points de passage du Gregoire ont été supprimés,
// remplacés par les séquences Michka).
void navigationMultiPoints() {
  if (!parcoursCharge) {
    stopMoteurs();
    return;
  }

  if (parcoursFini) {
    stopMoteurs();
    digitalWrite(LEDU1, LOW);
    digitalWrite(LEDU2, HIGH);
    etatRobot = "done";
    return;
  }

  if (objectifAtteint) {
    // On enchaîne les points sans pause ni arrêt moteur : cela évite les
    // cassures entre la tige et la pointe, puis sur la base du triangle.
    if (pointActuel < NB_POINTS - 1) {
      pointActuel++;
      objectifAtteint = false;
      integrale_dist = 0.0f;
      integrale_angle = 0.0f;
      erreurDist_precedente = 0.0f;
      erreurAngle_precedente = 0.0f;
      return;
    }

    stopMoteurs();
    digitalWrite(LEDU1, HIGH);
    digitalWrite(LEDU2, HIGH);

    if (millis() - tempsArrivePoint > PAUSE_ENTRE_POINTS) {
      parcoursFini = true;
      parcoursCharge = false;
      estModeFlecheNord = false;
      etatRobot = "done";
      logMsg("Parcours termine");
    }
    return;
  }

  unsigned long now = millis();
  float dt = (now - temps_precedent) / 1000.0f;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;
  temps_precedent = now;

  double xs, ys;
  getPositionStylo(xs, ys);

  float tx = PARCOURS[pointActuel].x;
  float ty = PARCOURS[pointActuel].y;

  float dx = tx - xs;
  float dy = ty - ys;
  float dist = sqrt(dx * dx + dy * dy);

  // Tolérances spécifiques flèche (segments internes plus permissifs)
  bool segmentInterneTol = pointActuel >= 5;
  bool segmentBaseTol = pointActuel == 3;
  float currentTolerance = segmentBaseTol ? 0.010f
                          : (segmentInterneTol ? 0.009f : 0.006f);

  if (dist < currentTolerance) {
    objectifAtteint = true;
    tempsArrivePoint = now;
    // Pas d'arrêt entre les points : segments mieux enchaînés.
    return;
  }

  digitalWrite(LEDU1, (now / 500) % 2);
  digitalWrite(LEDU2, LOW);

  float angleCible = atan2(dy, dx);
  float errA = normaliserAngle(angleCible - theta_robot);

  // Si le point est derrière le robot, on ne fait pas demi-tour :
  // on garde quasiment le même axe et on y va en marche arrière.
  // Cela évite les grands zigzags sur les diagonales du triangle.
  bool marcheArriere = false;
  bool segmentBaseFleche = (pointActuel == 3);
  bool segmentInterneFleche = (pointActuel >= 5);

  if (fabs(errA) > PI / 2.0f) {
    marcheArriere = true;
    angleCible = normaliserAngle(angleCible + PI);
    errA = normaliserAngle(angleCible - theta_robot);
  }

  // Plus lent et plus stable pour les diagonales et la marche arrière.
  float speedFactor;
  if (dist < 0.03f) speedFactor = 0.32f;
  else if (dist < 0.06f) speedFactor = 0.42f;
  else speedFactor = 0.52f;

  if (fabs(errA) > 0.6f) {
    speedFactor *= 0.65f;
  }

  float cmdD = calculerPID_Distance(dist, dt) * speedFactor;
  if (marcheArriere) {
    cmdD = -cmdD;
  }

  float cmdA = calculerPID_Angle(errA, dt);

  // En marche arrière, la correction de direction doit être inversée.
  // Sur la base du triangle, on corrige davantage (décalage latéral).
  cmdA *= segmentBaseFleche ? 0.55f : (segmentInterneFleche ? 0.42f : 0.50f);
  if (marcheArriere) {
    cmdA = -cmdA;
  }

  if (fabs(cmdD) < VITESSE_MIN && fabs(cmdD) > 0.0f) {
    cmdD = (cmdD > 0) ? VITESSE_MIN : -VITESSE_MIN;
  }

  float maxSpeed = segmentBaseFleche ? 210.0f : (marcheArriere ? 215.0f : 230.0f);

  float vG = constrain(cmdD, -maxSpeed, maxSpeed) - cmdA;
  float vD = constrain(cmdD, -maxSpeed, maxSpeed) + cmdA;

  // Limiteur de variation par itération (boucle cadencée à 50 Hz).
  float MAX_CHANGE = segmentBaseFleche ? 5.0f : (marcheArriere ? 12.0f : 16.0f);
  vG = constrain(vG, vG_prev - MAX_CHANGE, vG_prev + MAX_CHANGE);
  vD = constrain(vD, vD_prev - MAX_CHANGE, vD_prev + MAX_CHANGE);

  vG_prev = vG;
  vD_prev = vD;

  setMoteurs(vG, vD);
  etatRobot = marcheArriere ? "arrow_reverse" : "arrow_forward";

  static unsigned long lastArrowDebug = 0;
  if (millis() - lastArrowDebug > 250) {
    lastArrowDebug = millis();
    Serial.printf("[ARROW] pt=%d/%d reverse=%d base=%d dist=%.3f errA=%.2f cmdD=%.1f cmdA=%.1f vG=%.1f vD=%.1f\n",
                  pointActuel + 1, NB_POINTS, marcheArriere ? 1 : 0, segmentBaseFleche ? 1 : 0,
                  dist, errA, cmdD, cmdA, vG, vD);
  }
}

// ═══════════════════════════════════════════════════════════════
// Protocole texte (TCP / USB) — compatible avec l'interface Tkinter
// ═══════════════════════════════════════════════════════════════
void traiterCommande(const String& cmd) {
  digitalWrite(LEDU2, !digitalRead(LEDU2));

  if (cmd.startsWith("F:")) {
    float cm = cmd.substring(2).toFloat();
    unsigned long ticks = (unsigned long)(cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(+1, +1, ticks);
    logMsg(">> F:" + String(cm) + "cm = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("B:")) {
    float cm = cmd.substring(2).toFloat();
    unsigned long ticks = (unsigned long)(cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
    startMouvement(-1, -1, ticks);
    logMsg(">> B:" + String(cm) + "cm = " + String(ticks) + " ticks");

  } else if (cmd.startsWith("L:")) {
    float deg = cmd.substring(2).toFloat();
    if (imuOk) {
      startTurnGyro(+deg);
      logMsg(">> L:" + String(deg) + "deg (gyro PID)");
    } else {
      float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
      unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(+1, -1, ticks);
      logMsg(">> L:" + String(deg) + "deg = " + String(ticks) + " ticks (encodeur)");
    }

  } else if (cmd.startsWith("R:")) {
    float deg = cmd.substring(2).toFloat();
    if (imuOk) {
      startTurnGyro(-deg);
      logMsg(">> R:" + String(deg) + "deg (gyro PID)");
    } else {
      float arc_cm = (deg / 360.0f) * PI * TRACK_WIDTH_CM * TURN_CORRECTION;
      unsigned long ticks = (unsigned long)(arc_cm / WHEEL_CIRCUM_CM * TICKS_PER_REV);
      startMouvement(-1, +1, ticks);
      logMsg(">> R:" + String(deg) + "deg = " + String(ticks) + " ticks (encodeur)");
    }

  } else if (cmd.startsWith("V:")) {
    // Vitesse roue indépendante, ASSERVIE : V:vG:vD:duree_ms
    String s = cmd.substring(2);
    int c1 = s.indexOf(':');
    int c2 = s.indexOf(':', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      vseq_single.vg     = s.substring(0, c1).toFloat();
      vseq_single.vd     = s.substring(c1 + 1, c2).toFloat();
      vseq_single.dur_ms = s.substring(c2 + 1).toInt();
      escalierPending = false;
      seq2Active = false;
      startVSeq(&vseq_single, 1);
      etatRobot = "vseq";
      logMsg(">> V g=" + String(vseq_single.vg, 2) + " d=" +
             String(vseq_single.vd, 2) + " t=" + String(vseq_single.dur_ms) + "ms");
    } else {
      envoyer("ERR:V attend vG:vD:duree_ms");
    }

  } else if (cmd.startsWith("SEQ:")) {
    // Format attendu : SEQ:num ou SEQ:num:rayon
    String s = cmd.substring(4);
    int idx = s.indexOf(':');
    int n = 0;
    float rayon = 2.0f; // rayon par défaut si non spécifié

    if (idx > 0) {
      n = s.substring(0, idx).toInt();
      rayon = s.substring(idx + 1).toFloat();
    } else {
      n = s.toInt();
    }

    if (n == 1) {
      lancerSeq1();
    } else if (n == 2) {
      lancerSeq2(rayon);
    } else {
      envoyer("ERR:SEQ inconnue:" + String(n));
    }

  } else if (cmd == "S" || cmd == "s") {
    stopTout();

  } else if (cmd == "E" || cmd == "e") {
    long d, g;
    copyTicks(d, g);
    envoyer("Enc G=" + String(g) + " Enc D=" + String(d));

  } else if (cmd == "W" || cmd == "w") {
    // Vitesses mesurées (mises à jour pendant une séquence V uniquement)
    envoyer("Vmes G=" + String(vmes_g_f, 2) + " D=" + String(vmes_d_f, 2) + " cm/s");

  } else if (cmd == "G" || cmd == "g") {
    envoyer("Gyro rate=" + String(lireGyroZ(), 2) + " dps | angle=" +
            String(gyro_angle_deg, 2) + " deg | imu=" + String(imuOk ? 1 : 0));

  } else if (cmd == "M" || cmd == "m") {
    // Lecture magnétomètre (debug)
    int16_t mx, my, mz;
    bool ok = readRawMagnetometer(mx, my, mz);
    envoyer("Mag x=" + String(mx) + " y=" + String(my) + " z=" + String(mz) +
            " | cap=" + String(ok ? headingFromRaw(mx, my) : -1.0f, 1) +
            " deg | calib=" + String(auto_calibrated ? 1 : 0));

  } else if (cmd == "CAL" || cmd == "cal") {
    if (imuOk) {
      calibrerGyro();
      logMsg(">> Biais gyro = " + String(gyroZbias_dps, 3) + " dps");
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
      logMsg(">> PID virage Kp=" + String(KP_TURN, 3) +
             " Ki=" + String(KI_TURN, 3) + " Kd=" + String(KD_TURN, 3));
    } else {
      envoyer("ERR:PIDT attend Kp,Ki,Kd");
    }

  } else if (cmd.startsWith("PIDV:")) {
    // Réglage en direct du PI de vitesse : "PIDV:kp,ki"
    String s = cmd.substring(5);
    int c1 = s.indexOf(',');
    if (c1 > 0) {
      KP_V = s.substring(0, c1).toFloat();
      KI_V = s.substring(c1 + 1).toFloat();
      logMsg(">> PI vitesse Kp=" + String(KP_V, 2) + " Ki=" + String(KI_V, 1));
    } else {
      envoyer("ERR:PIDV attend Kp,Ki");
    }

  } else {
    envoyer("ERR:UNKNOWN:" + cmd);
  }
}

// ═══════════════════════════════════════════════════════════════
// HTTP / API (interface HTML)
// ═══════════════════════════════════════════════════════════════
String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  return s;
}

String mimeType(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css")) return "text/css";
  if (path.endsWith(".js")) return "application/javascript";
  if (path.endsWith(".png")) return "image/png";
  if (path.endsWith(".svg")) return "image/svg+xml";
  return "text/plain";
}

void handleIndex() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain", "index.html absent : pio run -t uploadfs requis");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}

void handleData() {
  double xs, ys;
  getPositionStylo(xs, ys);

  long d, g;
  copyTicks(d, g);

  int16_t mx, my, mz;
  bool magOk = readRawMagnetometer(mx, my, mz);
  float heading = magOk ? headingFromRaw(mx, my) : -1.0f;

  // Avancement : séquence V (escalier/cercle) ou flèche Nord
  int pt = 0, pts = 0;
  if (motionMode == MODE_VSEQ) {
    pt = vseq_idx + 1;
    pts = vseq_len;
  } else if (parcoursCharge || estModeFlecheNord) {
    pt = pointActuel + 1;
    pts = NB_POINTS;
  }

  String staIp = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "";

  String j = "{";
  j += "\"x\":" + String(xs, 4) + ",";
  j += "\"y\":" + String(ys, 4) + ",";
  j += "\"theta\":" + String(theta_robot, 5) + ",";
  j += "\"theta_deg\":" + String(theta_robot * 180.0 / PI, 2) + ",";
  j += "\"ticksD\":" + String(d) + ",";
  j += "\"ticksG\":" + String(g) + ",";
  j += "\"point\":" + String(pt) + ",";
  j += "\"points\":" + String(pts) + ",";
  j += "\"loaded\":" + String((parcoursCharge || motionMode != MODE_IDLE) ? "true" : "false") + ",";
  j += "\"finished\":" + String(parcoursFini ? "true" : "false") + ",";
  j += "\"circle\":" + String(seq2Active ? "true" : "false") + ",";
  j += "\"arrow_mode\":" + String(estModeFlecheNord ? "true" : "false") + ",";
  j += "\"radius\":" + String(rayonCercle, 2) + ",";
  j += "\"status\":\"" + jsonEscape(etatRobot) + "\",";
  j += "\"message\":\"" + jsonEscape(lastMessage) + "\",";
  j += "\"sta_ip\":\"" + staIp + "\",";
  j += "\"mag_x\":" + String(mx) + ",";
  j += "\"mag_y\":" + String(my) + ",";
  j += "\"mag_z\":" + String(mz) + ",";
  j += "\"heading\":" + String(heading, 2) + ",";
  j += "\"mag_calibrated\":" + String(auto_calibrated ? "true" : "false") + ",";
  j += "\"mag_offset_x\":" + String(mag_offset_x, 2) + ",";
  j += "\"mag_offset_y\":" + String(mag_offset_y, 2) + ",";
  j += "\"north_ref\":" + String(magnetic_north_angle, 2);
  j += "}";
  server.send(200, "application/json", j);
}

void handleCommand() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "cmd manquant");
    return;
  }

  String cmd = server.arg("cmd");

  if (cmd == "seq1") {
    lancerSeq1();
  } else if (cmd == "seq2") {
    float r = server.hasArg("radius") ? server.arg("radius").toFloat() : 13.0f;
    lancerSeq2(r);
  } else if (cmd == "arrow") {
    // Réponse envoyée AVANT l'action bloquante (alignement Nord + dessin)
    server.send(200, "application/json", "{\"ok\":true,\"started\":\"arrow\"}");
    drawNorthArrowFixed();
    return;
  } else if (cmd == "reset") {
    resetParcours();
    parcoursCharge = false;
    seq2Active = false;
    estModeFlecheNord = false;
  } else if (cmd == "stop") {
    stopTout();
  } else if (cmd == "mag_calib") {
    server.send(200, "application/json", "{\"ok\":true,\"started\":\"mag_calib\"}");
    calibrateMagnetometer();
    return;
  } else if (cmd == "face_north") {
    server.send(200, "application/json", "{\"ok\":true,\"started\":\"face_north\"}");
    faceNorth();
    return;
  } else if (cmd == "set_north") {
    setCurrentAsNorth();
  } else {
    server.send(400, "text/plain", "commande inconnue");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleManual() {
  String dir = server.hasArg("dir") ? server.arg("dir") : "stop";
  int speed = server.hasArg("speed") ? server.arg("speed").toInt() : 220;
  speed = constrain(speed, 0, 255);

  // Le mode manuel coupe toute séquence en cours.
  motionMode = MODE_IDLE;
  escalierPending = false;
  seq2Active = false;
  parcoursCharge = false;
  parcoursFini = true;
  estModeFlecheNord = false;

  if (dir == "f") {
    setMoteurs(speed, speed);
  } else if (dir == "b") {
    setMoteurs(-speed, -speed);
  } else if (dir == "l") {
    setMoteurs(-speed, speed);
  } else if (dir == "r") {
    setMoteurs(speed, -speed);
  } else {
    stopMoteurs();
  }

  etatRobot = "manual";
  server.send(200, "application/json", "{\"ok\":true}");
}

// Lecture (et réglage optionnel via paramètres GET) de tous les gains.
// Le panneau PID du HTML ne fait que LIRE ces valeurs.
void handlePID() {
  if (server.hasArg("kp_dist"))  KP_DIST  = server.arg("kp_dist").toFloat();
  if (server.hasArg("ki_dist"))  KI_DIST  = server.arg("ki_dist").toFloat();
  if (server.hasArg("kd_dist"))  KD_DIST  = server.arg("kd_dist").toFloat();
  if (server.hasArg("kp_angle")) KP_ANGLE = server.arg("kp_angle").toFloat();
  if (server.hasArg("ki_angle")) KI_ANGLE = server.arg("ki_angle").toFloat();
  if (server.hasArg("kd_angle")) KD_ANGLE = server.arg("kd_angle").toFloat();
  if (server.hasArg("kp_turn"))  KP_TURN  = server.arg("kp_turn").toFloat();
  if (server.hasArg("ki_turn"))  KI_TURN  = server.arg("ki_turn").toFloat();
  if (server.hasArg("kd_turn"))  KD_TURN  = server.arg("kd_turn").toFloat();
  if (server.hasArg("kp_v"))     KP_V     = server.arg("kp_v").toFloat();
  if (server.hasArg("ki_v"))     KI_V     = server.arg("ki_v").toFloat();

  String j = "{";
  j += "\"kp_turn\":" + String(KP_TURN, 3) + ",";
  j += "\"ki_turn\":" + String(KI_TURN, 3) + ",";
  j += "\"kd_turn\":" + String(KD_TURN, 3) + ",";
  j += "\"kp_v\":" + String(KP_V, 2) + ",";
  j += "\"ki_v\":" + String(KI_V, 1) + ",";
  j += "\"kp_dist\":" + String(KP_DIST, 3) + ",";
  j += "\"ki_dist\":" + String(KI_DIST, 3) + ",";
  j += "\"kd_dist\":" + String(KD_DIST, 3) + ",";
  j += "\"kp_angle\":" + String(KP_ANGLE, 3) + ",";
  j += "\"ki_angle\":" + String(KI_ANGLE, 3) + ",";
  j += "\"kd_angle\":" + String(KD_ANGLE, 3);
  j += "}";
  server.send(200, "application/json", j);
}

// ═══════════════════════════════════════════════════════════════
// Entrées console : TCP (Tkinter) et USB (debug)
// ═══════════════════════════════════════════════════════════════
void gererTcp() {
  // Acceptation d'un client TCP (un seul à la fois)
  if (!tcpClient || !tcpClient.connected()) {
    tcpClient = tcpServer.accept();
    if (tcpClient) Serial.println("Client TCP connecte : " + tcpClient.remoteIP().toString());
  }
  // Commandes via WiFi (protocole texte Tkinter)
  if (tcpClient && tcpClient.connected() && tcpClient.available()) {
    String cmd = tcpClient.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) traiterCommande(cmd);
  }
}

void gererSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) traiterCommande(cmd);
  }
}

// ═══════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // piles faibles : pas de reset brown-out
  Serial.begin(115200);
  delay(200);

  // --- GPIO / PWM / encodeurs (schéma Michka) ---
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

  // --- I2C : un seul begin pour l'IMU ET le magnétomètre ---
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // --- EEPROM + magnétomètre ---
  EEPROM.begin(EEPROM_SIZE);
  initMagnetometer();
  if (loadMagCalibration()) {
    Serial.println("Calibration magnetometre chargee depuis l'EEPROM");
  } else {
    Serial.println("Pas de calibration magnetometre en EEPROM");
  }

  // --- IMU (gyroscope) ---
  imuOk = imuInit();
  if (imuOk) {
    Serial.println("IMU LSM6DS3 OK - calibration du gyro (NE PAS BOUGER le robot)...");
    calibrerGyro();
    Serial.println("Biais gyro = " + String(gyroZbias_dps, 3) + " dps");
  } else {
    Serial.println("IMU non detectee - virages en repli ENCODEUR.");
  }

  // --- LittleFS (sert l'interface HTML) ---
  if (!LittleFS.begin(true)) {
    Serial.println("Erreur LittleFS : index.html indisponible (pio run -t uploadfs ?)");
  }

  // --- WiFi : AP + STA simultanés ---
  // AP  : toujours dispo (soutenance), reseau "Drawbot" -> http://192.168.4.1
  // STA : rejoint le reseau local en arriere-plan, sans bloquer le demarrage
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  if (strlen(STA_PASS) > 0) WiFi.begin(STA_SSID, STA_PASS);
  else                      WiFi.begin(STA_SSID);
  Serial.println("AP \"" + String(AP_SSID) + "\" -> http://" + WiFi.softAPIP().toString());
  Serial.println("Connexion a \"" + String(STA_SSID) + "\" en arriere-plan...");

  // --- Serveur HTTP (interface HTML) ---
  server.on("/", handleIndex);
  server.on("/data", handleData);
  server.on("/cmd", handleCommand);
  server.on("/manual", handleManual);
  server.on("/pid", handlePID);
  server.onNotFound([]() {
    String path = server.uri();
    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, "r");
      server.streamFile(f, mimeType(path));
      f.close();
    } else {
      server.send(404, "text/plain", "404");
    }
  });
  server.begin();

  // --- Serveur TCP (interface Tkinter) ---
  tcpServer.begin();

  stopMoteurs();
  resetParcours();
  etatRobot = "idle";
  lastMessage = "Pret";
  digitalWrite(LEDU1, HIGH);
  Serial.println("Drawbot pret : HTTP :80 (HTML) + TCP :8266 (Tkinter)");
}

// ═══════════════════════════════════════════════════════════════
// Boucle principale
// ═══════════════════════════════════════════════════════════════
void loop() {
  // Interfaces : HTML (HTTP), Tkinter (TCP), debug (USB)
  server.handleClient();
  gererTcp();
  gererSerial();

  // Odométrie en continu : la carte du HTML trace TOUS les mouvements,
  // y compris l'escalier et le cercle (séquences Michka).
  mettreAJourOdometrie();

  // Boucle de contrôle Michka, cadencée à 100 Hz
  static unsigned long dernierControle = 0;
  if (motionMode != MODE_IDLE && millis() - dernierControle >= CONTROL_PERIOD_MS) {
    dernierControle = millis();
    if (motionMode == MODE_TICKS)      controlTicks();
    else if (motionMode == MODE_TURN)  controlTurnGyro();
    else if (motionMode == MODE_VSEQ)  controlVSeq();
  }

  // Navigation flèche Nord (PID dist + angle), cadencée à 50 Hz comme
  // dans le projet d'origine (le dt et les limiteurs en dépendent).
  static unsigned long derniereNav = 0;
  if (parcoursCharge && motionMode == MODE_IDLE && millis() - derniereNav >= 20) {
    derniereNav = millis();
    navigationMultiPoints();
  }

  // Annonce de l'IP locale quand la connexion STA aboutit
  static bool staAnnonce = false;
  if (WiFi.status() == WL_CONNECTED && !staAnnonce) {
    staAnnonce = true;
    logMsg("WiFi local connecte : http://" + WiFi.localIP().toString());
  } else if (WiFi.status() != WL_CONNECTED && staAnnonce) {
    staAnnonce = false;
  }
}
