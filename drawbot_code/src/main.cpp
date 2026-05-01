#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ── I2C / Capteurs ───────────────────────────────────────────────────────────
#define LSM6DS3_ADDR      0x6B
#define LSM6DS3_WHO_AM_I  0x0F
#define LSM6DS3_CTRL1_XL  0x10   // accel: ODR=104Hz, ±2g
#define LSM6DS3_CTRL2_G   0x11   // gyro:  ODR=104Hz, ±250°/s
#define LSM6DS3_OUTX_L_G  0x22   // gyro  X low byte (6 bytes consécutifs)
#define LSM6DS3_OUTX_L_XL 0x28   // accel X low byte (6 bytes consécutifs)

#define LIS3MDL_ADDR      0x1E
#define LIS3MDL_CTRL1     0x20   // OM=ultra-high, ODR=10Hz → 0x70
#define LIS3MDL_CTRL2     0x21   // FS=±4 gauss → 0x00
#define LIS3MDL_CTRL3     0x22   // mode continu → 0x00
#define LIS3MDL_CTRL4     0x23   // OMZ=ultra-high → 0x0C
#define LIS3MDL_OUT_X_L   0xA8   // 0x28|0x80 : auto-incrément sur 6 bytes

static bool imu_ok = false;
static bool mag_ok = false;

static void i2c_write_reg(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool i2c_read_regs(uint8_t dev, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  Wire.requestFrom(dev, n);
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

void init_sensors() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t who = 0;
  i2c_read_regs(LSM6DS3_ADDR, LSM6DS3_WHO_AM_I, &who, 1);
  if (who == 0x69) {
    i2c_write_reg(LSM6DS3_ADDR, LSM6DS3_CTRL1_XL, 0x40);  // ODR=104Hz, ±2g
    i2c_write_reg(LSM6DS3_ADDR, LSM6DS3_CTRL2_G,  0x40);  // ODR=104Hz, ±250°/s
    imu_ok = true;
    Serial.println("[I2C] LSM6DS3 OK");
  } else {
    Serial.printf("[I2C] LSM6DS3 WHO_AM_I=0x%02X (attendu 0x69)\n", who);
  }

  i2c_write_reg(LIS3MDL_ADDR, LIS3MDL_CTRL1, 0x70);  // OM=ultra-high, ODR=10Hz
  i2c_write_reg(LIS3MDL_ADDR, LIS3MDL_CTRL2, 0x00);  // FS=±4 gauss
  i2c_write_reg(LIS3MDL_ADDR, LIS3MDL_CTRL3, 0x00);  // mode continu
  i2c_write_reg(LIS3MDL_ADDR, LIS3MDL_CTRL4, 0x0C);  // OMZ=ultra-high
  mag_ok = true;
  Serial.println("[I2C] LIS3MDL OK");
}

// ── Bluetooth classique (SPP) ───────────────────────────────────────────────
const char* BT_DEVICE_NAME = "Drawbot";
const int MAX_DRIVE_SPEED = 255;
// ── PD position ──────────────────────────────────────────────────────────────
const float PD_KP         = 0.15f;   // gain proportionnel  → à tuner
const float PD_KD         = 0.012f;  // gain dérivé         → à tuner
const int   PD_MIN_SPEED  = 70;      // PWM minimum (en dessous les moteurs calent)
const int   PD_MAX_SPEED  = 200;     // PWM maximum
const int   PD_BRAKE_ANTICIPATION = 10; // ticks avant la cible pour compenser le glissement

BluetoothSerial SerialBT;
bool bt_client_connected = false;

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
#define PWM_FREQ   1000
#define PWM_RES    8
#define CH_IN1_D   0
#define CH_IN2_D   1
#define CH_IN1_G   2
#define CH_IN2_G   3

// ── Encodeurs ────────────────────────────────────────────────────────────────
#define TICKS_PER_WHEEL_REV    2008
#define WHEEL_DIAMETER_MM      90.0f
#define WHEEL_CIRCUMFERENCE_CM (PI * WHEEL_DIAMETER_MM / 10.0f)  // ≈ 28.27 cm
#define MOVE_DEFAULT_SPEED     150
#define PEN_OFFSET_CM          13.0f   // distance stylo → centre essieu
#define TRACK_WIDTH_CM          8.1f   // écartement centre roue gauche → centre roue droite
#define CORNER_SPIN_TICKS ((unsigned long)((TRACK_WIDTH_CM / 2.0f * PI / 2.0f) / WHEEL_CIRCUMFERENCE_CM * TICKS_PER_WHEEL_REV))
#define CORNER_ADVANCE_TICKS  ((unsigned long)(PEN_OFFSET_CM / WHEEL_CIRCUMFERENCE_CM * TICKS_PER_WHEEL_REV))

volatile long enc_D = 0;
volatile long enc_G = 0;
int current_speed_D = 0;
int current_speed_G = 0;
bool timed_motion_active = false;
unsigned long timed_motion_deadline_ms = 0;
bool tick_motion_active = false;
unsigned long tick_motion_target = 0;
int tick_motion_speed_D = 0;
int tick_motion_speed_G = 0;

// État PD
long pd_prev_error = 0;
unsigned long pd_prev_ms = 0;
int pd_current_max_speed = 200;

// État corner (angle droit parfait par combinaison rotation+translation)
bool  corner_active    = false;
int   corner_dir       = 1;      // +1 gauche (CCW), -1 droite (CW)
float corner_theta     = 0.0f;   // angle parcouru en radians (0 → PI/2)
int   corner_pwm       = MOVE_DEFAULT_SPEED;
long  corner_prev_enc_D = 0;
long  corner_prev_enc_G = 0;

void IRAM_ATTR isr_enc_D() {
  enc_D += (digitalRead(ENC_D_B) == digitalRead(ENC_D_A)) ? -1 : 1;
}

void IRAM_ATTR isr_enc_G() {
  enc_G += (digitalRead(ENC_G_B) == digitalRead(ENC_G_A)) ? 1 : -1;
}

// ── Fonctions moteurs ────────────────────────────────────────────────────────
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

int clamp_drive_speed(int speed) {
  return constrain(speed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
}

void apply_motor_D(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, speed);
  } else if (speed < 0) {
    ledcWrite(CH_IN1_D, -speed);
    ledcWrite(CH_IN2_D, 0);
  } else {
    ledcWrite(CH_IN1_D, 0);
    ledcWrite(CH_IN2_D, 0);
  }
}

void apply_motor_G(int speed) {
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

void apply_motor_outputs() {
  apply_motor_D(current_speed_D);
  apply_motor_G(current_speed_G);
}

void cancel_autonomous_motion() {
  timed_motion_active = false;
  timed_motion_deadline_ms = 0;
  tick_motion_active = false;
  tick_motion_target = 0;
}

void set_target_speeds(int speed_D, int speed_G) {
  current_speed_D = clamp_drive_speed(speed_D);
  current_speed_G = clamp_drive_speed(speed_G);
  apply_motor_outputs();
}

void start_timed_motion(int speed_D, int speed_G, unsigned long duration_ms) {
  cancel_autonomous_motion();
  set_target_speeds(speed_D, speed_G);
  timed_motion_active = true;
  timed_motion_deadline_ms = millis() + duration_ms;
}

void start_tick_motion(int speed_D, int speed_G, unsigned long target_ticks) {
  cancel_autonomous_motion();
  noInterrupts();
  enc_D = 0;
  enc_G = 0;
  interrupts();
  tick_motion_speed_D = clamp_drive_speed(speed_D);
  tick_motion_speed_G = clamp_drive_speed(speed_G);
  pd_current_max_speed = max(abs(tick_motion_speed_D), abs(tick_motion_speed_G));
  if (pd_current_max_speed < PD_MIN_SPEED) pd_current_max_speed = PD_MIN_SPEED;
  set_target_speeds(speed_D, speed_G);
  tick_motion_active = true;
  tick_motion_target = target_ticks;
  pd_prev_error = (long)target_ticks;
  pd_prev_ms    = millis();
}

void brake_motors() {
  // Frein actif : les deux IN à HIGH = court-circuit moteur → arrêt net
  ledcWrite(CH_IN1_D, 255);
  ledcWrite(CH_IN2_D, 255);
  ledcWrite(CH_IN1_G, 255);
  ledcWrite(CH_IN2_G, 255);
  delay(80);
  // Relâche le frein
  ledcWrite(CH_IN1_D, 0);
  ledcWrite(CH_IN2_D, 0);
  ledcWrite(CH_IN1_G, 0);
  ledcWrite(CH_IN2_G, 0);
}

void force_stop_motors() {
  cancel_autonomous_motion();
  brake_motors();
  current_speed_D = 0;
  current_speed_G = 0;
}

void stop_motors() {
  force_stop_motors();
}

void read_encoder_counts(long& right_ticks, long& left_ticks) {
  noInterrupts();
  right_ticks = enc_D;
  left_ticks = enc_G;
  interrupts();
}

int signed_speed_limit(int requested_speed, int limit) {
  if (requested_speed > 0) {
    return min(requested_speed, limit);
  }
  if (requested_speed < 0) {
    return max(requested_speed, -limit);
  }
  return 0;
}

void update_tick_motion_pd(unsigned long travelled_ticks) {
  long error = (long)tick_motion_target - (long)travelled_ticks;

  unsigned long now_ms = millis();
  unsigned long dt_ms  = now_ms - pd_prev_ms;
  if (dt_ms == 0) dt_ms = 1;
  pd_prev_ms = now_ms;

  // Dérivée de l'erreur (ticks/ms)
  float d_error = (float)(error - pd_prev_error) / (float)dt_ms;
  pd_prev_error = error;

  // Sortie PD
  float output = PD_KP * (float)error + PD_KD * d_error * 1000.0f;

  // Magnitude clampée entre min et max
  int magnitude = (int)constrain(fabsf(output), (float)PD_MIN_SPEED, (float)pd_current_max_speed);

  // Préserver le ratio D/G (utile pour les virages)
  int max_orig = max(abs(tick_motion_speed_D), abs(tick_motion_speed_G));
  int new_D = (max_orig > 0) ? (int)((float)magnitude * tick_motion_speed_D / max_orig) : 0;
  int new_G = (max_orig > 0) ? (int)((float)magnitude * tick_motion_speed_G / max_orig) : 0;

  if (new_D != current_speed_D || new_G != current_speed_G) {
    set_target_speeds(new_D, new_G);
  }
}

void start_corner(int dir, int speed) {
  cancel_autonomous_motion();
  corner_active = true;
  corner_dir    = dir;
  corner_theta  = 0.0f;
  corner_pwm    = speed;
  noInterrupts();
  corner_prev_enc_D = enc_D;
  corner_prev_enc_G = enc_G;
  interrupts();
  // Initialise les signes pour le premier tick (theta=0)
  float ratio = TRACK_WIDTH_CM / (2.0f * PEN_OFFSET_CM);
  current_speed_D = (int)(speed * ratio) * dir;   // positif pour gauche
  current_speed_G = -(int)(speed * ratio) * dir;  // négatif pour gauche
  apply_motor_outputs();
}

void update_corner_motion() {
  long enc_r, enc_l;
  read_encoder_counts(enc_r, enc_l);

  long delta_r = enc_r - corner_prev_enc_D;
  long delta_l = enc_l - corner_prev_enc_G;
  corner_prev_enc_D = enc_r;
  corner_prev_enc_G = enc_l;

  // Signe physique = signe de la vitesse commandée (indépendant du câblage encodeur)
  float signed_dr = (current_speed_D >= 0 ? 1.0f : -1.0f) * fabsf((float)delta_r);
  float signed_dl = (current_speed_G >= 0 ? 1.0f : -1.0f) * fabsf((float)delta_l);

  float dr_cm = signed_dr * WHEEL_CIRCUMFERENCE_CM / TICKS_PER_WHEEL_REV;
  float dl_cm = signed_dl * WHEEL_CIRCUMFERENCE_CM / TICKS_PER_WHEEL_REV;

  float dtheta = (dr_cm - dl_cm) / TRACK_WIDTH_CM;
  corner_theta += dtheta;
  if (corner_theta < 0.0f) corner_theta = 0.0f;

  if (corner_theta >= PI / 2.0f - 0.03f) {
    // 90° atteint : on repart tout droit
    set_target_speeds(corner_pwm, corner_pwm);
    corner_active = false;
    return;  // la réponse sera envoyée par l'appelant
  }

  // v_R = V·(sin θ + (W/2D)·cos θ)
  // v_L = V·(sin θ − (W/2D)·cos θ)
  float ratio = TRACK_WIDTH_CM / (2.0f * PEN_OFFSET_CM);
  float s = sinf(corner_theta);
  float c = cosf(corner_theta);
  int vr = constrain((int)(corner_pwm * (s + (float)corner_dir * ratio * c)), -255, 255);
  int vl = constrain((int)(corner_pwm * (s - (float)corner_dir * ratio * c)), -255, 255);

  // Appliquer le seuil minimum moteur (en préservant le signe)
  if (vr > 0 && vr < PD_MIN_SPEED)  vr =  PD_MIN_SPEED;
  if (vr < 0 && vr > -PD_MIN_SPEED) vr = -PD_MIN_SPEED;
  if (vl > 0 && vl < PD_MIN_SPEED)  vl =  PD_MIN_SPEED;
  if (vl < 0 && vl > -PD_MIN_SPEED) vl = -PD_MIN_SPEED;

  current_speed_D = vr;
  current_speed_G = vl;
  apply_motor_outputs();
}

void send_response(const String& response) {
  if (SerialBT.hasClient()) {
    SerialBT.println(response);
  }
}

bool parse_motion_command(const String& cmd, int& speed_D, int& speed_G, unsigned long& third_value) {
  int first_sep = cmd.indexOf(':');
  int second_sep = cmd.indexOf(':', first_sep + 1);
  int third_sep = cmd.indexOf(':', second_sep + 1);

  if (first_sep < 0 || second_sep < 0 || third_sep < 0) {
    return false;
  }

  String speed_D_str = cmd.substring(first_sep + 1, second_sep);
  String speed_G_str = cmd.substring(second_sep + 1, third_sep);
  String duration_str = cmd.substring(third_sep + 1);

  if (speed_D_str.length() == 0 || speed_G_str.length() == 0 || duration_str.length() == 0) {
    return false;
  }

  speed_D = clamp_drive_speed(speed_D_str.toInt());
  speed_G = clamp_drive_speed(speed_G_str.toInt());
  long parsed_value = duration_str.toInt();
  if (parsed_value < 0) {
    parsed_value = 0;
  }
  third_value = (unsigned long) parsed_value;
  return true;
}

// ── Traitement des commandes ─────────────────────────────────────────────────
void handle_command(const String& cmd) {
  if (cmd.startsWith("F:")) {
    int speed = clamp_drive_speed(cmd.substring(2).toInt());
    cancel_autonomous_motion();
    set_target_speeds(speed, speed);
    send_response("OK:FORWARD:" + String(speed));

  } else if (cmd.startsWith("B:")) {
    int speed = clamp_drive_speed(cmd.substring(2).toInt());
    cancel_autonomous_motion();
    set_target_speeds(-speed, -speed);
    send_response("OK:BACKWARD:" + String(speed));

  } else if (cmd.startsWith("L:")) {
    int speed = clamp_drive_speed(cmd.substring(2).toInt());
    cancel_autonomous_motion();
    set_target_speeds(speed, -speed);
    send_response("OK:LEFT:" + String(speed));

  } else if (cmd.startsWith("R:")) {
    int speed = clamp_drive_speed(cmd.substring(2).toInt());
    cancel_autonomous_motion();
    set_target_speeds(-speed, speed);
    send_response("OK:RIGHT:" + String(speed));

  } else if (cmd.startsWith("RUN_MS:")) {
    int speed_D = 0;
    int speed_G = 0;
    unsigned long duration_ms = 0;
    if (!parse_motion_command(cmd, speed_D, speed_G, duration_ms)) {
      send_response("ERR:BAD_RUN_MS");
      return;
    }

    if (duration_ms == 0) {
      stop_motors();
      send_response("OK:RUN_MS:0");
      return;
    }

    start_timed_motion(speed_D, speed_G, duration_ms);
    send_response("OK:RUN_MS:" + String(speed_D) + ":" + String(speed_G) + ":" + String(duration_ms));

  } else if (cmd.startsWith("RUN_TICKS:")) {
    int speed_D = 0;
    int speed_G = 0;
    unsigned long target_ticks = 0;
    if (!parse_motion_command(cmd, speed_D, speed_G, target_ticks)) {
      send_response("ERR:BAD_RUN_TICKS");
      return;
    }

    if (target_ticks == 0) {
      stop_motors();
      send_response("OK:RUN_TICKS:0");
      return;
    }

    start_tick_motion(speed_D, speed_G, target_ticks);
    send_response("OK:RUN_TICKS:" + String(speed_D) + ":" + String(speed_G) + ":" + String(target_ticks));

  } else if (cmd == "TEST_REV") {
    start_tick_motion(MOVE_DEFAULT_SPEED, MOVE_DEFAULT_SPEED, TICKS_PER_WHEEL_REV);
    send_response("OK:TEST_REV:TICKS:" + String(TICKS_PER_WHEEL_REV));

  } else if (cmd.startsWith("MOVE:")) {
    String rest = cmd.substring(5);
    int sep = rest.indexOf(':');
    float distance_cm = rest.substring(0, sep < 0 ? rest.length() : sep).toFloat();
    int speed = sep >= 0 ? clamp_drive_speed(rest.substring(sep + 1).toInt()) : MOVE_DEFAULT_SPEED;
    if (speed == 0) speed = MOVE_DEFAULT_SPEED;
    unsigned long ticks = (unsigned long) (fabsf(distance_cm) / WHEEL_CIRCUMFERENCE_CM * TICKS_PER_WHEEL_REV);
    int dir = distance_cm >= 0 ? 1 : -1;
    start_tick_motion(dir * speed, dir * speed, ticks);
    send_response("OK:MOVE:" + String(distance_cm) + ":TICKS:" + String(ticks));

  } else if (cmd == "S") {
    stop_motors();
    send_response("OK:STOP");

  } else if (cmd == "ENC") {
    long right_ticks = 0;
    long left_ticks = 0;
    read_encoder_counts(right_ticks, left_ticks);
    send_response("ENC:" + String(right_ticks) + "," + String(left_ticks));

  } else if (cmd == "RESET_ENC") {
    stop_motors();
    noInterrupts();
    enc_D = 0;
    enc_G = 0;
    interrupts();
    send_response("OK:RESET_ENC");

  } else if (cmd == "PING") {
    send_response("PONG");

  } else if (cmd == "IMU") {
    if (!imu_ok) { send_response("ERR:IMU_NOT_INIT"); return; }
    uint8_t buf[6];
    if (!i2c_read_regs(LSM6DS3_ADDR, LSM6DS3_OUTX_L_G, buf, 6)) {
      send_response("ERR:IMU_READ"); return;
    }
    int16_t gx = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t gy = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t gz = (int16_t)(buf[5] << 8 | buf[4]);
    if (!i2c_read_regs(LSM6DS3_ADDR, LSM6DS3_OUTX_L_XL, buf, 6)) {
      send_response("ERR:IMU_READ"); return;
    }
    int16_t ax = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t ay = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t az = (int16_t)(buf[5] << 8 | buf[4]);
    send_response("IMU:" + String(ax) + "," + String(ay) + "," + String(az) +
                  "," + String(gx) + "," + String(gy) + "," + String(gz));

  } else if (cmd == "MAG") {
    if (!mag_ok) { send_response("ERR:MAG_NOT_INIT"); return; }
    uint8_t buf[6];
    if (!i2c_read_regs(LIS3MDL_ADDR, LIS3MDL_OUT_X_L, buf, 6)) {
      send_response("ERR:MAG_READ"); return;
    }
    int16_t mx = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t my = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t mz = (int16_t)(buf[5] << 8 | buf[4]);
    send_response("MAG:" + String(mx) + "," + String(my) + "," + String(mz));

  } else if (cmd.startsWith("CORNER_LEFT") || cmd.startsWith("CORNER_RIGHT")) {
    // Angle droit parfait par combinaison rotation + translation
    // Appeler quand le stylo est exactement au coin
    bool left = cmd.startsWith("CORNER_LEFT");
    int sep = cmd.indexOf(':');
    int speed = (sep >= 0) ? cmd.substring(sep + 1).toInt() : MOVE_DEFAULT_SPEED;
    if (speed <= 0) speed = MOVE_DEFAULT_SPEED;
    start_corner(left ? 1 : -1, speed);
    send_response(left ? "OK:CORNER_LEFT:STARTED" : "OK:CORNER_RIGHT:STARTED");

  } else {
    send_response("ERR:UNKNOWN:" + cmd);
  }
}

void update_bluetooth_state() {
  bool has_client = SerialBT.hasClient();

  if (has_client && !bt_client_connected) {
    bt_client_connected = true;
    Serial.println("Client Bluetooth connecte");
    stop_motors();
  } else if (!has_client && bt_client_connected) {
    bt_client_connected = false;
    Serial.println("Client Bluetooth deconnecte");
    stop_motors();
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.println("\n=== Drawbot ===");

  motor_setup();
  stop_motors();

  pinMode(ENC_D_A, INPUT_PULLUP);
  pinMode(ENC_D_B, INPUT_PULLUP);
  pinMode(ENC_G_A, INPUT_PULLUP);
  pinMode(ENC_G_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), isr_enc_D, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_G_A), isr_enc_G, CHANGE);

  init_sensors();

  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("Erreur: initialisation Bluetooth impossible");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Bluetooth SPP pret");
  Serial.println("Nom de l'appareil : " + String(BT_DEVICE_NAME));
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  update_bluetooth_state();

  if (timed_motion_active && (long) (millis() - timed_motion_deadline_ms) >= 0) {
    stop_motors();
    send_response("OK:RUN_MS_DONE");
  }

  if (tick_motion_active) {
    long right_ticks = 0;
    long left_ticks = 0;
    read_encoder_counts(right_ticks, left_ticks);
    unsigned long average_ticks = ((unsigned long) abs(right_ticks) + (unsigned long) abs(left_ticks)) / 2;
    update_tick_motion_pd(average_ticks);
    unsigned long effective_target = (tick_motion_target > (unsigned long)PD_BRAKE_ANTICIPATION)
                                      ? tick_motion_target - PD_BRAKE_ANTICIPATION
                                      : tick_motion_target;
    if (average_ticks >= effective_target) {
      stop_motors();
      send_response("OK:RUN_TICKS_DONE");
    }
  }

  if (corner_active) {
    update_corner_motion();
    if (!corner_active) {
      send_response("OK:CORNER_DONE");
    }
  }

  if (bt_client_connected && SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.println("CMD: " + cmd);
      handle_command(cmd);
    }
  }
}
