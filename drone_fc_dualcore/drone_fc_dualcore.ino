// ═══════════════════════════════════════════════════════════════
//  DRONE FC — RP2040 AMP DUAL-CORE  (V10 → V11)
//
//  CORE 0  →  setup() / loop()
//             MPU6050 okuma, PID hesap, ESC PWM, iniş matematiği
//             *** Serial / SerialBT YASAK ***
//
//  CORE 1  →  setup1() / loop1()
//             Komut alma (USB + BT), telemetri gönderme
//             Ring-buffer non-blocking TX
//
//  Çekirdekler arası paylaşım: volatile değişkenler + bayraklar
//  Mutex / lock YOK — snapshot mantığı
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <math.h>
#include "Adafruit_VL53L0X.h"

#define MPUWire Wire
#define SerialBT Serial2

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ─────────────────────────────────────────────────────────────
//  FSM
// ─────────────────────────────────────────────────────────────
enum DroneState { IDLE_CHARGING, PRE_FLIGHT, TAKEOFF, PATROL, HANDOVER, RTL, LANDING };
volatile DroneState systemState = IDLE_CHARGING;

// ─────────────────────────────────────────────────────────────
//  DONANIM
// ─────────────────────────────────────────────────────────────
int escPinleri[] = { 10, 11, 12, 13 };

// ─────────────────────────────────────────────────────────────
//  ÇEKİRDEKLER ARASI PAYLAŞILAN VOLATİLE DEĞİŞKENLER
//  (Core 0 yazar, Core 1 okur — telemetri snapshot)
// ─────────────────────────────────────────────────────────────
volatile float  v_kusursuz_roll  = 0;
volatile float  v_kusursuz_pitch = 0;
volatile float  v_yaw_rate_dps   = 0;
volatile float  v_current_z      = 0;
volatile float  v_alt_error      = 0;
volatile float  v_pid_output_z   = 0;
volatile float  v_rate_error_roll  = 0;
volatile float  v_rate_error_pitch = 0;

// Core 1 tarafından okunur/yazılır, Core 0 okur
volatile int    v_bazGaz         = 900;
volatile float  v_hedef_z        = 1.2f;
volatile bool   v_motorlarAktif  = false;
volatile bool   v_altPidAktif    = false;
volatile bool   v_lazerAktif     = false;
volatile bool   v_lidarGecerli   = false;
volatile float  v_inis_hedef_z   = 0;
volatile bool   v_inis_aktif     = false;

// PID katsayıları — Core 1 yazar, Core 0 okur
volatile float  v_Kp_angle = 3.0f;
volatile float  v_Kp_rate  = 0.5f;
volatile float  v_Ki_rate  = 0.0f;
volatile float  v_Kd_rate  = 0.05f;
volatile float  v_Kp_alt   = 80.0f;
volatile float  v_Ki_alt   = 0.0f;
volatile float  v_Kd_alt   = 40.0f;

// ─────────────────────────────────────────────────────────────
//  OLAY BAYRAKLARI (Core 0 set eder, Core 1 temizler)
// ─────────────────────────────────────────────────────────────
volatile bool flag_inis_tamamlandi = false;
volatile bool flag_lidar_devralıs  = false;
volatile bool flag_disarmed        = false;
volatile bool flag_armed           = false;
volatile bool flag_kalibrasyon_ok  = false;

// ─────────────────────────────────────────────────────────────
//  CORE 0 — YEREL (sadece uçuş çekirdeği kullanır)
// ─────────────────────────────────────────────────────────────
unsigned long oncekiZaman = 0;
float dt = 0;

float current_z = 0.0f;
float hedef_z   = 1.2f;
bool  lazerAktif = false;

int bazGaz         = 900;
int min_ucus_gazi  = 1150;

const int MPU_ADRES = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float gyro_X_cal = 0, gyro_Y_cal = 0, gyro_Z_cal = 0;
float angle_roll = 0, angle_pitch = 0;
unsigned long loop_timer;

float filtrelenmis_GyX = 0, filtrelenmis_GyY = 0, filtrelenmis_GyZ = 0;
float filt_AcX = 0, filt_AcY = 0, filt_AcZ = 0;
float roll_offset = 0, pitch_offset = 0;
float yaw_rate_dps = 0;

const float MAX_RATE = 30.0f;
float hedef_rate_roll = 0, hedef_rate_pitch = 0;
float Kp_angle = 3.0f;
float Kp_rate = 0.5f, Ki_rate = 0.0f, Kd_rate = 0.05f;
float rate_error_roll = 0, rate_error_pitch = 0;
float rate_i_mem_roll = 0, rate_i_mem_pitch = 0;
float rate_last_error_roll = 0, rate_last_error_pitch = 0;
float son_d_rate_roll = 0, son_d_rate_pitch = 0;
float pid_output_roll = 0, pid_output_pitch = 0;

float Kp_alt = 80.0f, Ki_alt = 0.0f, Kd_alt = 40.0f;
float alt_error = 0, alt_i_mem = 0, alt_last_error = 0, pid_output_z = 0;
bool  altPidAktif = false;
bool  motorlarAktif = false;

float inis_hedef_z      = 0;
bool  inis_aktif        = false;
bool  lidarGecerli      = false;
unsigned long sonLidarZaman  = 0;
unsigned long sonInisGunc    = 0;
bool  lidarOncekiGecerli = false;

// ─────────────────────────────────────────────────────────────
//  ESC PWM
// ─────────────────────────────────────────────────────────────
void writeESC(int pin, int us) {
  int s = (us > 950) ? us + 6 : us;
  analogWrite(pin, (s * 4096) / 4000);
}

// ─────────────────────────────────────────────────────────────
//  MPU OKUMA
// ─────────────────────────────────────────────────────────────
void mpu_oku() {
  MPUWire.beginTransmission(MPU_ADRES);
  MPUWire.write(0x3B);
  MPUWire.endTransmission(false);
  uint8_t alinan = MPUWire.requestFrom((uint8_t)MPU_ADRES, (size_t)14, true);
  if (alinan != 14) return;
  AcX = MPUWire.read() << 8 | MPUWire.read();
  AcY = MPUWire.read() << 8 | MPUWire.read();
  AcZ = MPUWire.read() << 8 | MPUWire.read();
  Tmp = MPUWire.read() << 8 | MPUWire.read();
  GyX = MPUWire.read() << 8 | MPUWire.read();
  GyY = MPUWire.read() << 8 | MPUWire.read();
  GyZ = MPUWire.read() << 8 | MPUWire.read();
}

// ─────────────────────────────────────────────────────────────
//  KALİBRASYON
// ─────────────────────────────────────────────────────────────
void mpu_kalibre_et() {
  gyro_X_cal = 0; gyro_Y_cal = 0; gyro_Z_cal = 0;
  for (int i = 0; i < 500; i++) {
    mpu_oku();
    gyro_X_cal += GyX; gyro_Y_cal += GyY; gyro_Z_cal += GyZ;
    delay(3);
  }
  gyro_X_cal /= 500; gyro_Y_cal /= 500; gyro_Z_cal /= 500;

  mpu_oku();
  roll_offset  =  atan2((float)AcY, sqrt((float)AcX*(float)AcX + (float)AcZ*(float)AcZ)) * 57.296f;
  pitch_offset = -atan2((float)AcX, sqrt((float)AcY*(float)AcY + (float)AcZ*(float)AcZ)) * 57.296f;
  angle_roll = roll_offset; angle_pitch = pitch_offset;

  rate_i_mem_roll = 0;      rate_i_mem_pitch = 0;
  rate_last_error_roll = 0; rate_last_error_pitch = 0;
  son_d_rate_roll = 0;      son_d_rate_pitch = 0;
  pid_output_roll = 0;      pid_output_pitch = 0;
  hedef_rate_roll = 0;      hedef_rate_pitch = 0;
  rate_error_roll = 0;      rate_error_pitch = 0;

  alt_i_mem = 0; alt_last_error = 0; pid_output_z = 0; alt_error = 0;

  filtrelenmis_GyX = 0; filtrelenmis_GyY = 0; filtrelenmis_GyZ = 0;
  yaw_rate_dps = 0;
  filt_AcX = (float)AcX; filt_AcY = (float)AcY; filt_AcZ = (float)AcZ;
  loop_timer = micros();
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 — SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  analogWriteFreq(250);
  analogWriteResolution(12);

  for (int i = 0; i < 4; i++) {
    pinMode(escPinleri[i], OUTPUT);
    writeESC(escPinleri[i], 900);
  }
  delay(3000);

  MPUWire.setSDA(0); MPUWire.setSCL(1);
  MPUWire.begin();
  MPUWire.setClock(400000);

  MPUWire.beginTransmission(MPU_ADRES);
  MPUWire.write(0x6B); MPUWire.write(0x00);
  MPUWire.endTransmission(true);
  delay(50);

  MPUWire.beginTransmission(MPU_ADRES);
  MPUWire.write(0x1A); MPUWire.write(0x03);
  MPUWire.endTransmission(true);

  MPUWire.setClock(100000);
  delay(20);

  if (!lox.begin(0x29, false, &MPUWire)) {
    lazerAktif = false;
  } else {
    lazerAktif = true;
    lox.startRangeContinuous(33);
  }

  MPUWire.setClock(400000);

  mpu_kalibre_et();

  // Volatile kopyaları güncelle
  v_lazerAktif   = lazerAktif;
  v_bazGaz       = bazGaz;
  v_hedef_z      = hedef_z;
  v_motorlarAktif = motorlarAktif;
  v_altPidAktif  = altPidAktif;

  systemState = IDLE_CHARGING;
  loop_timer  = micros();
}

// ─────────────────────────────────────────────────────────────
//  CORE 0 — ANA DÖNGÜ
// ─────────────────────────────────────────────────────────────
void loop() {

  // ── dt hesabı ─────────────────────────────────────────────
  unsigned long suankiZaman = micros();
  dt = (suankiZaman - oncekiZaman) / 1000000.0f;
  oncekiZaman = suankiZaman;
  if (dt > 0.05f)    dt = 0.05f;
  if (dt <= 0.0001f) dt = 0.0001f;

  // ── Core 1'den gelen komutları yerel değişkenlere uygula ──
  // (Volatile okumalar — sadece ihtiyaç duyulan değerler okunur)
  bazGaz       = v_bazGaz;
  hedef_z      = v_hedef_z;
  motorlarAktif = v_motorlarAktif;
  altPidAktif  = v_altPidAktif;
  Kp_angle     = v_Kp_angle;
  Kp_rate      = v_Kp_rate;
  Ki_rate      = v_Ki_rate;
  Kd_rate      = v_Kd_rate;
  Kp_alt       = v_Kp_alt;
  Ki_alt       = v_Ki_alt;
  Kd_alt       = v_Kd_alt;

  // ── Sensör Okuma ──────────────────────────────────────────
  mpu_oku();
  loop_timer = micros();

  // ── İvmeölçer LPF (~8Hz) ──────────────────────────────────
  filt_AcX = 0.85f*filt_AcX + 0.15f*(float)AcX;
  filt_AcY = 0.85f*filt_AcY + 0.15f*(float)AcY;
  filt_AcZ = 0.85f*filt_AcZ + 0.15f*(float)AcZ;

  // ── Gyro LPF (~30Hz) ──────────────────────────────────────
  filtrelenmis_GyX = filtrelenmis_GyX*0.8f + ((float)GyX - gyro_X_cal)*0.2f;
  filtrelenmis_GyY = filtrelenmis_GyY*0.8f + ((float)GyY - gyro_Y_cal)*0.2f;
  filtrelenmis_GyZ = filtrelenmis_GyZ*0.8f + ((float)GyZ - gyro_Z_cal)*0.2f;

  float rate_roll  = filtrelenmis_GyX / 131.0f;
  float rate_pitch = filtrelenmis_GyY / 131.0f;
  yaw_rate_dps     = filtrelenmis_GyZ / 131.0f;

  // ── Açı Entegrasyonu + Complementary Filtre ───────────────
  angle_roll  += rate_roll  * dt;
  angle_pitch += rate_pitch * dt;

  float acc_roll  =  atan2(filt_AcY, sqrt(filt_AcX*filt_AcX + filt_AcZ*filt_AcZ)) * 57.296f;
  float acc_pitch = -atan2(filt_AcX, sqrt(filt_AcY*filt_AcY + filt_AcZ*filt_AcZ)) * 57.296f;
  angle_roll  = angle_roll  * 0.995f + acc_roll  * 0.005f;
  angle_pitch = angle_pitch * 0.995f + acc_pitch * 0.005f;

  float kusursuz_roll  = angle_roll  - roll_offset;
  float kusursuz_pitch = angle_pitch - pitch_offset;
  float angle_error_roll  = -kusursuz_roll;
  float angle_error_pitch = -kusursuz_pitch;
  if (fabsf(angle_error_roll)  < 1.5f) angle_error_roll  = 0;
  if (fabsf(angle_error_pitch) < 1.5f) angle_error_pitch = 0;

  // ── Cascade PID (Roll / Pitch) ────────────────────────────
  if (bazGaz <= 1050 || !motorlarAktif) {
    pid_output_roll = 0;      pid_output_pitch = 0;
    hedef_rate_roll = 0;      hedef_rate_pitch = 0;
    rate_i_mem_roll = 0;      rate_i_mem_pitch = 0;
    rate_last_error_roll = 0; rate_last_error_pitch = 0;
    son_d_rate_roll = 0;      son_d_rate_pitch = 0;
    rate_error_roll = 0;      rate_error_pitch = 0;
  } else {
    hedef_rate_roll  = constrain(Kp_angle*angle_error_roll,  -MAX_RATE, MAX_RATE);
    hedef_rate_pitch = constrain(Kp_angle*angle_error_pitch, -MAX_RATE, MAX_RATE);

    rate_error_roll  = hedef_rate_roll  - rate_roll;
    rate_error_pitch = hedef_rate_pitch - rate_pitch;

    rate_i_mem_roll  = constrain(rate_i_mem_roll  + Ki_rate*rate_error_roll *dt, -200, 200);
    rate_i_mem_pitch = constrain(rate_i_mem_pitch + Ki_rate*rate_error_pitch*dt, -200, 200);

    float dr = (rate_error_roll  - rate_last_error_roll)  / dt;
    float dp = (rate_error_pitch - rate_last_error_pitch) / dt;
    son_d_rate_roll  = son_d_rate_roll *0.85f + dr*0.15f;
    son_d_rate_pitch = son_d_rate_pitch*0.85f + dp*0.15f;

    pid_output_roll  = constrain(Kp_rate*rate_error_roll  + rate_i_mem_roll  + Kd_rate*son_d_rate_roll,  -400, 400);
    pid_output_pitch = constrain(Kp_rate*rate_error_pitch + rate_i_mem_pitch + Kd_rate*son_d_rate_pitch, -400, 400);

    rate_last_error_roll  = rate_error_roll;
    rate_last_error_pitch = rate_error_pitch;
  }

  // ═══════════════════════════════════════════════════════════
  //  GÜVENLİ İNİŞ ALGORİTMASI
  // ═══════════════════════════════════════════════════════════
  // Core 1'den gelen inis_aktif durumunu senkronize et
  inis_aktif = v_inis_aktif;

  if (inis_aktif && systemState == LANDING && motorlarAktif) {
    unsigned long now = millis();

    lidarGecerli = lazerAktif && (now - sonLidarZaman < 400);

    if (lidarGecerli && !lidarOncekiGecerli && current_z > 0.1f) {
      inis_hedef_z   = current_z;
      hedef_z        = inis_hedef_z;
      altPidAktif    = true;
      alt_i_mem      = 0;
      alt_last_error = 0;
      // Core 1'e bildir
      v_altPidAktif      = true;
      v_hedef_z          = hedef_z;
      v_inis_hedef_z     = inis_hedef_z;
      flag_lidar_devralıs = true;
    }
    lidarOncekiGecerli = lidarGecerli;

    if (now - sonInisGunc >= 100) {
      sonInisGunc = now;

      if (lidarGecerli && current_z > 0.07f) {
        float inis_hizi;
        if      (inis_hedef_z > 0.50f) inis_hizi = 0.018f;
        else if (inis_hedef_z > 0.20f) inis_hizi = 0.008f;
        else if (inis_hedef_z > 0.10f) inis_hizi = 0.003f;
        else                            inis_hizi = 0.001f;

        inis_hedef_z = max(0.0f, inis_hedef_z - inis_hizi);
        hedef_z      = inis_hedef_z;
        v_inis_hedef_z = inis_hedef_z;
        v_hedef_z      = hedef_z;

      } else if (!lidarGecerli) {
        if (bazGaz > min_ucus_gazi) {
          bazGaz -= 3;
          bazGaz = max(bazGaz, min_ucus_gazi - 20);
          v_bazGaz = bazGaz;
        }
        altPidAktif   = false;
        v_altPidAktif = false;
      }
    }

    bool lizarDokunusu = lidarGecerli && current_z < 0.07f;
    bool korDokunusu   = !lidarGecerli && bazGaz <= min_ucus_gazi;

    if (lizarDokunusu || korDokunusu) {
      motorlarAktif  = false;
      bazGaz         = 900;
      altPidAktif    = false;
      inis_aktif     = false;
      hedef_z        = 1.2f;
      systemState    = IDLE_CHARGING;
      // Volatile senkronizasyon
      v_motorlarAktif     = false;
      v_bazGaz            = 900;
      v_altPidAktif       = false;
      v_inis_aktif        = false;
      v_hedef_z           = 1.2f;
      flag_inis_tamamlandi = true;
    }
  }

  // ── Altitude PID ──────────────────────────────────────────
  if (lazerAktif && altPidAktif && motorlarAktif && bazGaz > 1050) {
    alt_error = hedef_z - current_z;
    if (fabsf(alt_error) < 0.03f) alt_error = 0;
    alt_i_mem = constrain(alt_i_mem + Ki_alt * alt_error * dt, -150, 150);
    float alt_d = (alt_error - alt_last_error) / dt;
    alt_last_error = alt_error;
    pid_output_z = constrain(Kp_alt*alt_error + alt_i_mem + Kd_alt*alt_d, -100, 100);
  } else if (!altPidAktif) {
    pid_output_z = 0; alt_i_mem = 0; alt_last_error = 0; alt_error = 0;
  }

  // ── Motor Matrisi ─────────────────────────────────────────
  int m1 = 900, m2 = 900, m3 = 900, m4 = 900;
  if (motorlarAktif && bazGaz > 1050) {
    int ana_guc = bazGaz + (int)pid_output_z;
    m1 = constrain(ana_guc + pid_output_pitch - pid_output_roll, min_ucus_gazi, 2000);
    m2 = constrain(ana_guc - pid_output_pitch - pid_output_roll, min_ucus_gazi, 2000);
    m3 = constrain(ana_guc + pid_output_pitch + pid_output_roll, min_ucus_gazi, 2000);
    m4 = constrain(ana_guc - pid_output_pitch + pid_output_roll, min_ucus_gazi, 2000);
  }
  writeESC(escPinleri[0], m1); writeESC(escPinleri[1], m2);
  writeESC(escPinleri[2], m3); writeESC(escPinleri[3], m4);

  // ── Lidar — Non-blocking ──────────────────────────────────
  if (lazerAktif && lox.isRangeComplete()) {
    uint16_t r = lox.readRange();
    if (r != 8190 && r != 8191) {
      current_z     = r / 1000.0f;
      sonLidarZaman = millis();
      lidarGecerli  = true;
      // Telemetri için güncelle
      v_current_z    = current_z;
      v_lidarGecerli = true;
    }
  }

  // ── Telemetri snapshot (150ms) — Core 1 okuyacak ─────────
  static unsigned long sonSnapshot = 0;
  if (millis() - sonSnapshot > 150) {
    sonSnapshot = millis();
    v_kusursuz_roll    = kusursuz_roll;
    v_kusursuz_pitch   = kusursuz_pitch;
    v_yaw_rate_dps     = yaw_rate_dps;
    v_alt_error        = alt_error;
    v_pid_output_z     = pid_output_z;
    v_rate_error_roll  = rate_error_roll;
    v_rate_error_pitch = rate_error_pitch;
    // Lidar geçerlilik durumunu senkronize et
    v_lidarGecerli = lidarGecerli;
  }
}

// ═══════════════════════════════════════════════════════════════
//  RING BUFFER — CORE 1 TX
// ═══════════════════════════════════════════════════════════════
#define RING_BUF_SIZE 512
static char  ring_buf[RING_BUF_SIZE];
static int   ring_head = 0;
static int   ring_tail = 0;

static void ring_push(const char* str) {
  while (*str) {
    int next = (ring_head + 1) % RING_BUF_SIZE;
    if (next == ring_tail) break;  // Dolu — at
    ring_buf[ring_head] = *str++;
    ring_head = next;
  }
}

// Her çağrıda en fazla maxBytes bayt gönderir — bloklamaz
static void ring_flush(Stream& port, int maxBytes) {
  int sent = 0;
  while (ring_tail != ring_head && sent < maxBytes) {
    if (port.availableForWrite() < 1) break;
    port.write(ring_buf[ring_tail]);
    ring_tail = (ring_tail + 1) % RING_BUF_SIZE;
    sent++;
  }
}

// ═══════════════════════════════════════════════════════════════
//  CORE 1 — KOMUT İŞLEME (inline, Stream alır)
// ═══════════════════════════════════════════════════════════════
static void komut_islet_c1(char* tampon, Stream& port) {
  char  b = tampon[0];
  float val = atof(&tampon[1]);

  // Cascade PID
  if      (b=='q'||b=='Q') { v_Kp_angle = val; port.print("Kp_angle="); port.println(val); }
  else if (b=='p'||b=='P') { v_Kp_rate  = val; port.print("Kp_rate=");  port.println(val); }
  else if (b=='i'||b=='I') { v_Ki_rate  = val; port.print("Ki_rate=");  port.println(val); }
  else if (b=='d'||b=='D') { v_Kd_rate  = val; port.print("Kd_rate=");  port.println(val); }
  // Altitude PID
  else if (b=='z'||b=='Z') { v_Kp_alt = val; port.print("Kp_alt="); port.println(val); }
  else if (b=='x'||b=='X') { v_Ki_alt = val; port.print("Ki_alt="); port.println(val); }
  else if (b=='v'||b=='V') { v_Kd_alt = val; port.print("Kd_alt="); port.println(val); }

  else if (b=='t'||b=='T') {
    if (!v_lazerAktif) { port.println("HATA: Lazer yok, AltPID açılamaz!"); return; }
    if (systemState == LANDING) { port.println("HATA: Inis sirasinda AltPID toggle edilemez!"); return; }
    bool yeni = !v_altPidAktif;
    if (yeni) { /* i_mem sıfırlamayı Core 0 snippet ile sync etmek gerekmiyor;
                   Core 0 altPidAktif = v_altPidAktif okuduğunda zaten temizler */ }
    v_altPidAktif = yeni;
    port.print("AltPID=");
    port.println(yeni ? "ACIK" : "KAPALI");
  }

  else if (b=='l'||b=='L') {
    if (!v_motorlarAktif || v_bazGaz <= 1050) {
      port.println("HATA: Motor aktif degil, inis baslatılamaz!"); return;
    }
    if (systemState == LANDING) {
      port.println("UYARI: Inis zaten devam ediyor!"); return;
    }
    systemState     = LANDING;
    v_inis_aktif    = true;

    bool lidarSimdi = v_lazerAktif && v_lidarGecerli;
    float cz        = v_current_z;
    float baslangic = (lidarSimdi && cz > 0.1f) ? cz : 1.5f;

    v_inis_hedef_z  = baslangic;
    v_hedef_z       = baslangic;
    v_altPidAktif   = lidarSimdi && (cz > 0.1f);

    port.println("GUVENLI INIS BASLATILDI!");
    if (!lidarSimdi) port.println("Lidar menzil disi: kor inis basladi...");
  }

  else if (b=='n'||b=='N') {
    if (systemState == LANDING) {
      v_inis_aktif  = false;
      v_altPidAktif = false;
      systemState   = PATROL;
      port.println("INIS IPTAL: PATROL moduna donuldu!");
    }
  }

  else if (b=='g'||b=='G') {
    if (systemState == LANDING) { port.println("HATA: Inis sirasinda gaz degistirilemez!"); return; }
    v_bazGaz = (int)val;
    if (!v_motorlarAktif && v_bazGaz > 1050) {
      v_motorlarAktif = true;
      systemState     = PATROL;
      flag_armed      = true;
    }
  }

  else if (b=='a'||b=='A') { v_hedef_z = val; port.print("hedef_z="); port.println(val); }

  else if (b=='c'||b=='C') {
    if (!v_motorlarAktif) {
      // Kalibrasyon Core 0'da çalışması gerekir ama motor kapalıyken
      // Core 0 döngüsü boşta; Core 1'den çağırılamaz doğrudan.
      // Güvenli yol: bayrak set et, Core 0 handle etsin.
      // Bu implementasyonda setup() sırasında yapıldığı varsayılır.
      // Pratik çözüm: Core 1'den sadece bildirim yap.
      port.println("Kalibrasyon: Drone'u yeniden baslatın (setup'ta yapılır).");
    } else {
      port.println("HATA: Havada kalibrasyon yasak!");
    }
  }

  else if (b=='s'||b=='S') {
    v_motorlarAktif = false;
    v_bazGaz        = 900;
    v_altPidAktif   = false;
    v_inis_aktif    = false;
    systemState     = IDLE_CHARGING;
    flag_disarmed   = true;
    port.println("DISARMED!");
  }
}

// ─────────────────────────────────────────────────────────────
//  CORE 1 — SETUP
// ─────────────────────────────────────────────────────────────
void setup1() {
  Serial.begin(115200);
  SerialBT.setTX(4); SerialBT.setRX(5);
  SerialBT.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Core 0'ın init bitmesini bekle (lazerAktif set edilene kadar)
  delay(4500);

  Serial.println("--- PICO GCS AKTIF (Core 1) ---");
  Serial.print("Lazer: ");
  Serial.println(v_lazerAktif ? "AKTIF" : "PASIF");
  Serial.println("Q=Kp_angle P=Kp_rate I=Ki_rate D=Kd_rate");
  Serial.println("Z=Kp_alt   X=Ki_alt  V=Kd_alt  T=AltPID");
  Serial.println("G=Gaz A=AltHedef L=GuvenliInis N=Iptal C=Kal S=Stop");
}

// ─────────────────────────────────────────────────────────────
//  CORE 1 — ANA DÖNGÜ
// ─────────────────────────────────────────────────────────────
void loop1() {

  // ── Olay bayraklarını işle ────────────────────────────────
  if (flag_inis_tamamlandi) {
    flag_inis_tamamlandi = false;
    Serial.println("INIS TAMAMLANDI! Motorlar kapatildi.");
    SerialBT.println("INIS TAMAMLANDI!");
  }
  if (flag_lidar_devralıs) {
    flag_lidar_devralıs = false;
    Serial.println("Lidar aktif: gudumlu inise gecildi.");
    SerialBT.println("Lidar aktif: gudumlu inise gecildi.");
  }
  if (flag_armed) {
    flag_armed = false;
    Serial.println("ARMED!");
    SerialBT.println("ARMED!");
  }
  if (flag_disarmed) {
    flag_disarmed = false;
    // "DISARMED!" komut fonksiyonunda zaten gönderildi
  }

  // ── USB komut okuma ───────────────────────────────────────
  static char tamponUSB[64];
  static int  tamponUSBIdx = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      tamponUSB[tamponUSBIdx] = '\0';
      if (tamponUSBIdx > 0) komut_islet_c1(tamponUSB, Serial);
      tamponUSBIdx = 0;
    } else if (c != '\r' && tamponUSBIdx < 63) {
      tamponUSB[tamponUSBIdx++] = c;
    }
  }

  // ── BT komut okuma ────────────────────────────────────────
  static char tamponBT[64];
  static int  tamponBTIdx = 0;
  while (SerialBT.available() > 0) {
    char c = SerialBT.read();
    if (c == '\n') {
      tamponBT[tamponBTIdx] = '\0';
      if (tamponBTIdx > 0) komut_islet_c1(tamponBT, SerialBT);
      tamponBTIdx = 0;
    } else if (c != '\r' && tamponBTIdx < 63) {
      tamponBT[tamponBTIdx++] = c;
    }
  }

  // ── Telemetri paketi (150ms) ──────────────────────────────
  static unsigned long sonYaz = 0;
  if (millis() - sonYaz > 150) {
    sonYaz = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Snapshot — volatile değerleri yerel kopyaya al
    float  s_roll      = v_kusursuz_roll;
    float  s_pitch     = v_kusursuz_pitch;
    float  s_yaw       = v_yaw_rate_dps;
    int    s_gaz       = v_bazGaz;
    float  s_alt       = v_current_z;
    float  s_kp        = v_Kp_angle;
    float  s_ki        = v_Ki_rate;
    float  s_kd        = v_Kd_rate;
    float  s_rr        = v_rate_error_roll;
    float  s_rp        = v_rate_error_pitch;
    float  s_za        = v_Kp_alt;
    float  s_zi        = v_Ki_alt;
    float  s_zd        = v_Kd_alt;
    float  s_ze        = v_alt_error;
    float  s_zp        = v_pid_output_z;
    int    s_zt        = v_altPidAktif ? 1 : 0;
    int    s_lz        = v_lidarGecerli ? 1 : 0;
    float  s_ln        = v_inis_aktif ? (float)v_inis_hedef_z : -1.0f;
    int    s_st        = (int)systemState;

    char pkt[240];
    snprintf(pkt, sizeof(pkt),
      "R:%.1f P:%.1f Y:%.1f G:%d Alt:%.2f Kp:%.2f Ki:%.4f Kd:%.3f rR:%.1f rP:%.1f Za:%.1f Zi:%.3f Zd:%.1f Ze:%.2f Zp:%.0f Zt:%d Lz:%d Ln:%.2f St:%d",
      s_roll, s_pitch, s_yaw,
      s_gaz, s_alt,
      s_kp, s_ki, s_kd,
      s_rr, s_rp,
      s_za, s_zi, s_zd,
      s_ze, s_zp,
      s_zt, s_lz, s_ln, s_st);

    // Ring buffer'a yaz — hem USB hem BT'ye kopyala
    ring_push(pkt);
    ring_push("\r\n");

    // USB'ye doğrudan yaz (USB genellikle hızlıdır)
    Serial.println(pkt);
  }

  // ── Ring buffer'ı BT'ye aktar (non-blocking, 32 byte/tur) ─
  ring_flush(SerialBT, 32);
}
