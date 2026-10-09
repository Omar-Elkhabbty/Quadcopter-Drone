#include <Arduino.h>
#include <Wire.h>

// ================================================================
// 1. تعاريف المنافذ والترددات (Pinout & Hardware Config)
// ================================================================
#define MOTOR1_PIN 13      // Front Right (CCW)
#define MOTOR2_PIN 12      // Rear Right  (CW)
#define MOTOR3_PIN 14      // Rear Left   (CCW)
#define MOTOR4_PIN 27      // Front Left  (CW)

#define GPS_RX_PIN 18      // Hardware UART1 RX
#define GPS_TX_PIN 17      // Hardware UART1 TX
#define IBUS_RX_PIN 16     // Hardware UART2 RX
#define VBAT_ADC_PIN 34    // Analog Input for Voltage Sensing

#define PWM_FREQ 250       // 250 Hz (4ms Refresh Rate)
#define PWM_RES 12         // 12-bit Resolution (0 - 4095)
#define ESC_MIN_DUTY 1024  // ~1000us (إيقاف المحرك)
#define ESC_MAX_DUTY 2048  // ~2000us (السرعة القصوى)

#define MPU6050_ADDR 0x68
#define BMP280_ADDR  0x76

// ================================================================
// 2. الهياكل المتغيرات العامة (Global System Variables)
// ================================================================

// أ. بيانات تحديد المواقع GPS
struct GPS_Data {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0;
  uint8_t satellites = 0;
  bool fix_valid = false;
};
GPS_Data current_gps;
GPS_Data home_gps;
bool home_locked = false;
String gps_buffer = "";

// ب. قنوات أجهزة التحكم iBUS (FlySky)
uint8_t ibus_buffer[32];
uint8_t ibus_idx = 0;
uint16_t rc_channel[6]; // Ch1:Roll, Ch2:Pitch, Ch3:Throttle, Ch4:Yaw, Ch5:Arm, Ch6:FlightMode
bool is_armed = false;
bool angle_mode_active = false;

// ج. بيانات حساس الاتزان والزوايا (IMU & Fusion)
int16_t raw_acc_x, raw_acc_y, raw_acc_z;
int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
float gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z;
float gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

float angle_roll_acc, angle_pitch_acc;
float angle_roll = 0.0, angle_pitch = 0.0; // الزوايا المحسوبة بالمرشح المكمل

// د. بيانات البارومتر والارتفاع (BMP280 Data)
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
float baro_altitude = 0.0;
float sea_level_pressure = 101325.0; // Pa

// هـ. متغيرات نظام الطاقة والبطارية
float battery_voltage = 0.0;

// و. زمن الحلقة الحسابية Loop Timing
unsigned long prev_micros = 0;
float dt = 0.004; // 4ms default for 250Hz

// ز. أهداف التوجيه ومعاملات التحكم PID
float req_roll_rate = 0, req_pitch_rate = 0, req_yaw_rate = 0;
int req_throttle = 1000;

// معاملات PID (Roll / Pitch / Yaw)
float kp_roll = 1.3, ki_roll = 0.04, kd_roll = 18.0;
float pid_roll = 0, error_roll = 0, prev_error_roll = 0, integral_roll = 0;

float kp_pitch = 1.3, ki_pitch = 0.04, kd_pitch = 18.0;
float pid_pitch = 0, error_pitch = 0, prev_error_pitch = 0, integral_pitch = 0;

float kp_yaw = 2.0, ki_yaw = 0.05, kd_yaw = 0.0;
float pid_yaw = 0, error_yaw = 0, prev_error_yaw = 0, integral_yaw = 0;

// ================================================================
// 3. درايفرات الحساسات عبر الناقل I2C (MPU6050 & BMP280)
// ================================================================

void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Power Management 1
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // Gyro Config
  Wire.write(0x08); // ±500 deg/s range (65.5 LSB/deg/s)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // Accel Config
  Wire.write(0x10); // ±8g range (4096 LSB/g)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // DLPF Config
  Wire.write(0x03); // ~42Hz Low Pass Filter لخفض الاهتزازات
  Wire.endTransmission(true);
}

void calibrateGyro() {
  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)6, true);

    gyro_x_cal += (Wire.read() << 8 | Wire.read()) / 65.5;
    gyro_y_cal += (Wire.read() << 8 | Wire.read()) / 65.5;
    gyro_z_cal += (Wire.read() << 8 | Wire.read()) / 65.5;
    delayMicroseconds(1000);
  }
  gyro_x_cal /= 2000.0;
  gyro_y_cal /= 2000.0;
  gyro_z_cal /= 2000.0;
}

void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for Accel & Gyro
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true);

  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature bytes
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();

  // تحويل القراءات إلى وحدات معيارية
  gyro_x = ((float)raw_gyro_x / 65.5) - gyro_x_cal;
  gyro_y = ((float)raw_gyro_y / 65.5) - gyro_y_cal;
  gyro_z = ((float)raw_gyro_z / 65.5) - gyro_z_cal;

  acc_x = (float)raw_acc_x / 4096.0;
  acc_y = (float)raw_acc_y / 4096.0;
  acc_z = (float)raw_acc_z / 4096.0;

  // حساب زوايا الميل من التسارع
  angle_pitch_acc = atan2(acc_y, sqrt(acc_x * acc_x + acc_z * acc_z)) * 57.296;
  angle_roll_acc  = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 57.296;

  // المرشح المكمل (Complementary Filter) لدمج الجايروسكوب والمسرع
  angle_roll  = 0.98 * (angle_roll + gyro_x * dt) + 0.02 * angle_roll_acc;
  angle_pitch = 0.98 * (angle_pitch + gyro_y * dt) + 0.02 * angle_pitch_acc;
}

void initBMP280() {
  // قراءة معاملات المعايرة الخاصة بالبارومتر
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0x88);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (size_t)24, true);

  dig_T1 = Wire.read() | (Wire.read() << 8);
  dig_T2 = Wire.read() | (Wire.read() << 8);
  dig_T3 = Wire.read() | (Wire.read() << 8);
  dig_P1 = Wire.read() | (Wire.read() << 8);
  dig_P2 = Wire.read() | (Wire.read() << 8);
  dig_P3 = Wire.read() | (Wire.read() << 8);
  dig_P4 = Wire.read() | (Wire.read() << 8);
  dig_P5 = Wire.read() | (Wire.read() << 8);
  dig_P6 = Wire.read() | (Wire.read() << 8);
  dig_P7 = Wire.read() | (Wire.read() << 8);
  dig_P8 = Wire.read() | (Wire.read() << 8);
  dig_P9 = Wire.read() | (Wire.read() << 8);

  // ضبط أطوار التشغيل
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xF4); // ctrl_meas
  Wire.write(0x27); // Normal mode, temp and press oversampling x1
  Wire.endTransmission(true);
}

void readBarometer() {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(0xF7); // Press Data MSB
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP280_ADDR, (size_t)6, true);

  int32_t adc_P = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
  int32_t adc_T = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);

  // معادلات التعويض الحراري للضغط
  int32_t var1_T = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2_T = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  int32_t t_fine = var1_T + var2_T;

  int64_t var1_P = ((int64_t)t_fine) - 128000;
  int64_t var2_P = var1_P * var1_P * (int64_t)dig_P6;
  var2_P = var2_P + ((var1_P * (int64_t)dig_P5) << 17);
  var2_P = var2_P + (((int64_t)dig_P4) << 35);
  var1_P = ((var1_P * var1_P * (int64_t)dig_P3) >> 8) + ((var1_P * (int64_t)dig_P2) << 12);
  var1_P = (((((int64_t)1) << 47) + var1_P)) * ((int64_t)dig_P1) >> 33;

  if (var1_P != 0) {
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2_P) * 3125) / var1_P;
    var1_P = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2_P = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1_P + var2_P) >> 8) + (((int64_t)dig_P7) << 4);
    float pressure = (float)p / 256.0;

    // حساب الارتفاع التقريبي بالأمتار
    baro_altitude = 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
  }
}

// ================================================================
// 4. مفكك بيانات مستقبل الراديو FlySky iBUS
// ================================================================

void readIBUS() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (ibus_idx == 0 && b != 0x20) continue;
    if (ibus_idx == 1 && b != 0x40) { ibus_idx = 0; continue; }

    ibus_buffer[ibus_idx++] = b;

    if (ibus_idx == 32) {
      ibus_idx = 0;

      uint16_t chksum = 0xFFFF;
      for (int i = 0; i < 30; i++) chksum -= ibus_buffer[i];
      uint16_t rx_chksum = ibus_buffer[30] | (ibus_buffer[31] << 8);

      if (chksum == rx_chksum) {
        for (int ch = 0; ch < 6; ch++) {
          rc_channel[ch] = ibus_buffer[2 + (ch * 2)] | (ibus_buffer[3 + (ch * 2)] << 8);
        }

        // تحويل أجهزة التحكم إلى أوامر للحركة
        req_roll_rate  = ((float)rc_channel[0] - 1500.0) * (150.0 / 500.0);
        req_pitch_rate = ((float)rc_channel[1] - 1500.0) * (150.0 / 500.0);
        req_throttle   = constrain(rc_channel[2], 1000, 2000);
        req_yaw_rate   = ((float)rc_channel[3] - 1500.0) * (200.0 / 500.0);

        is_armed          = (rc_channel[4] > 1500); // مفتاح الأمان Aux 1
        angle_mode_active = (rc_channel[5] > 1500); // مفتاح النمط Aux 2

        // حفظ نقطة الإقلاع للـ GPS فور التسليح
        if (is_armed && !home_locked && current_gps.fix_valid) {
          home_gps = current_gps;
          home_locked = true;
        } else if (!is_armed) {
          home_locked = false;
        }
      }
    }
  }
}

// ================================================================
// 5. مفكك بيانات نظام الـ GPS (NMEA Sentence Parser)
// ================================================================

void parseNMEA(String line) {
  if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
    int comma[14];
    int c_idx = 0;
    for (int i = 0; i < line.length(); i++) {
      if (line.charAt(i) == ',') {
        if (c_idx < 14) comma[c_idx++] = i;
      }
    }

    if (c_idx >= 9) {
      String fix_str = line.substring(comma[5] + 1, comma[6]);
      String sat_str = line.substring(comma[6] + 1, comma[7]);
      String alt_str = line.substring(comma[8] + 1, comma[9]);

      current_gps.satellites = sat_str.toInt();
      current_gps.fix_valid = (fix_str.toInt() > 0);
      current_gps.altitude = alt_str.toFloat();

      // Latitude Parsing
      String raw_lat = line.substring(comma[1] + 1, comma[2]);
      String lat_dir = line.substring(comma[2] + 1, comma[3]);
      if (raw_lat.length() > 4) {
        double deg = raw_lat.substring(0, 2).toDouble();
        double min = raw_lat.substring(2).toDouble();
        current_gps.latitude = deg + (min / 60.0);
        if (lat_dir == "S") current_gps.latitude *= -1.0;
      }

      // Longitude Parsing
      String raw_lon = line.substring(comma[3] + 1, comma[4]);
      String lon_dir = line.substring(comma[4] + 1, comma[5]);
      if (raw_lon.length() > 5) {
        double deg = raw_lon.substring(0, 3).toDouble();
        double min = raw_lon.substring(3).toDouble();
        current_gps.longitude = deg + (min / 60.0);
        if (lon_dir == "W") current_gps.longitude *= -1.0;
      }
    }
  }
}

void readGPS() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      parseNMEA(gps_buffer);
      gps_buffer = "";
    } else if (c != '\r') {
      gps_buffer += c;
    }
  }
}

// ================================================================
// 6. قراءة مستوى شحن البطارية (Battery ADC Sensing)
// ================================================================

void checkBattery() {
  int raw_adc = analogRead(VBAT_ADC_PIN);
  // مقسم جهد R1 = 33k, R2 = 10k -> معامل الخفض = (33+10)/10 = 4.3
  battery_voltage = ((float)raw_adc / 4095.0) * 3.3 * 4.3;
}

// ================================================================
// 7. خوارزمية الاتزان وتعديل السرعة (PID & Motor Mixing)
// ================================================================

void calculatePID() {
  // إذا كان نمط Angle Mode مفعلاً، يتم تحويل خطأ الزوايا لسرعة دورانية أصلية
  float target_roll_rate = req_roll_rate;
  float target_pitch_rate = req_pitch_rate;

  if (angle_mode_active) {
    target_roll_rate  = (req_roll_rate - angle_roll) * 3.0;
    target_pitch_rate = (req_pitch_rate - angle_pitch) * 3.0;
  }

  // 1. Roll PID Loop
  error_roll = target_roll_rate - gyro_x;
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -400.0, 400.0);
  float derivative_roll = (error_roll - prev_error_roll) / dt;
  pid_roll = (kp_roll * error_roll) + (ki_roll * integral_roll) + (kd_roll * derivative_roll);
  prev_error_roll = error_roll;

  // 2. Pitch PID Loop
  error_pitch = target_pitch_rate - gyro_y;
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -400.0, 400.0);
  float derivative_pitch = (error_pitch - prev_error_pitch) / dt;
  pid_pitch = (kp_pitch * error_pitch) + (ki_pitch * integral_pitch) + (kd_pitch * derivative_pitch);
  prev_error_pitch = error_pitch;

  // 3. Yaw PID Loop
  error_yaw = req_yaw_rate - gyro_z;
  integral_yaw += error_yaw * dt;
  integral_yaw = constrain(integral_yaw, -400.0, 400.0);
  float derivative_yaw = (error_yaw - prev_error_yaw) / dt;
  pid_yaw = (kp_yaw * error_yaw) + (ki_yaw * integral_yaw) + (kd_yaw * derivative_yaw);
  prev_error_yaw = error_yaw;
}

void writeMotors() {
  // وضع الأمان: قطع الإشارة تماماً إذا كان مفتاح الأمان مفعّلاً أو دواسة البنزين أقل من 1050
  if (!is_armed || req_throttle < 1050) {
    ledcWrite(MOTOR1_PIN, ESC_MIN_DUTY);
    ledcWrite(MOTOR2_PIN, ESC_MIN_DUTY);
    ledcWrite(MOTOR3_PIN, ESC_MIN_DUTY);
    ledcWrite(MOTOR4_PIN, ESC_MIN_DUTY);
    integral_roll = integral_pitch = integral_yaw = 0;
    return;
  }

  // مصفوفة الخلط (Quad-X Motor Mixing)
  int m1 = req_throttle - pid_roll + pid_pitch + pid_yaw; // Front Right (CCW)
  int m2 = req_throttle - pid_roll - pid_pitch - pid_yaw; // Rear Right  (CW)
  int m3 = req_throttle + pid_roll - pid_pitch + pid_yaw; // Rear Left   (CCW)
  int m4 = req_throttle + pid_roll + pid_pitch - pid_yaw; // Front Left  (CW)

  // تقييد السرعة بين الحد الأدنى والأقصى للـ ESCs
  m1 = constrain(m1, ESC_MIN_DUTY, ESC_MAX_DUTY);
  m2 = constrain(m2, ESC_MIN_DUTY, ESC_MAX_DUTY);
  m3 = constrain(m3, ESC_MIN_DUTY, ESC_MAX_DUTY);
  m4 = constrain(m4, ESC_MIN_DUTY, ESC_MAX_DUTY);

  // إرسال إشارات PWM
  ledcWrite(MOTOR1_PIN, m1);
  ledcWrite(MOTOR2_PIN, m2);
  ledcWrite(MOTOR3_PIN, m3);
  ledcWrite(MOTOR4_PIN, m4);
}

void initPWM() {
  ledcAttach(MOTOR1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR2_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR3_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR4_PIN, PWM_FREQ, PWM_RES);

  ledcWrite(MOTOR1_PIN, ESC_MIN_DUTY);
  ledcWrite(MOTOR2_PIN, ESC_MIN_DUTY);
  ledcWrite(MOTOR3_PIN, ESC_MIN_DUTY);
  ledcWrite(MOTOR4_PIN, ESC_MIN_DUTY);
  delay(1000);
}

// ================================================================
// 8. تهيئة النظام والحلقة الرئيسية (Setup & Loop)
// ================================================================

void setup() {
  Serial.begin(115200);                                   // Serial Monitor (UART0)
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // GPS Module (UART1)
  Serial2.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);      // FlySky iBUS (UART2)

  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz I2C Fast Mode

  initMPU6050();
  calibrateGyro();
  initBMP280();
  initPWM();

  prev_micros = micros();
  Serial.println("[SYSTEM] Flight Controller Ready!");
}

void loop() {
  // حاسبة زمن الدورة دقيقة جداً (Fixed 250Hz - 4ms)
  unsigned long now = micros();
  dt = (now - prev_micros) / 1000000.0;
  prev_micros = now;

  readIBUS();       // 1. استقبال قنوات التحكم
  readGPS();        // 2. تحديث بيانات الإحداثيات
  readIMU();        // 3. قراءة الاتزان والزوايا الحالية
  readBarometer();  // 4. قراءة الارتفاع من البارومتر
  checkBattery();   // 5. فحص جهد البطارية
  calculatePID();   // 6. حساب قيمة التصحيح
  writeMotors();    // 7. تطبيق نبضات التحكم على المحركات

  // طباعة بيانات التشخيص والتتبع كل ثانية عبر Monitor
  static unsigned long last_telemetry = 0;
  if (millis() - last_telemetry > 1000) {
    last_telemetry = millis();
    Serial.printf("Armed: %d | VBat: %.2fV | Sats: %d | Lat: %.6f | Lon: %.6f | Alt: %.2fm\n",
                  is_armed, battery_voltage, current_gps.satellites,
                  current_gps.latitude, current_gps.longitude, baro_altitude);
  }

  // الحفاظ الثارم والمطلق على زمن الدورة 250Hz (4000us)
  while (micros() - now < 4000);
}