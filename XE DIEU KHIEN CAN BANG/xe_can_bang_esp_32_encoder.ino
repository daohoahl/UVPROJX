// ================== BLUETOOTH CONTROL CONFIG ==================
// Ký tự điều khiển từ App (dễ sửa ở đây)
#define BT_FORWARD    'F'   // Đi tới
#define BT_BACKWARD   'B'   // Đi lùi  
#define BT_LEFT       'L'   // Quay trái
#define BT_RIGHT      'R'   // Quay phải
#define BT_STOP       'S'   // Dừng

// Tốc độ di chuyển (có thể chỉnh)
// MODE 0: Góc nghiêng (độ) - robot nghiêng để di chuyển (2-5 độ là đủ!)
// MODE 1: PWM offset - cộng trực tiếp vào PWM (có giới hạn góc an toàn)
// MODE 2: Encoder target - di chuyển đến vị trí encoder
#define MOVE_MODE       0      // 0=Angle, 1=PWM, 2=Encoder

// Giá trị cho từng mode:
// Mode 0: góc nghiêng (độ) - NÊN 2~5 độ, KHÔNG nên > 10
// Mode 1: PWM offset (0-255) - sẽ bị giới hạn khi góc lớn
// Mode 2: encoder steps mỗi lần nhấn
#define SPEED_FORWARD   3      // Mode0: 3 độ nghiêng, Mode1: 50 PWM
#define SPEED_BACKWARD  3
#define SPEED_LEFT      40     // PWM offset để quay trái
#define SPEED_RIGHT     40     // PWM offset để quay phải

// Giới hạn an toàn cho Mode 1 (PWM mode)
#define MAX_SAFE_ANGLE  15.0f  // Nếu nghiêng > 15 độ thì tắt ForwardPWM (tránh ngã)
#define PWM_MODE_SPEED  50     // PWM thực sự dùng cho Mode 1

// Timeout điều khiển (ms) - tự động quay về cân bằng khi không có lệnh
#define CMD_TIMEOUT_MS  300    // Sau 300ms không có lệnh mới → reset về cân bằng
#define BUFFER_TIMEOUT_MS 500  // Clear buffer nếu không nhận newline trong 500ms

// Tên Bluetooth (hiện trên điện thoại)
#define BT_NAME "robot_can_bang"

// ================== PIN DEFINITION ==================
#define ENA    25   // PWM motor trái
#define IN1    26   // Hướng motor trái
#define IN2    27
#define ENB    14   // PWM motor phải
#define IN3    13   // Hướng motor phải (nếu bạn dùng IN3=13, IN4=12 thì đổi lại)
#define IN4    12
float angle_offset = 2.5;  // <--- GIÁ TRỊ NÀY LÀ GÓC CÂN BẰNG (OFFSET)
// // Encoder ban ygb37
#define ENC_L_A  34  // Input only
#define ENC_L_B  35
#define ENC_R_A  32
#define ENC_R_B  33

/* Encoder ban ygb37
// #define ENA    25   // PWM motor trái
// #define IN1    27   // Hướng motor trái
// #define IN2    26
// #define ENB    14   // PWM motor phải
// #define IN3    12   // Hướng motor phải (nếu bạn dùng IN3=13, IN4=12 thì đổi lại)
// #define IN4    13
// float angle_offset = 2.5;  // <--- GIÁ TRỊ NÀY LÀ GÓC CÂN BẰNG (OFFSET)
// // Encoder
// #define ENC_L_A  34  // Input only
// #define ENC_L_B  35
// #define ENC_R_A  32
// #define ENC_R_B  33
*/
// I2C MPU6050
#define SDA_PIN  21
#define SCL_PIN  22

// ================== LIBRARIES  ==================
#include <Wire.h>
#include <Kalman.h>  // Thư viện Kalman (class Kalman)
#include "BluetoothSerial.h"  // ESP32 Bluetooth Classic

#ifndef PI
#define PI 3.14159265359f
#endif
#define ToDeg (180.0f / PI)
#define ToRad (PI / 180.0f)

Kalman kalman_h;  // Instance Kalman
BluetoothSerial SerialBT;  // Bluetooth Serial

#define factortheta (PI / 60.0f)
#define factorphi   (PI / 10.0f)

// Precomputed constants for encoder conversion
const float ENC_TO_DEG = 360.0f / 495.0f;
const float THETA_FACTOR = 0.5f * ENC_TO_DEG;
const float PHI_FACTOR = (3.5f / 15.0f) * ENC_TO_DEG;

// ================== VARIABLES ==================
uint32_t timerloop = 0;
uint32_t timer = 0;

volatile long leftencoder = 0;
volatile long righencoder = 0;

double mpudata = 0;  // góc psi (pitch)
double accX, accZ;
float Gyro;
float gyroRate = 0;  // gyro rate in rad/s for LQR (psidot)

uint8_t i2cData[14];

// LQR Gains - DÒ LẠI TỪ ĐẦU
// Bước 1: Chỉ tune K3, K4 (cân bằng góc)
// Bước 2: Thêm K5, K6 (chống xoay)
// Bước 3: Thêm K1, K2 (giữ vị trí)
float K1 = 0.0f;    // TẮT - tune sau
float K2 = 0.0f;    // TẮT - tune sau
float K3 = 2500.0f;   // BẮT ĐẦU: 1° = 15 PWM
float K4 = 0.0f;    // BẮT ĐẦU: damping nhẹ
float K5 = 0.0f;    // TẮT - tune sau
float K6 = 0.0f;    // TẮT - tune sau


// Baseline gains for RESET command
const float K1_BASE = 0.0f, K2_BASE = 0.0f;
const float K3_BASE = 15.0f, K4_BASE = 1.5f;
const float K5_BASE = 0.0f, K6_BASE = 1.0f;

// Deadband: ignore very small PWM requests (reduces jitter)
const int PWM_DEADBAND = 20;  // if |pwm| < this, set to 0

// Debug logging
bool debugLog = false;
int logCounter = 0;
const int LOG_INTERVAL = 10;  // print every N loops (~33Hz at 333Hz loop)

long PWML, PWMR;
bool falldown = false;

// Motor compensation
int min_pwm_L = 110;   // minimum PWM for left motor (reduced)
int min_pwm_R = 110;   // minimum PWM for right motor (reduced)
int motorTrim = 0;     // add to left PWM (positive -> increase left)

// Falldown threshold
const float FALL_ANGLE = 60.0f;  // degrees

float theta, psi, phi;
float thetadot, psidot, phidot;
float thetaold = 0, psiold = 0, phiold = 0;

float addtheta = 0;
float addphi = 0;

float ForwardBack = 0;  // Giá trị điều khiển tiến/lùi
int LeftRight = 0;      // PWM offset: >0 quay trái, <0 quay phải
int ForwardPWM = 0;     // PWM offset cho tiến/lùi (Mode 1)
long targetEncoder = 0; // Encoder target (Mode 2)

// Timeout tracking
unsigned long lastCmdTime = 0;      // Thời điểm nhận lệnh cuối
unsigned long lastBufferTime = 0;   // Thời điểm nhận ký tự cuối vào buffer
bool cmdActive = false;             // Có lệnh di chuyển đang active

// Góc nghiêng mục tiêu để di chuyển (độ)
float targetAngle = 0;  // Khi bấm F: nghiêng về trước để đi tới

const uint8_t IMUAddress = 0x68;

// ================== I2C HELPERS ==================
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop = true) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop);
  return rcode;
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop = true) {
  return i2cWrite(registerAddress, &data, 1, sendStop);
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false);
  if (rcode) return rcode;
  Wire.requestFrom(IMUAddress, nbytes);
  uint8_t index = 0;
  while (Wire.available() && index < nbytes) {
    data[index++] = Wire.read();
  }
  return 0;
}

// ================== ENCODER ISR ==================
void IRAM_ATTR left_isr() {
  if (digitalRead(ENC_L_B)) leftencoder++;
  else leftencoder--;
}

void IRAM_ATTR righ_isr() {
  if (digitalRead(ENC_R_B)) righencoder--;
  else righencoder++;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  SerialBT.begin(BT_NAME);  // Bluetooth device name
  Serial.println("==================================");
  Serial.println("Bluetooth: " BT_NAME);
  Serial.println("Waiting for connection...");
  Serial.println("==================================");

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoder pins
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);
  attachInterrupt(ENC_L_A, left_isr, RISING);
  attachInterrupt(ENC_R_A, righ_isr, RISING);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // MPU6050 init
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4));
  while (i2cWrite(0x6B, 0x01));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {
    Serial.println(F("MPU6050 not found!"));
    while (1);
  }

  delay(100);
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalman_h.setAngle(pitch);
  kalman_h.setQangle(0.0000085);
  kalman_h.setQbias(0.000005);
  kalman_h.setRmeasure(0.0009);
  timer = micros();

  Serial.println("Robot ready! Type 'help' for commands.");
  Serial.println("Connect via Bluetooth: BalanceBot");
}

// ================== MAIN LOOP ==================
void loop() {
  readmpu();

  // Handle commands from both USB Serial and Bluetooth
  handleSerialCommon(Serial, false);      // USB Serial - không debug
  handleSerialCommon(SerialBT, true);     // Bluetooth - có debug

  if ((micros() - timerloop) > 3000) {  // ~333Hz control loop

    theta = gettheta(leftencoder, righencoder) * ToRad;
    psi   = (mpudata+ angle_offset) * ToRad;
    phi   = getphi(leftencoder, righencoder) * ToRad;

    double dt = (double)(micros() - timerloop) / 1000000.0;
    timerloop = micros();

    thetadot = (theta - thetaold) / dt;
    psidot   = gyroRate;  // use gyro directly (less noise than derivative)
    phidot   = (phi   - phiold)   / dt;

    thetaold = theta;
    phiold   = phi;

    // Cập nhật điều khiển theo mode
    #if MOVE_MODE == 0
      // Mode 0: Góc nghiêng - robot nghiêng để di chuyển
      targetAngle = ForwardBack;
    #elif MOVE_MODE == 2
      // Mode 2: Encoder - di chuyển đến vị trí target
      targetEncoder += (long)ForwardBack;  // Cộng dồn encoder target
      ForwardBack = 0;  // Reset sau khi cộng
    #endif

    // Truyền góc mục tiêu vào LQR
    #if MOVE_MODE == 2
      // Mode 2: Dùng K1 để điều khiển vị trí
      float posError = (float)(leftencoder + righencoder - targetEncoder) * THETA_FACTOR * ToRad;
      getlqr(posError, thetadot, psi, psidot, phi, phidot, 0);
    #else
      getlqr(theta, thetadot, psi, psidot, phi, phidot, targetAngle);
    #endif
    
    // Áp dụng xoay và tiến/lùi trực tiếp vào PWM
    // Mode 1: Giới hạn an toàn - chỉ cộng PWM khi góc còn nhỏ
    int safePWM = ForwardPWM;
    #if MOVE_MODE == 1
      float absAngle = fabs(psi * ToDeg);
      if (absAngle > MAX_SAFE_ANGLE) {
        safePWM = 0;  // Tắt PWM offset khi nghiêng quá nhiều (tránh ngã)
      } else if (absAngle > MAX_SAFE_ANGLE * 0.7f) {
        // Giảm dần PWM khi gần giới hạn
        safePWM = ForwardPWM * (1.0f - (absAngle - MAX_SAFE_ANGLE * 0.7f) / (MAX_SAFE_ANGLE * 0.3f));
      }
    #endif
    
    long pwmL_final = PWML - LeftRight + safePWM;  // +PWM = tiến
    long pwmR_final = PWMR + LeftRight + safePWM;
    
    motorcontrol(pwmL_final, pwmR_final, (mpudata + angle_offset), falldown);

    // Debug logging (toggle with LOG command)
    if (debugLog) {
      logCounter++;
      if (logCounter >= LOG_INTERVAL) {
        logCounter = 0;
        Serial.print("psi:"); Serial.print(psi * ToDeg, 1);
        Serial.print(" target:"); Serial.print(targetAngle, 1);
        Serial.print(" FB:"); Serial.print(ForwardBack, 1);
        Serial.print(" PWM:"); Serial.print(PWML); Serial.print("/"); Serial.println(PWMR);
      }
    }
  }
}

// ================== FUNCTIONS ==================
void readmpu() {
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  Gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]);

  double dt = (double)(micros() - timer) / 1000000.0;
  timer = micros();
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double gyrorate = Gyro / 131.0;

  mpudata = kalman_h.getAngle(pitch, gyrorate, dt);
  gyroRate = gyrorate * ToRad;  // convert deg/s to rad/s for LQR
  falldown = (abs(mpudata) > FALL_ANGLE);
}

float gettheta(long lencoder, long rencoder) {
  return THETA_FACTOR * (lencoder + rencoder);
}

float getphi(long lencoder, long rencoder) {
  return PHI_FACTOR * (lencoder - rencoder);
}

void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_, float targetAngle_) {
  // State feedback: u = -K * x
  // targetAngle_: góc nghiêng mục tiêu (độ) - robot sẽ cố đạt góc này
  // Khi targetAngle > 0: robot nghiêng về trước -> đi tới
  // Khi targetAngle < 0: robot nghiêng về sau -> đi lùi
  
  // Chuyển targetAngle từ độ sang rad và trừ vào psi
  // Robot sẽ "nghĩ" nó đang nghiêng sai hướng và tự điều chỉnh
  float psiError = psi_ - (targetAngle_ * ToRad);
  
  float common = K1 * theta_ + K2 * thetadot_ + K3 * psiError + K4 * psidot_;
  float diff   = K5 * phi_ + K6 * phidot_;
  
  float leftvolt  = common - diff;
  float righvolt  = common + diff;

  // Direct linear mapping: output is already in "voltage" units
  // Clamp to PWM range without dynamic scaling
  PWML = (long)constrain(leftvolt, -255.0f, 255.0f);
  PWMR = (long)constrain(righvolt, -255.0f, 255.0f);
}

void motorcontrol(long lpwm, long rpwm, float angle, bool stopstate) {
  // Chỉ dừng khi ngã hoàn toàn (>30 độ) - BỎ dead zone 5 độ!
  if (stopstate) { stopandreset(); return; }

  // Áp dụng motor trim
  lpwm += motorTrim;

  // Deadband: nếu PWM quá nhỏ, không chạy motor (tránh jitter)
  if (abs(lpwm) < PWM_DEADBAND) lpwm = 0;
  if (abs(rpwm) < PWM_DEADBAND) rpwm = 0;

  // Nếu PWM != 0 nhưng < min_pwm, boost lên min_pwm
  if (lpwm != 0 && abs(lpwm) < min_pwm_L) lpwm = (lpwm > 0) ? min_pwm_L : -min_pwm_L;
  if (rpwm != 0 && abs(rpwm) < min_pwm_R) rpwm = (rpwm > 0) ? min_pwm_R : -min_pwm_R;

  // ... phần còn lại giữ nguyên

  analogWrite(ENA, abs(lpwm));
  digitalWrite(IN1, lpwm > 0 ? LOW : HIGH);
  digitalWrite(IN2, lpwm > 0 ? HIGH : LOW);

  analogWrite(ENB, abs(rpwm));
  digitalWrite(IN3, rpwm > 0 ? LOW : HIGH);
  digitalWrite(IN4, rpwm > 0 ? HIGH : LOW);
}

void stopandreset() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

  leftencoder = 0;
  righencoder = 0;
  targetAngle = 0;
  ForwardBack = 0;
  LeftRight = 0;
  ForwardPWM = 0;
  targetEncoder = 0;
}

// ================== SERIAL/BLUETOOTH COMMAND HANDLER ==================
void printCurrentK(Stream &port) {
  port.println("=== Current LQR Gains ===");
  port.print("K1: "); port.println(K1, 3);
  port.print("K2: "); port.println(K2, 3);
  port.print("K3: "); port.println(K3, 3);
  port.print("K4: "); port.println(K4, 3);
  port.print("K5: "); port.println(K5, 3);
  port.print("K6: "); port.println(K6, 3);
  port.println("========================");
}

// Buffer cho Bluetooth (tránh blocking)
char btBuffer[32];
int btBufferIndex = 0;
bool debugBT = true;  // Bật/tắt debug Bluetooth

// Xử lý Serial chung
void handleSerialCommon(Stream &port, bool isBluetooth) {
  // Kiểm tra timeout buffer - clear nếu quá lâu không có newline
  if (btBufferIndex > 0 && (millis() - lastBufferTime > BUFFER_TIMEOUT_MS)) {
    if (debugBT) Serial.println("[BT] Buffer timeout, clearing");
    btBufferIndex = 0;
  }
  
  while (port.available() > 0) {
    char c = port.read();
    
    // Debug: in ra ký tự nhận được (chỉ khi từ Bluetooth)
    if (isBluetooth && debugBT) {
      Serial.print("[BT RX]: '"); Serial.print(c); 
      Serial.print("' (0x"); Serial.print((int)c, HEX); Serial.println(")");
    }
    
    // Chuyển sang uppercase để so sánh
    char upperC = toupper(c);
    
    // Điều khiển di chuyển - CHỈ qua Bluetooth
    // LUÔN xử lý lệnh điều khiển, không phụ thuộc buffer
    if (isBluetooth) {
      bool cmdHandled = false;
      
      if (upperC == 'F') { 
        #if MOVE_MODE == 1
          ForwardPWM = PWM_MODE_SPEED;
        #else
          ForwardBack = SPEED_FORWARD;
        #endif
        lastCmdTime = millis(); cmdActive = true;
        Serial.println(">>> CMD: FORWARD <<<");
        cmdHandled = true;
      }
      else if (upperC == 'B') { 
        #if MOVE_MODE == 1
          ForwardPWM = -PWM_MODE_SPEED;
        #else
          ForwardBack = -SPEED_BACKWARD;
        #endif
        lastCmdTime = millis(); cmdActive = true;
        Serial.println(">>> CMD: BACKWARD <<<");
        cmdHandled = true;
      }
      else if (upperC == 'L') { 
        LeftRight = SPEED_LEFT; 
        lastCmdTime = millis(); cmdActive = true;
        Serial.println(">>> CMD: LEFT <<<");
        cmdHandled = true;
      }
      else if (upperC == 'R') { 
        LeftRight = -SPEED_RIGHT; 
        lastCmdTime = millis(); cmdActive = true;
        Serial.println(">>> CMD: RIGHT <<<");
        cmdHandled = true;
      }
      else if (upperC == 'S') { 
        ForwardBack = 0; LeftRight = 0; targetAngle = 0;
        ForwardPWM = 0; targetEncoder = leftencoder + righencoder;
        cmdActive = false;
        Serial.println(">>> CMD: STOP <<<");
        cmdHandled = true;
      }
      // Số 1-8 cho điều khiển
      else if (c >= '1' && c <= '8') {
        switch (c) {
          case '1': 
            #if MOVE_MODE == 1
              ForwardPWM = PWM_MODE_SPEED;
            #else
              ForwardBack = SPEED_FORWARD;
            #endif
            lastCmdTime = millis(); cmdActive = true;
            Serial.println(">>> CMD: 1-FWD <<<"); 
            break;
          case '3': 
            #if MOVE_MODE == 1
              ForwardPWM = -PWM_MODE_SPEED;
            #else
              ForwardBack = -SPEED_BACKWARD;
            #endif
            lastCmdTime = millis(); cmdActive = true;
            Serial.println(">>> CMD: 3-BWD <<<"); 
            break;
          case '2': case '4': 
            ForwardBack = 0; ForwardPWM = 0; 
            cmdActive = false;
            break;
          case '5': 
            LeftRight = SPEED_LEFT; 
            lastCmdTime = millis(); cmdActive = true;
            Serial.println(">>> CMD: 5-LEFT <<<"); 
            break;
          case '7': 
            LeftRight = -SPEED_RIGHT; 
            lastCmdTime = millis(); cmdActive = true;
            Serial.println(">>> CMD: 7-RIGHT <<<"); 
            break;
          case '6': case '8': 
            LeftRight = 0; 
            break;
        }
        cmdHandled = true;
      }
      
      if (cmdHandled) {
        btBufferIndex = 0;  // Clear buffer khi có lệnh điều khiển
        continue;  // Tiếp tục đọc ký tự tiếp theo
      }
    }
    
    // Nếu là newline hoặc carriage return, xử lý lệnh dài
    if (c == '\n' || c == '\r') {
      if (btBufferIndex > 0) {
        btBuffer[btBufferIndex] = '\0';
        processCommandSerial(btBuffer);
        btBufferIndex = 0;
      }
      continue;
    }
    
    // Thêm vào buffer (cho lệnh dài như K3=800, LOG)
    if (btBufferIndex < 31) {
      btBuffer[btBufferIndex++] = c;
      lastBufferTime = millis();
    }
  }
}

void processCommandSerial(const char* input) {
  String cmd = String(input);
  cmd.trim();
  if (cmd.length() == 0) return;
  
  String upperCmd = cmd;
  upperCmd.toUpperCase();

  // LQR Gain tuning
  if (upperCmd.startsWith("K1=")) { K1 = cmd.substring(3).toFloat(); Serial.println("K1=" + String(K1, 3)); return; }
  if (upperCmd.startsWith("K2=")) { K2 = cmd.substring(3).toFloat(); Serial.println("K2=" + String(K2, 3)); return; }
  if (upperCmd.startsWith("K3=")) { K3 = cmd.substring(3).toFloat(); Serial.println("K3=" + String(K3, 3)); return; }
  if (upperCmd.startsWith("K4=")) { K4 = cmd.substring(3).toFloat(); Serial.println("K4=" + String(K4, 3)); return; }
  if (upperCmd.startsWith("K5=")) { K5 = cmd.substring(3).toFloat(); Serial.println("K5=" + String(K5, 3)); return; }
  if (upperCmd.startsWith("K6=")) { K6 = cmd.substring(3).toFloat(); Serial.println("K6=" + String(K6, 3)); return; }
  
  if (upperCmd == "K" || upperCmd == "SHOW") { printCurrentK(Serial); return; }
  
  // RESET gains to baseline
  if (upperCmd == "RESET") {
    K1 = K1_BASE; K2 = K2_BASE; K3 = K3_BASE;
    K4 = K4_BASE; K5 = K5_BASE; K6 = K6_BASE;
    Serial.println("Gains RESET to baseline");
    printCurrentK(Serial);
    return;
  }
  
  // Toggle debug logging
  if (upperCmd == "LOG") {
    debugLog = !debugLog;
    Serial.println(debugLog ? "Logging ON" : "Logging OFF");
    return;
  }
  
  // Toggle BT debug
  if (upperCmd == "BTDEBUG") {
    debugBT = !debugBT;
    Serial.println(debugBT ? "BT Debug ON" : "BT Debug OFF");
    return;
  }
  
  // Motor calibration
  if (upperCmd.startsWith("MINL=")) { min_pwm_L = cmd.substring(5).toInt(); Serial.println("MINL=" + String(min_pwm_L)); return; }
  if (upperCmd.startsWith("MINR=")) { min_pwm_R = cmd.substring(5).toInt(); Serial.println("MINR=" + String(min_pwm_R)); return; }
  if (upperCmd.startsWith("TRIM=")) { motorTrim = cmd.substring(5).toInt(); Serial.println("TRIM=" + String(motorTrim)); return; }
  if (upperCmd.startsWith("OFFSET=")) { angle_offset = cmd.substring(7).toFloat(); Serial.println("OFFSET=" + String(angle_offset, 2)); return; }
  
  // Status
  if (upperCmd == "ENC?") {
    Serial.print("L="); Serial.print(leftencoder);
    Serial.print(" R="); Serial.print(righencoder);
    Serial.print(" PWM:"); Serial.print(PWML); Serial.print("/"); Serial.println(PWMR);
    return;
  }
  if (upperCmd == "ENC=0") { leftencoder = 0; righencoder = 0; Serial.println("Enc reset"); return; }
  if (upperCmd == "STATUS" || upperCmd == "?") {
    Serial.print("Angle:"); Serial.print(mpudata + angle_offset, 1);
    Serial.print(" PWM:"); Serial.print(PWML); Serial.print("/"); Serial.println(PWMR);
    return;
  }
  
  if (upperCmd == "HELP") {
    Serial.println("=== Commands ===");
    Serial.println("F/B/L/R/S - Move/Stop");
    Serial.println("K1=..K6= - Set gains");
    Serial.println("K - Show gains");
    Serial.println("RESET - Restore baseline");
    Serial.println("LOG - Toggle debug");
    Serial.println("BTDEBUG - Toggle BT debug");
    Serial.println("MINL=/MINR=/TRIM=/OFFSET=");
    Serial.println("ENC?/ENC=0/?");
    Serial.println("================");
    return;
  }
}