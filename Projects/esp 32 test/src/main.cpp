#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <DFRobot_QMC5883.h>
DFRobot_QMC5883 compass(&Wire, 0x0D);
Servo servoFL,servoFR,servo,bL,servoBR;
// Replace with your network credentials
const char* ssid = "Nilaj's Moto";
const char* password = "12345678";

WebServer server(80);
MPU6050 mpu;

float rateroll, ratepitch, rateyaw;
float angleroll, anglepitch,angleyaw;
float ax, ay, az;
float mx,my,mz; 
float alpha = 0.9;
float ax_raw, ay_raw, az_raw;
float ar, ap;
// Quaternion state estimate
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// Filter gain (adjustable)
const float beta = 0.1f; 
unsigned long last_time = 0;

struct CalibrationData {
    int16_t minX, maxX, minY, maxY, minZ, maxZ;
    bool isCalibrated;
    unsigned long calibrationStart;
    int sampleCount;
} calibData;

const unsigned long CALIBRATION_TIME_MS = 15000;  // 15 seconds calibration
const int MIN_SAMPLES = 500;  // Minimum samples for good calibration

void initializeCalibration() {
    calibData.minX = calibData.minY = calibData.minZ = 32767;
    calibData.maxX = calibData.maxY = calibData.maxZ = -32768;
    calibData.isCalibrated = false;
    calibData.calibrationStart = 0;
    calibData.sampleCount = 0;
}

void calibrateMagnetometer() {
    if (calibData.calibrationStart == 0) {
        calibData.calibrationStart = millis();
        Serial.println("Starting magnetometer calibration...");
        Serial.println("Rotate the sensor in all directions for 15 seconds!");
    }
    
    // Read raw magnetometer data
    sVector_t mag = compass.readRaw();
    
    if (mag.XAxis != 0 || mag.YAxis != 0 || mag.ZAxis != 0) {  // Valid reading
        // Update min/max values
        if (mag.XAxis < calibData.minX) calibData.minX = mag.XAxis;
        if (mag.XAxis > calibData.maxX) calibData.maxX = mag.XAxis;
        if (mag.YAxis < calibData.minY) calibData.minY = mag.YAxis;
        if (mag.YAxis > calibData.maxY) calibData.maxY = mag.YAxis;
        if (mag.ZAxis < calibData.minZ) calibData.minZ = mag.ZAxis;
        if (mag.ZAxis > calibData.maxZ) calibData.maxZ = mag.ZAxis;
        
        calibData.sampleCount++;
        
        // Print progress every 2 seconds
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 2000) {
            int remaining = (CALIBRATION_TIME_MS - (millis() - calibData.calibrationStart)) / 1000;
            Serial.print("Calibrating... ");
            Serial.print(remaining);
            Serial.print(" seconds remaining, samples: ");
            Serial.println(calibData.sampleCount);
            lastPrint = millis();
        }
    }
     if (millis() - calibData.calibrationStart >= CALIBRATION_TIME_MS && 
        calibData.sampleCount >= MIN_SAMPLES) {
        
        calibData.isCalibrated = true;
        Serial.println("\nCalibration complete!");
        Serial.print("X: "); Serial.print(calibData.minX); Serial.print(" to "); Serial.println(calibData.maxX);
        Serial.print("Y: "); Serial.print(calibData.minY); Serial.print(" to "); Serial.println(calibData.maxY);
        Serial.print("Z: "); Serial.print(calibData.minZ); Serial.print(" to "); Serial.println(calibData.maxZ);
        Serial.print("Total samples: "); Serial.println(calibData.sampleCount);
    }
}

void readMagnetometer() {
    if (!calibData.isCalibrated) {
        mx = my = mz = 0;  // Don't use uncalibrated data
        return;
    }
    
    // Read raw magnetometer data using DFRobot library
    sVector_t mag = compass.readRaw();
    
    // Apply hard iron correction (offset calibration)
    float calibrated_x = mag.XAxis - (calibData.maxX + calibData.minX) / 2.0;
    float calibrated_y = mag.YAxis - (calibData.maxY + calibData.minY) / 2.0;
    float calibrated_z = mag.ZAxis - (calibData.maxZ + calibData.minZ) / 2.0;
    
    // Normalize the magnetometer readings
    float mag_norm = sqrt(calibrated_x * calibrated_x + calibrated_y * calibrated_y + calibrated_z * calibrated_z);
    if (mag_norm > 0) {
        mx = calibrated_x / mag_norm;
        my = calibrated_y / mag_norm;
        mz = calibrated_z / mag_norm;
    } else {
        mx = my = mz = 0;
    }
}

void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz, float dt) {
  float norm;
  float hx, hy, _2bx, _2bz;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  // Normalize accelerometer
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Only use magnetometer if it's calibrated and has valid data
  bool useMag = (mx != 0 || my != 0 || mz != 0) && calibData.isCalibrated;
  
  if (useMag) {
    // Normalize magnetometer
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) {
      useMag = false;
    } else {
      mx /= norm;
      my /= norm;
      mz /= norm;
    }
  }

  if (useMag) {
    // Reference direction of Earth's magnetic field
    float _2q0mx = 2.0f * q0 * mx;
    float _2q0my = 2.0f * q0 * my;
    float _2q0mz = 2.0f * q0 * mz;
    float _2q1mx = 2.0f * q1 * mx;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Auxiliary variables
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;

    // Gradient descent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
         + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + (_2bx * q3 - 4.0f * _2bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
         + (-4.0f * _2bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + (_2bx * q0 - 4.0f * _2bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
         + (-4.0f * _2bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
  } 

  // Normalize step magnitude
  norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  if (norm > 0) {
    s0 /= norm;
    s1 /= norm;
    s2 /= norm;
    s3 /= norm;
  }

  // Convert degrees per second to radians per second
  gx *= PI / 180.0f;
  gy *= PI / 180.0f;
  gz *= PI / 180.0f;

  // Integrate rate of change of quaternion
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion
  norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= norm;
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;

  // Convert quaternion to Euler angles
  angleroll  = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*q1*q1 - 2*q2*q2) * 180.0 / PI;
  anglepitch = -1*asin(2*q0*q2 - 2*q3*q1) * 180.0 / PI;
  angleyaw   = atan2(2*q0*q3 + 2*q1*q2, 1 - 2*q2*q2 - 2*q3*q3) * 180.0 / PI;
}

void gyrosignal(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccX = Wire.read() << 8 | Wire.read();
  int16_t AccY = Wire.read() << 8 | Wire.read();
  int16_t AccZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  rateroll = (float)GyroX / 65.5;
  ratepitch = (float)GyroY / 65.5;
  rateyaw = (float)GyroZ / 65.5;

  ax_raw = (float)AccX / 4096.0 - 0.02;
  ay_raw = (float)AccY / 4096.0 + 0.02;
  az_raw = (float)AccZ / 4096.0 - 0.03;

  ax = alpha * ax + (1 - alpha) * ax_raw;
  ay = alpha * ay + (1 - alpha) * ay_raw;
  az = alpha * az + (1 - alpha) * az_raw;

  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;
  madgwickUpdate(rateroll,ratepitch,rateyaw,ax,ay,az,mx,my,mz,dt);
}

void sendOrientation() {
  gyrosignal();
  String response = "PITCH:" + String(anglepitch, 2) + ",ROLL:" + String(angleroll, 2);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", response);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  // Initialize compass with DFRobot library
  while (!compass.begin()) {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  
  // Configure compass based on detected sensor type
  if (compass.isQMC()) {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS);
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  } else if (compass.isHMC()) {
    Serial.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  } else if (compass.isVCM()) {
    Serial.println("Initialize VCM5883L");
    compass.setMeasurementMode(VCM5883L_CONTINOUS);
    compass.setDataRate(VCM5883L_DATARATE_200HZ);
  }

  // Initialize MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Initialize calibration
  initializeCalibration();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi. IP Address: ");
  Serial.println(WiFi.localIP());

  // Start web server
  server.on("/", sendOrientation);
  server.begin();
  Serial.println("Server started.");
  
  // Initialize timing
  last_time = millis();
}

void loop() {
  // Handle magnetometer calibration first
  if (!calibData.isCalibrated) {
    calibrateMagnetometer();
    delay(20);  // Small delay during calibration
    return;
  }
  
  // Read sensors and update orientation
  readMagnetometer();
  gyrosignal();
  
  // Handle web server
  server.handleClient();
  
  // Print orientation data
  Serial.print("Roll: ");
  Serial.print(angleroll, 2);
  Serial.print("°  Pitch: ");
  Serial.print(anglepitch, 2);
  Serial.print("°  Yaw: ");
  Serial.print(angleyaw, 2);
  Serial.println("°");
  
  delay(1);  // Small delay for stability
}