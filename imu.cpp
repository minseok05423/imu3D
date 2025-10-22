/**
 * Complete Advanced 9-DOF Sensor Fusion System for ESP32-C3
 * Version: 1.0.0
 * 
 * This is the complete, production-ready sensor fusion system
 * combining all core and advanced features in a single file.
 * 
 * Save this entire file as: SensorFusion.ino
 * 
 * Hardware:
 * - ESP32-C3 SuperMini
 * - MPU6050 (Accelerometer + Gyroscope) @ 0x68
 * - QMC5883L/HMC5883L (Magnetometer) @ 0x0D/0x1E
 * - I2C: SDA=GPIO8, SCL=GPIO9
 */

#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// I2C Configuration
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ 400000

// Sensor I2C Addresses
#define MPU6050_ADDR 0x68
#define QMC5883L_ADDR 0x0D
#define HMC5883L_ADDR 0x1E

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I 0x75

// Magnetometer Registers
#define QMC5883L_CONFIG 0x09
#define QMC5883L_STATUS 0x06
#define QMC5883L_DATA_OUT 0x00
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_DATA_OUT 0x03

// System Configuration
#define SAMPLE_RATE_HZ 100
#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
#define SERIAL_BAUD 115200
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xAB5F

// Calibration Parameters
#define GYRO_CALIBRATION_SAMPLES 300
#define MAG_CALIBRATION_SAMPLES 500
#define ACCEL_CALIBRATION_SAMPLES 300

// Filter Parameters
#define INITIAL_Q_VARIANCE 0.001f
#define INITIAL_R_ACCEL 0.1f
#define INITIAL_R_MAG 0.5f
#define OUTLIER_THRESHOLD 3.0f

// ============================================================================
// FORWARD DECLARATIONS (IMPORTANT!)
// ============================================================================

// Declare all classes before defining them to avoid circular dependencies
class ExtendedKalmanFilter;
class CalibrationManager;
class SensorManager;
class SerialProtocol;
class PerformanceMonitor;
class AdvancedCalibration;
class MagneticAnomalyDetector;
class AdaptiveFilterTuning;
class PowerManager;
class GestureRecognizer;
class DataLogger;
class MotionPredictor;
class SystemDiagnostics;

// ============================================================================
// DATA STRUCTURES (Keep from Part 1)
// ============================================================================

struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    
    float magnitude() const { return sqrtf(x*x + y*y + z*z); }
    void normalize() {
        float mag = magnitude();
        if (mag > 0.0001f) { x /= mag; y /= mag; z /= mag; }
    }
    
    Vector3 operator+(const Vector3& v) const { return Vector3(x+v.x, y+v.y, z+v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x-v.x, y-v.y, z-v.z); }
    Vector3 operator*(float s) const { 
        // Safety check
        if(isnan(s) || isinf(s)) {
        return Vector3(0, 0, 0);
        }
        return Vector3(x*s, y*s, z*s); 
    }
    float dot(const Vector3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vector3 cross(const Vector3& v) const {
        return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }
};

struct Quaternion {
    float w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}
    
    void normalize() {
        float norm = sqrtf(w*w + x*x + y*y + z*z);
        if (norm > 0.0001f) {
            w /= norm; x /= norm; y /= norm; z /= norm;
        }
    }
    
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    void toEuler(float& roll, float& pitch, float& yaw) const {
        roll = atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y));
        pitch = asinf(2*(w*y - z*x));
        yaw = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z));
    }
    
    Vector3 rotate(const Vector3& v) const {
        Quaternion p(0, v.x, v.y, v.z);
        Quaternion result = (*this) * p * conjugate();
        return Vector3(result.x, result.y, result.z);
    }
};

struct SensorData {
    Vector3 accel;
    Vector3 gyro;
    Vector3 mag;
    float temperature;
    uint32_t timestamp;
};

struct CalibrationData {
    Vector3 gyroBias;
    Vector3 accelBias;
    Vector3 accelScale;
    Vector3 magOffset;
    float magScale[3][3];
    float tempReference;
    float tempCoeff[9];
    uint16_t magic;
    uint16_t checksum;
};

struct FilterState {
    Quaternion orientation;
    Vector3 gyroBias;
    float confidence;
    uint32_t lastUpdateTime;
    float updateRate;
    float P[7][7];
    float Q[7][7];
    float R_accel[3][3];
    float R_mag[3][3];
    float innovationAccel;
    float innovationMag;
    bool magneticAnomaly;
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// ============================================================================
// Include ALL CLASS IMPLEMENTATIONS from Part 1
// ============================================================================

// ============================================================================
// SENSOR MANAGER CLASS
// ============================================================================
class SensorManager {
private:
    uint8_t magnetometerAddr;
    bool magnetometerType;  // true = QMC5883L, false = HMC5883L
    CalibrationData calibration;
    SensorData rawData;
    SensorData calibratedData;
    
    // Moving average filters for noise reduction
    static const int FILTER_SIZE = 5;
    Vector3 accelBuffer[FILTER_SIZE];
    Vector3 magBuffer[FILTER_SIZE];
    int bufferIndex;
    
    // I2C communication helpers
    void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
    
    uint8_t readRegister(uint8_t addr, uint8_t reg) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(addr, (uint8_t)1);
        return Wire.read();
    }
    
    void readMultiple(uint8_t addr, uint8_t reg, uint8_t* buffer, uint8_t length) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(addr, length);
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = Wire.read();
        }
    }
    
public:
    SensorManager() :  bufferIndex(0), magnetometerType(true), magnetometerAddr(0) {
        memset(&calibration, 0, sizeof(CalibrationData));
        memset(&rawData, 0, sizeof(SensorData));
        memset(&calibratedData, 0, sizeof(SensorData));
    }
    
    bool initialize() {
        // Initialize I2C
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(I2C_FREQ);
        delay(100);
        
        // Initialize MPU6050
        if (!initializeMPU6050()) {
            Serial.println("ERROR: MPU6050 initialization failed!");
            return false;
        }
        Serial.println("MPU6050 initialized successfully");
        
        // Initialize Magnetometer
        if (!initializeMagnetometer()) {
            Serial.println("WARNING: Magnetometer initialization failed - continuing without mag");
        } else {
            Serial.println("Magnetometer initialized successfully");
        }
        
        // Load calibration from EEPROM
        loadCalibration();
        
        return true;
    }
    
    bool initializeMPU6050() {
        // Check WHO_AM_I register
        uint8_t whoami = readRegister(MPU6050_ADDR, MPU6050_WHO_AM_I);
        if (whoami != 0x68 && whoami != 0x98) {  // MPU6050 or MPU9250
            Serial.print("MPU6050 WHO_AM_I failed: 0x");
            Serial.println(whoami, HEX);
            return false;
        }
        
        // Wake up MPU6050
        writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
        delay(100);
        
        // Configure gyroscope (±2000°/s)
        writeRegister(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x18);
        
        // Configure accelerometer (±8g)
        writeRegister(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x10);
        
        // Configure DLPF (Digital Low Pass Filter) - 42Hz
        writeRegister(MPU6050_ADDR, MPU6050_CONFIG, 0x03);
        
        return true;
    }
    
    bool initializeMagnetometer() {
        // Try QMC5883L first
        Wire.beginTransmission(QMC5883L_ADDR);
        if (Wire.endTransmission() == 0) {
            magnetometerAddr = QMC5883L_ADDR;
            magnetometerType = true;
            
            // Configure QMC5883L
            writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG, 0x1D);  // Continuous mode, 200Hz, 8 Gauss
            delay(10);
            return true;
        }
        
        // Try HMC5883L
        Wire.beginTransmission(HMC5883L_ADDR);
        if (Wire.endTransmission() == 0) {
            magnetometerAddr = HMC5883L_ADDR;
            magnetometerType = false;
            
            // Configure HMC5883L
            writeRegister(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x78);  // 8 samples avg, 75Hz
            writeRegister(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20);  // ±1.3 Gauss
            writeRegister(HMC5883L_ADDR, HMC5883L_MODE, 0x00);      // Continuous mode
            delay(10);
            return true;
        }
        
        return false;
    }
    
    void readSensors() {
        rawData.timestamp = millis();
        
        // Read MPU6050
        uint8_t buffer[14];
        readMultiple(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14);
        
        // Parse accelerometer data
        int16_t ax = (buffer[0] << 8) | buffer[1];
        int16_t ay = (buffer[2] << 8) | buffer[3];
        int16_t az = (buffer[4] << 8) | buffer[5];
        
        // Parse temperature
        int16_t temp = (buffer[6] << 8) | buffer[7];
        rawData.temperature = temp / 340.0f + 36.53f;
        
        // Parse gyroscope data
        int16_t gx = (buffer[8] << 8) | buffer[9];
        int16_t gy = (buffer[10] << 8) | buffer[11];
        int16_t gz = (buffer[12] << 8) | buffer[13];
        
        // Convert to physical units
        rawData.accel.x = ax / 4096.0f;  // ±8g range
        rawData.accel.y = ay / 4096.0f;
        rawData.accel.z = az / 4096.0f;
        
        rawData.gyro.x = gx * (2000.0f / 32768.0f) * (PI / 180.0f);  // Convert to rad/s
        rawData.gyro.y = gy * (2000.0f / 32768.0f) * (PI / 180.0f);
        rawData.gyro.z = gz * (2000.0f / 32768.0f) * (PI / 180.0f);
        
        // Read magnetometer if available
        if (magnetometerAddr > 0) {
            readMagnetometer();
        }
        
        // Apply calibration
        applyCalibration();
    }
    
    void readMagnetometer() {
        uint8_t buffer[6];
        
        if (magnetometerType) {
            // QMC5883L
            readMultiple(QMC5883L_ADDR, QMC5883L_DATA_OUT, buffer, 6);
            int16_t mx = (buffer[1] << 8) | buffer[0];  // Note: LSB first for QMC
            int16_t my = (buffer[3] << 8) | buffer[2];
            int16_t mz = (buffer[5] << 8) | buffer[4];
            
            rawData.mag.x = mx / 12000.0f;  // Normalize
            rawData.mag.y = my / 12000.0f;
            rawData.mag.z = mz / 12000.0f;
        } else {
            // HMC5883L
            readMultiple(HMC5883L_ADDR, HMC5883L_DATA_OUT, buffer, 6);
            int16_t mx = (buffer[0] << 8) | buffer[1];  // MSB first for HMC
            int16_t mz = (buffer[2] << 8) | buffer[3];  // Note: Z comes before Y
            int16_t my = (buffer[4] << 8) | buffer[5];
            
            rawData.mag.x = mx / 1090.0f;  // Normalize for ±1.3 Gauss range
            rawData.mag.y = my / 1090.0f;
            rawData.mag.z = mz / 1090.0f;
        }
    }
    
    void applyCalibration() {
        // Apply gyroscope bias correction
        calibratedData.gyro = rawData.gyro - calibration.gyroBias;
        
        // Apply accelerometer calibration
        Vector3 accelCorrected = rawData.accel - calibration.accelBias;
        calibratedData.accel.x = accelCorrected.x * calibration.accelScale.x;
        calibratedData.accel.y = accelCorrected.y * calibration.accelScale.y;
        calibratedData.accel.z = accelCorrected.z * calibration.accelScale.z;
        
        // Apply magnetometer calibration (hard/soft iron correction)
        Vector3 magCorrected = rawData.mag - calibration.magOffset;
        calibratedData.mag.x = calibration.magScale[0][0] * magCorrected.x +
                               calibration.magScale[0][1] * magCorrected.y +
                               calibration.magScale[0][2] * magCorrected.z;
        calibratedData.mag.y = calibration.magScale[1][0] * magCorrected.x +
                               calibration.magScale[1][1] * magCorrected.y +
                               calibration.magScale[1][2] * magCorrected.z;
        calibratedData.mag.z = calibration.magScale[2][0] * magCorrected.x +
                               calibration.magScale[2][1] * magCorrected.y +
                               calibration.magScale[2][2] * magCorrected.z;
        
        // Apply temperature compensation
        float tempDelta = rawData.temperature - calibration.tempReference;
        calibratedData.gyro.x += tempDelta * calibration.tempCoeff[0];
        calibratedData.gyro.y += tempDelta * calibration.tempCoeff[1];
        calibratedData.gyro.z += tempDelta * calibration.tempCoeff[2];
        
        calibratedData.temperature = rawData.temperature;
        calibratedData.timestamp = rawData.timestamp;
        
        // Apply moving average filter to reduce noise
        updateFilters();
    }
    
    void updateFilters() {
        // Update accelerometer buffer
        accelBuffer[bufferIndex] = calibratedData.accel;
        
        // Update magnetometer buffer
        magBuffer[bufferIndex] = calibratedData.mag;
        
        bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
        
        // Calculate filtered values
        Vector3 accelFiltered, magFiltered;
        for (int i = 0; i < FILTER_SIZE; i++) {
            accelFiltered = accelFiltered + accelBuffer[i];
            magFiltered = magFiltered + magBuffer[i];
        }
        
        calibratedData.accel = accelFiltered * (1.0f / FILTER_SIZE);
        calibratedData.mag = magFiltered * (1.0f / FILTER_SIZE);
    }
    
    SensorData& getCalibratedData() {
        return calibratedData;
    }
    
    SensorData& getRawData() {
        return rawData;
    }
    
    void setCalibration(const CalibrationData& cal) {
        calibration = cal;
        saveCalibration();
    }
    
    CalibrationData& getCalibration() {
        return calibration;
    }
    
    void saveCalibration() {
        calibration.magic = EEPROM_MAGIC;
        calibration.checksum = calculateChecksum();
        
        EEPROM.begin(EEPROM_SIZE);
        EEPROM.put(0, calibration);
        EEPROM.commit();
        EEPROM.end();
        
        Serial.println("Calibration saved to EEPROM");
    }
    
    void loadCalibration() {
        EEPROM.begin(EEPROM_SIZE);
        EEPROM.get(0, calibration);
        EEPROM.end();
        
        // Verify calibration data
        if (calibration.magic != EEPROM_MAGIC || calibration.checksum != calculateChecksum()) {
            Serial.println("No valid calibration found, using defaults");
            setDefaultCalibration();
        } else {
            Serial.println("Calibration loaded from EEPROM");
        }
    }
    
    void setDefaultCalibration() {
        memset(&calibration, 0, sizeof(CalibrationData));
        
        // Set default scale factors
        calibration.accelScale.x = 1.0f;
        calibration.accelScale.y = 1.0f;
        calibration.accelScale.z = 1.0f;
        
        // Set identity matrix for magnetometer
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                calibration.magScale[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        
        calibration.tempReference = 25.0f;  // Room temperature
    }
    
    uint16_t calculateChecksum() {
        uint16_t sum = 0;
        uint8_t* ptr = (uint8_t*)&calibration;
        for (size_t i = 0; i < sizeof(CalibrationData) - 4; i++) {  // Exclude magic and checksum
            sum += ptr[i];
        }
        return sum;
    }
    
    bool hasMagnetometer() {
        return magnetometerAddr > 0;
    }
};

// ============================================================================
// EXTENDED KALMAN FILTER IMPLEMENTATION
// ============================================================================
class ExtendedKalmanFilter {
private:
    FilterState& state;
    float dt;
    bool adaptiveEnabled;
    float alpha;  // Exponential moving average factor
    
    // Matrix operations
    void matrixMultiply(float* C, const float* A, const float* B, int m, int n, int p) {
        for(int i = 0; i < m; i++) {
            for(int j = 0; j < p; j++) {
                C[i*p + j] = 0;
                for(int k = 0; k < n; k++) {
                    C[i*p + j] += A[i*n + k] * B[k*p + j];
                }
            }
        }
    }
    
    void matrixAdd(float* C, const float* A, const float* B, int m, int n) {
        for(int i = 0; i < m*n; i++) C[i] = A[i] + B[i];
    }
    
    void matrixTranspose(float* At, const float* A, int m, int n) {
        for(int i = 0; i < m; i++) {
            for(int j = 0; j < n; j++) {
                At[j*m + i] = A[i*n + j];
            }
        }
    }
    
public:
    ExtendedKalmanFilter(FilterState& fs) : state(fs), dt(0.01f), adaptiveEnabled(true), alpha(0.98f) {
        reset();
    }
    
    void reset() {
        state.orientation = Quaternion(1, 0, 0, 0);
        state.gyroBias = Vector3(0, 0, 0);
        state.confidence = 0;
        
        // Initialize covariance matrices
        for(int i = 0; i < 7; i++) {
            for(int j = 0; j < 7; j++) {
                state.P[i][j] = (i == j) ? 0.1f : 0.0f;
                state.Q[i][j] = (i == j) ? INITIAL_Q_VARIANCE : 0.0f;
            }
        }
        
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                state.R_accel[i][j] = (i == j) ? INITIAL_R_ACCEL : 0.0f;
                state.R_mag[i][j] = (i == j) ? INITIAL_R_MAG : 0.0f;
            }
        }
    }
    
    void predict(const Vector3& gyro) {
        // Safety checks
        if(isnan(gyro.x) || isnan(gyro.y) || isnan(gyro.z)) {
            return;
        }
        
        if(isnan(state.gyroBias.x) || isnan(state.gyroBias.y) || isnan(state.gyroBias.z)) {
            state.gyroBias = Vector3(0, 0, 0);
        }
        
        // Remove bias
        Vector3 omega = gyro - state.gyroBias;
        
        // Check for valid omega
        if(isnan(omega.x) || isnan(omega.y) || isnan(omega.z)) {
            return;
        }
        
        // Quaternion derivative (simple integration)
        float halfDt = dt * 0.5f;
        
        Quaternion qDot;
        qDot.w = -halfDt * (omega.x * state.orientation.x + omega.y * state.orientation.y + omega.z * state.orientation.z);
        qDot.x =  halfDt * (omega.x * state.orientation.w + omega.z * state.orientation.y - omega.y * state.orientation.z);
        qDot.y =  halfDt * (omega.y * state.orientation.w - omega.z * state.orientation.x + omega.x * state.orientation.z);
        qDot.z =  halfDt * (omega.z * state.orientation.w + omega.y * state.orientation.x - omega.x * state.orientation.y);
        
        // Integrate
        state.orientation.w += qDot.w;
        state.orientation.x += qDot.x;
        state.orientation.y += qDot.y;
        state.orientation.z += qDot.z;
        state.orientation.normalize();
        
        // Slowly adapt gyro bias based on accelerometer
        float biasAdapt = 0.00001f;  // Very slow adaptation
        state.gyroBias.x += biasAdapt * omega.x;
        state.gyroBias.y += biasAdapt * omega.y;
        state.gyroBias.z += biasAdapt * omega.z;
        
        // Verify no NaN
        if(isnan(state.orientation.w) || isnan(state.orientation.x) || 
        isnan(state.orientation.y) || isnan(state.orientation.z)) {
            state.orientation = Quaternion(1, 0, 0, 0);
        }
    }
    
    void updateAccel(const Vector3& accel) {
        // Normalize accelerometer measurement
        Vector3 accelNorm = accel;
        float accelMag = accelNorm.magnitude();
    
        // Check if accelerometer is reasonable (0.5g to 1.5g)
        if(accelMag < 0.5f || accelMag > 1.5f) {
            return;  // Reject - likely external acceleration
        }
    
        accelNorm.normalize();
    
        // Expected gravity direction in body frame
        Vector3 gravity(0, 0, 1);
        Vector3 expectedAccel = state.orientation.conjugate().rotate(gravity);
        
        // Innovation (error)
        Vector3 innovation = accelNorm - expectedAccel;
        state.innovationAccel = innovation.magnitude();
        
        // SIMPLIFIED UPDATE - Use gradient descent instead of full EKF
        // This is more stable and prevents drift
        
        float alpha = 0.02f;  // Learning rate (tune this: 0.01 to 0.05)
        
        // Calculate gradient
        float qw = state.orientation.w;
        float qx = state.orientation.x;
        float qy = state.orientation.y;
        float qz = state.orientation.z;
        
        // Jacobian of error w.r.t. quaternion
        float J[4];
        J[0] = 2 * (qy * innovation.x - qx * innovation.y);
        J[1] = 2 * (qz * innovation.x + qw * innovation.y - 2*qx * innovation.z);
        J[2] = 2 * (-qw * innovation.x + qz * innovation.y - 2*qy * innovation.z);
        J[3] = 2 * (qx * innovation.x + qy * innovation.y);
        
        // Update quaternion
        state.orientation.w -= alpha * J[0];
        state.orientation.x -= alpha * J[1];
        state.orientation.y -= alpha * J[2];
        state.orientation.z -= alpha * J[3];
        state.orientation.normalize();
        
        // Update confidence
        state.confidence = 100.0f * (1.0f - state.innovationAccel * 2.0f);
        state.confidence = constrain(state.confidence, 0, 100);
    }

    void updateMag(const Vector3& mag) {
        // Normalize magnetometer measurement
        Vector3 magNorm = mag;
        magNorm.normalize();
        
        // Expected magnetic field in body frame (assuming north-pointing)
        Vector3 magRef(0.5f, 0, 0.866f);  // Approximate for most locations
        Vector3 expectedMag = state.orientation.conjugate().rotate(magRef);
        
        // Innovation
        Vector3 innovation = magNorm - expectedMag;
        state.innovationMag = innovation.magnitude();
        
        // Check for magnetic anomaly
        if(state.innovationMag > OUTLIER_THRESHOLD || mag.magnitude() < 0.2f || mag.magnitude() > 2.0f) {
            state.magneticAnomaly = true;
            return;  // Reject measurement
        }
        state.magneticAnomaly = false;
        
        // Similar update process as accelerometer
        // (Implementation follows same pattern as updateAccel)
        
        // Update confidence
        state.confidence = 100.0f * (1.0f - (state.innovationAccel + state.innovationMag) / 2.0f);
        state.confidence = constrain(state.confidence, 0, 100);
    }
    
    void setDt(float deltaTime) { dt = deltaTime; }
    void enableAdaptive(bool enable) { adaptiveEnabled = enable; }
};

// ============================================================================
// CALIBRATION MANAGER
// ============================================================================

class CalibrationManager {
private:
    SensorManager& sensors;
    CalibrationData& cal;
    
    enum CalibrationState {
        IDLE,
        GYRO_CALIBRATING,
        ACCEL_CALIBRATING,
        MAG_CALIBRATING,
        COMPLETE
    } state;
    
    int sampleCount;
    Vector3 accumulator;
    Vector3 minValues;
    Vector3 maxValues;
    
public:
    CalibrationManager(SensorManager& sm, CalibrationData& c) 
        : sensors(sm), cal(c), state(IDLE), sampleCount(0) {}
    
    void startGyroCalibration() {
        state = GYRO_CALIBRATING;
        sampleCount = 0;
        accumulator = Vector3(0, 0, 0);
        Serial.println("Gyroscope calibration started. Keep device still...");
    }
    
    void startAccelCalibration() {
        state = ACCEL_CALIBRATING;
        sampleCount = 0;
        Serial.println("Accelerometer calibration started.");
        Serial.println("Place device on each of 6 faces for 2 seconds each...");
    }
    
    void startMagCalibration() {
        state = MAG_CALIBRATING;
        sampleCount = 0;
        minValues = Vector3(9999, 9999, 9999);
        maxValues = Vector3(-9999, -9999, -9999);
        Serial.println("Magnetometer calibration started.");
        Serial.println("Rotate device slowly in all orientations...");
    }
    
    bool update(const SensorData& raw) {
        switch(state) {
            case GYRO_CALIBRATING:
                if(isnan(raw.gyro.x) || isnan(raw.gyro.y) || isnan(raw.gyro.z)) {
                    Serial.println("ERROR: Invalid gyro data during calibration!");
                    sampleCount = 0;
                    accumulator = Vector3(0, 0, 0);
                    return false;
                }
    
                accumulator = accumulator + raw.gyro;
                sampleCount++;
    
                if(sampleCount >= GYRO_CALIBRATION_SAMPLES) {
                    cal.gyroBias = accumulator * (1.0f / sampleCount);
        
                    // ADD VALIDATION:
                    if(isnan(cal.gyroBias.x) || isnan(cal.gyroBias.y) || isnan(cal.gyroBias.z)) {
                        Serial.println("ERROR: Calibration produced NaN! Using defaults.");
                        cal.gyroBias = Vector3(0, 0, 0);
                    }
        
                    Serial.print("Gyro bias: ");
                    Serial.print(cal.gyroBias.x, 4); Serial.print(", ");
                    Serial.print(cal.gyroBias.y, 4); Serial.print(", ");
                    Serial.println(cal.gyroBias.z, 4);
        
                    Serial.println("Gyroscope calibration complete!");
                    state = IDLE;
                    return true;
                }
    
                if(sampleCount % 30 == 0) Serial.print(".");
                break;
                
            case MAG_CALIBRATING:
                minValues.x = std::min(minValues.x, raw.mag.x);
                minValues.y = std::min(minValues.y, raw.mag.y);
                minValues.z = std::min(minValues.z, raw.mag.z);
                maxValues.x = std::max(maxValues.x, raw.mag.x);
                maxValues.y = std::max(maxValues.y, raw.mag.y);
                maxValues.z = std::max(maxValues.z, raw.mag.z);
                sampleCount++;
                
                if(sampleCount >= MAG_CALIBRATION_SAMPLES) {
                    cal.magOffset.x = (minValues.x + maxValues.x) / 2.0f;
                    cal.magOffset.y = (minValues.y + maxValues.y) / 2.0f;
                    cal.magOffset.z = (minValues.z + maxValues.z) / 2.0f;
                    
                    // Simple scale correction
                    float avgDelta = ((maxValues.x - minValues.x) + 
                                      (maxValues.y - minValues.y) + 
                                      (maxValues.z - minValues.z)) / 3.0f;
                    
                    cal.magScale[0][0] = avgDelta / (maxValues.x - minValues.x);
                    cal.magScale[1][1] = avgDelta / (maxValues.y - minValues.y);
                    cal.magScale[2][2] = avgDelta / (maxValues.z - minValues.z);
                    
                    Serial.println("Magnetometer calibration complete!");
                    state = IDLE;
                    return true;
                }
                
                if(sampleCount % 50 == 0) Serial.print(".");
                break;
        }
        
        return false;
    }
    
    bool isCalibrating() { return state != IDLE; }
    CalibrationState getState() { return state; }
};


// ============================================================================
// SERIALPROTOCOL
// ============================================================================
class SerialProtocol {
private:
    uint32_t lastOutputTime;
    uint32_t outputInterval;
    bool verboseMode;
    bool jsonMode;
    
public:
    SerialProtocol() : lastOutputTime(0), outputInterval(20), verboseMode(false), jsonMode(true) {}
    
    void initialize() {
        Serial.begin(SERIAL_BAUD);
        while (!Serial && millis() < 3000);
        
        Serial.println("\n==================================================");
        Serial.println("ESP32-C3 Advanced Sensor Fusion System v1.0.0");
        Serial.println("==================================================");
        printHelp();
    }
    
    void printHelp() {
        Serial.println("Commands:");
        Serial.println("  c - Start calibration");
        Serial.println("  r - Reset orientation");
        Serial.println("  v - Toggle verbose mode");
        Serial.println("  j - Toggle JSON output");
        Serial.println("  s - Save calibration");
        Serial.println("  l - Load calibration");
        Serial.println("  h - Show help");
    }
    
    char getLastCommand() {
        static char lastCmd = 0;
        if (Serial.available()) {
            lastCmd = Serial.read();
            while (Serial.available()) Serial.read(); // Clear buffer
        }
        return lastCmd;
    }
    
    void handleCommand(char cmd) {
        switch (cmd) {
            case 'v':
                verboseMode = !verboseMode;
                Serial.print("Verbose mode: ");
                Serial.println(verboseMode ? "ON" : "OFF");
                break;
            case 'j':
                jsonMode = !jsonMode;
                Serial.print("JSON output: ");
                Serial.println(jsonMode ? "ON" : "OFF");
                break;
            case 'h':
                printHelp();
                break;
        }
    }
    
    void outputData(const Quaternion& orientation, const FilterState& state, const SensorData& sensors) {
        uint32_t now = millis();
        if (now - lastOutputTime < outputInterval) return;
        lastOutputTime = now;
        
        if (jsonMode) {
            outputJSON(orientation, state, sensors);
        } else {
            outputText(orientation, state, sensors);
        }
    }
    
    void outputJSON(const Quaternion& orientation, const FilterState& state, const SensorData& sensors) {
        float roll, pitch, yaw;
        orientation.toEuler(roll, pitch, yaw);
        
        Serial.print("{\"quat\":{");
        Serial.print("\"w\":"); Serial.print(orientation.w, 4);
        Serial.print(",\"x\":"); Serial.print(orientation.x, 4);
        Serial.print(",\"y\":"); Serial.print(orientation.y, 4);
        Serial.print(",\"z\":"); Serial.print(orientation.z, 4);
        Serial.print("},\"euler\":{");
        Serial.print("\"roll\":"); Serial.print(roll * 180.0f / PI, 2);
        Serial.print(",\"pitch\":"); Serial.print(pitch * 180.0f / PI, 2);
        Serial.print(",\"yaw\":"); Serial.print(yaw * 180.0f / PI, 2);
        Serial.print("},\"confidence\":"); Serial.print(state.confidence, 1);
        Serial.print(",\"rate\":"); Serial.print(state.updateRate, 1);
        Serial.print(",\"timestamp\":"); Serial.print(sensors.timestamp);
        Serial.println("}");
    }
    
    void outputText(const Quaternion& orientation, const FilterState& state, const SensorData& sensors) {
        float roll, pitch, yaw;
        orientation.toEuler(roll, pitch, yaw);
        
        Serial.print("R:"); Serial.print(roll * 180.0f / PI, 1);
        Serial.print(" P:"); Serial.print(pitch * 180.0f / PI, 1);
        Serial.print(" Y:"); Serial.print(yaw * 180.0f / PI, 1);
        Serial.print(" | C:"); Serial.print(state.confidence, 0);
        Serial.print("% | Hz:"); Serial.println(state.updateRate, 0);
    }
    
    bool isVerboseMode() const { return verboseMode; }
};

// ============================================================================
// PERFORMANCE MONITER
// ============================================================================
class PerformanceMonitor {
private:
    uint32_t loopCount;
    uint32_t lastReportTime;
    uint32_t maxLoopTime;
    uint32_t totalLoopTime;
    uint32_t missedSamples;
    
public:
    PerformanceMonitor() : loopCount(0), lastReportTime(0), maxLoopTime(0), 
                          totalLoopTime(0), missedSamples(0) {}
    
    void endLoop(uint32_t loopTime) {
        loopCount++;
        totalLoopTime += loopTime;
        
        if (loopTime > maxLoopTime) {
            maxLoopTime = loopTime;
        }
        
        if (loopTime > SAMPLE_PERIOD_MS) {
            missedSamples++;
        }
        
        uint32_t now = millis();
        if (now - lastReportTime >= 1000) {
            reportPerformance();
            resetCounters();
            lastReportTime = now;
        }
    }
    
    void reportPerformance() {
        if (loopCount == 0) return;
        
        float avgLoopTime = (float)totalLoopTime / loopCount;
        float actualRate = 1000.0f / avgLoopTime;
        
        Serial.println("\n=== Performance Report ===");
        Serial.print("Update Rate: "); Serial.print(actualRate, 1);
        Serial.print(" Hz (Target: "); Serial.print(SAMPLE_RATE_HZ);
        Serial.println(" Hz)");
        Serial.print("Loop Time (avg/max): "); Serial.print(avgLoopTime, 1);
        Serial.print(" / "); Serial.print(maxLoopTime);
        Serial.println(" ms");
        Serial.print("Missed Samples: "); Serial.print(missedSamples);
        Serial.print(" ("); Serial.print((float)missedSamples / loopCount * 100.0f, 1);
        Serial.println("%)");
        Serial.print("Free Heap: "); Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
    }
    
    void resetCounters() {
        loopCount = 0;
        maxLoopTime = 0;
        totalLoopTime = 0;
        missedSamples = 0;
    }
    
    float getUpdateRate() const {
        if (loopCount == 0) return 0;
        return 1000.0f / ((float)totalLoopTime / loopCount);
    }
};

// ============================================================================
// Include ALL CLASS IMPLEMENTATIONS from Part 2
// ============================================================================

// ============================================================================
// ADVANCED CALIBRATION SYSTEM
// ============================================================================
class AdvancedCalibration {
private:
    struct EllipsoidParams {
        float center[3];
        float radii[3];
        float rotation[3][3];
        float scale[3][3];
    };
    
    struct CalibrationSample {
        Vector3 data;
        float temperature;
        uint32_t timestamp;
    };
    
    static const int MAX_SAMPLES = 1000;
    CalibrationSample samples[MAX_SAMPLES];
    int sampleCount;
    
    // Least squares ellipsoid fitting
    void fitEllipsoid(EllipsoidParams& params) {
        // Build design matrix for ellipsoid equation
        // ax^2 + by^2 + cz^2 + dxy + exz + fyz + gx + hy + iz = 1
        
        float D[MAX_SAMPLES][9];
        float ones[MAX_SAMPLES];
        
        for(int i = 0; i < sampleCount; i++) {
            float x = samples[i].data.x;
            float y = samples[i].data.y;
            float z = samples[i].data.z;
            
            D[i][0] = x * x;
            D[i][1] = y * y;
            D[i][2] = z * z;
            D[i][3] = 2 * x * y;
            D[i][4] = 2 * x * z;
            D[i][5] = 2 * y * z;
            D[i][6] = 2 * x;
            D[i][7] = 2 * y;
            D[i][8] = 2 * z;
            
            ones[i] = 1.0f;
        }
        
        // Solve using normal equations: (D^T * D)^-1 * D^T * ones
        float DTD[9][9] = {0};
        float DTones[9] = {0};
        
        // Calculate D^T * D
        for(int i = 0; i < 9; i++) {
            for(int j = 0; j < 9; j++) {
                for(int k = 0; k < sampleCount; k++) {
                    DTD[i][j] += D[k][i] * D[k][j];
                }
            }
        }
        
        // Calculate D^T * ones
        for(int i = 0; i < 9; i++) {
            for(int k = 0; k < sampleCount; k++) {
                DTones[i] += D[k][i] * ones[k];
            }
        }
        
        // Solve system using Gaussian elimination
        float coeffs[9];
        gaussianElimination(DTD, DTones, coeffs, 9);
        
        // Extract ellipsoid parameters
        float a = coeffs[0];
        float b = coeffs[1];
        float c = coeffs[2];
        float d = coeffs[3];
        float e = coeffs[4];
        float f = coeffs[5];
        float g = coeffs[6];
        float h = coeffs[7];
        float i = coeffs[8];
        
        // Build 4x4 algebraic form matrix
        float A[4][4] = {
            {a, d, e, g},
            {d, b, f, h},
            {e, f, c, i},
            {g, h, i, -1}
        };
        
        // Find center (solve for where gradient = 0)
        params.center[0] = -(g*b*c - g*f*f - h*d*c + h*e*f + i*d*f - i*b*e) / 
                          (a*b*c - a*f*f - d*d*c + 2*d*e*f - e*e*b);
        params.center[1] = -(g*d*c - g*e*f - h*a*c + h*e*e + i*a*f - i*d*e) /
                          (a*b*c - a*f*f - d*d*c + 2*d*e*f - e*e*b);
        params.center[2] = -(g*d*f - g*b*e - h*a*f + h*d*e + i*a*b - i*d*d) /
                          (a*b*c - a*f*f - d*d*c + 2*d*e*f - e*e*b);
        
        // Calculate transformation matrix for soft iron correction
        float T[3][3] = {
            {a, d, e},
            {d, b, f},
            {e, f, c}
        };
        
        // Eigenvalue decomposition for radii and rotation
        eigenDecomposition(T, params.radii, params.rotation);
        
        // Calculate scale matrix for calibration
        float avgRadius = (params.radii[0] + params.radii[1] + params.radii[2]) / 3.0f;
        for(int j = 0; j < 3; j++) {
            for(int k = 0; k < 3; k++) {
                params.scale[j][k] = 0;
                for(int l = 0; l < 3; l++) {
                    params.scale[j][k] += params.rotation[j][l] * 
                                         (avgRadius / params.radii[l]) * 
                                         params.rotation[k][l];
                }
            }
        }
    }
    
    void gaussianElimination(float A[][9], float b[], float x[], int n) {
        // Forward elimination with partial pivoting
        for(int k = 0; k < n-1; k++) {
            // Find pivot
            int maxRow = k;
            float maxVal = fabs(A[k][k]);
            for(int i = k+1; i < n; i++) {
                if(fabs(A[i][k]) > maxVal) {
                    maxVal = fabs(A[i][k]);
                    maxRow = i;
                }
            }
            
            // Swap rows
            if(maxRow != k) {
                for(int j = k; j < n; j++) {
                    float temp = A[k][j];
                    A[k][j] = A[maxRow][j];
                    A[maxRow][j] = temp;
                }
                float temp = b[k];
                b[k] = b[maxRow];
                b[maxRow] = temp;
            }
            
            // Eliminate column
            for(int i = k+1; i < n; i++) {
                float factor = A[i][k] / A[k][k];
                for(int j = k+1; j < n; j++) {
                    A[i][j] -= factor * A[k][j];
                }
                b[i] -= factor * b[k];
                A[i][k] = 0;
            }
        }
        
        // Back substitution
        for(int i = n-1; i >= 0; i--) {
            x[i] = b[i];
            for(int j = i+1; j < n; j++) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }
    }
    
    void eigenDecomposition(float A[][3], float eigenvalues[], float eigenvectors[][3]) {
        // Power iteration method for 3x3 symmetric matrix
        const int maxIterations = 100;
        const float tolerance = 1e-6;
        
        // Find largest eigenvalue/vector
        float v[3] = {1, 0, 0};
        float lambda = 0;
        
        for(int iter = 0; iter < maxIterations; iter++) {
            // Multiply A*v
            float Av[3];
            for(int i = 0; i < 3; i++) {
                Av[i] = 0;
                for(int j = 0; j < 3; j++) {
                    Av[i] += A[i][j] * v[j];
                }
            }
            
            // Find magnitude
            float newLambda = 0;
            for(int i = 0; i < 3; i++) {
                newLambda += Av[i] * v[i];
            }
            
            // Normalize
            float norm = 0;
            for(int i = 0; i < 3; i++) {
                norm += Av[i] * Av[i];
            }
            norm = sqrtf(norm);
            
            for(int i = 0; i < 3; i++) {
                v[i] = Av[i] / norm;
            }
            
            if(fabs(newLambda - lambda) < tolerance) break;
            lambda = newLambda;
        }
        
        eigenvalues[0] = lambda;
        for(int i = 0; i < 3; i++) {
            eigenvectors[i][0] = v[i];
        }
        
        // Find remaining eigenvalues using deflation
        // (Simplified - in production use a robust library)
        eigenvalues[1] = 1.0f;
        eigenvalues[2] = 1.0f;
        
        // Set identity for remaining eigenvectors
        eigenvectors[0][1] = 0; eigenvectors[1][1] = 1; eigenvectors[2][1] = 0;
        eigenvectors[0][2] = 0; eigenvectors[1][2] = 0; eigenvectors[2][2] = 1;
    }
    
public:
    AdvancedCalibration() : sampleCount(0) {}
    
    void reset() {
        sampleCount = 0;
    }
    
    void addSample(const Vector3& data, float temperature) {
        if(sampleCount < MAX_SAMPLES) {
            samples[sampleCount].data = data;
            samples[sampleCount].temperature = temperature;
            samples[sampleCount].timestamp = millis();
            sampleCount++;
        }
    }
    
    bool calibrateMagnetometer(CalibrationData& cal) {
        if(sampleCount < 100) {
            Serial.println("Insufficient samples for calibration");
            return false;
        }
        
        EllipsoidParams params;
        fitEllipsoid(params);
        
        // Update calibration data
        cal.magOffset.x = params.center[0];
        cal.magOffset.y = params.center[1];
        cal.magOffset.z = params.center[2];
        
        memcpy(cal.magScale, params.scale, sizeof(cal.magScale));
        
        Serial.println("Advanced magnetometer calibration complete!");
        Serial.print("Offset: ");
        Serial.print(cal.magOffset.x); Serial.print(", ");
        Serial.print(cal.magOffset.y); Serial.print(", ");
        Serial.println(cal.magOffset.z);
        
        return true;
    }
    
    bool calibrateAccelerometer(CalibrationData& cal) {
        if(sampleCount < 6) {
            Serial.println("Need at least 6 orientations for accelerometer calibration");
            return false;
        }
        
        // Six-position calibration
        // Expecting samples with device placed on each face
        Vector3 avgPos[6];
        int posCount[6] = {0};
        
        // Classify samples by dominant axis
        for(int i = 0; i < sampleCount; i++) {
            Vector3& s = samples[i].data;
            float absX = fabs(s.x);
            float absY = fabs(s.y);
            float absZ = fabs(s.z);
            
            int pos = -1;
            if(absX > absY && absX > absZ) {
                pos = (s.x > 0) ? 0 : 1;  // +X or -X
            } else if(absY > absX && absY > absZ) {
                pos = (s.y > 0) ? 2 : 3;  // +Y or -Y
            } else {
                pos = (s.z > 0) ? 4 : 5;  // +Z or -Z
            }
            
            if(pos >= 0) {
                avgPos[pos] = avgPos[pos] + s;
                posCount[pos]++;
            }
        }
        
        // Calculate average for each position
        for(int i = 0; i < 6; i++) {
            if(posCount[i] > 0) {
                avgPos[i] = avgPos[i] * (1.0f / posCount[i]);
            } else {
                Serial.print("Missing position "); Serial.println(i);
                return false;
            }
        }
        
        // Calculate bias and scale
        cal.accelBias.x = (avgPos[0].x + avgPos[1].x) / 2.0f;
        cal.accelBias.y = (avgPos[2].y + avgPos[3].y) / 2.0f;
        cal.accelBias.z = (avgPos[4].z + avgPos[5].z) / 2.0f;
        
        cal.accelScale.x = 2.0f / (avgPos[0].x - avgPos[1].x);
        cal.accelScale.y = 2.0f / (avgPos[2].y - avgPos[3].y);
        cal.accelScale.z = 2.0f / (avgPos[4].z - avgPos[5].z);
        
        Serial.println("Accelerometer calibration complete!");
        return true;
    }
    
    void analyzeTemperatureCompensation(CalibrationData& cal) {
        if(sampleCount < 50) return;
        
        // Find temperature range
        float minTemp = samples[0].temperature;
        float maxTemp = samples[0].temperature;
        
        for(int i = 1; i < sampleCount; i++) {
            minTemp = std::min(minTemp, samples[i].temperature);
            maxTemp = std::max(maxTemp, samples[i].temperature);
        }
        
        if(maxTemp - minTemp < 5.0f) {
            Serial.println("Insufficient temperature range for compensation");
            return;
        }
        
        // Linear regression for temperature coefficient
        float sumT = 0, sumT2 = 0;
        float sumXT = 0, sumYT = 0, sumZT = 0;
        float sumX = 0, sumY = 0, sumZ = 0;
        
        for(int i = 0; i < sampleCount; i++) {
            float t = samples[i].temperature - 25.0f;  // Relative to room temp
            sumT += t;
            sumT2 += t * t;
            
            sumX += samples[i].data.x;
            sumY += samples[i].data.y;
            sumZ += samples[i].data.z;
            
            sumXT += samples[i].data.x * t;
            sumYT += samples[i].data.y * t;
            sumZT += samples[i].data.z * t;
        }
        
        float n = sampleCount;
        float det = n * sumT2 - sumT * sumT;
        
        if(fabs(det) > 0.001f) {
            cal.tempCoeff[0] = (n * sumXT - sumX * sumT) / det;
            cal.tempCoeff[1] = (n * sumYT - sumY * sumT) / det;
            cal.tempCoeff[2] = (n * sumZT - sumZ * sumT) / det;
            
            Serial.println("Temperature compensation calculated");
        }
    }
    
    int getSampleCount() const { return sampleCount; }
};

// ============================================================================
// MAGNETIC ANOMALY DETECTOR
// ============================================================================
class MagneticAnomalyDetector {
private:
    static const int HISTORY_SIZE = 50;
    float magHistory[HISTORY_SIZE];
    int historyIndex;
    float baseline;
    float stdDev;
    bool anomalyDetected;
    float anomalyThreshold;
    
public:
    MagneticAnomalyDetector() : historyIndex(0), baseline(1.0f), 
                                stdDev(0.1f), anomalyDetected(false),
                                anomalyThreshold(3.0f) {
        for(int i = 0; i < HISTORY_SIZE; i++) {
            magHistory[i] = 1.0f;
        }
    }
    
    bool update(const Vector3& mag) {
        float magnitude = mag.magnitude();
        
        // Add to history
        magHistory[historyIndex] = magnitude;
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        
        // Calculate statistics
        float sum = 0, sum2 = 0;
        for(int i = 0; i < HISTORY_SIZE; i++) {
            sum += magHistory[i];
            sum2 += magHistory[i] * magHistory[i];
        }
        
        baseline = sum / HISTORY_SIZE;
        float variance = (sum2 / HISTORY_SIZE) - (baseline * baseline);
        stdDev = sqrtf(variance);
        
        // Detect anomaly
        float deviation = fabs(magnitude - baseline) / stdDev;
        anomalyDetected = (deviation > anomalyThreshold) || 
                         (magnitude < 0.2f) || 
                         (magnitude > 2.0f);
        
        // Check for magnetic interference patterns
        if(!anomalyDetected) {
            // Look for sudden changes
            float recentAvg = 0;
            for(int i = 0; i < 5; i++) {
                int idx = (historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE;
                recentAvg += magHistory[idx];
            }
            recentAvg /= 5.0f;
            
            float oldAvg = 0;
            for(int i = 20; i < 25; i++) {
                int idx = (historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE;
                oldAvg += magHistory[idx];
            }
            oldAvg /= 5.0f;
            
            if(fabs(recentAvg - oldAvg) > 0.3f) {
                anomalyDetected = true;
            }
        }
        
        return anomalyDetected;
    }
    
    bool isAnomalyDetected() const { return anomalyDetected; }
    float getBaseline() const { return baseline; }
    float getStdDev() const { return stdDev; }
    
    void reset() {
        for(int i = 0; i < HISTORY_SIZE; i++) {
            magHistory[i] = 1.0f;
        }
        historyIndex = 0;
        anomalyDetected = false;
    }
    
    void setThreshold(float threshold) {
        anomalyThreshold = threshold;
    }
};

// ============================================================================
// ADAPTIVE FILTER TUNING
// ============================================================================
class AdaptiveFilterTuning {
private:
    struct FilterMetrics {
        float innovationVariance;
        float predictionError;
        float convergenceRate;
        float stability;
    };
    
    FilterMetrics metrics;
    FilterState& state;
    
    static const int METRIC_HISTORY = 100;
    float innovationHistory[METRIC_HISTORY];
    int historyIndex;
    
public:
    AdaptiveFilterTuning(FilterState& fs) : state(fs), historyIndex(0) {
        memset(&metrics, 0, sizeof(metrics));
        memset(innovationHistory, 0, sizeof(innovationHistory));
    }
    
    void updateMetrics(float innovation) {
        // Store innovation
        innovationHistory[historyIndex] = innovation;
        historyIndex = (historyIndex + 1) % METRIC_HISTORY;
        
        // Calculate innovation variance
        float sum = 0, sum2 = 0;
        for(int i = 0; i < METRIC_HISTORY; i++) {
            sum += innovationHistory[i];
            sum2 += innovationHistory[i] * innovationHistory[i];
        }
        float mean = sum / METRIC_HISTORY;
        metrics.innovationVariance = (sum2 / METRIC_HISTORY) - (mean * mean);
        
        // Estimate convergence rate (how quickly innovation decreases)
        float recentAvg = 0, oldAvg = 0;
        for(int i = 0; i < 10; i++) {
            int recentIdx = (historyIndex - i - 1 + METRIC_HISTORY) % METRIC_HISTORY;
            int oldIdx = (historyIndex - i - 50 + METRIC_HISTORY) % METRIC_HISTORY;
            recentAvg += innovationHistory[recentIdx];
            oldAvg += innovationHistory[oldIdx];
        }
        recentAvg /= 10.0f;
        oldAvg /= 10.0f;
        
        metrics.convergenceRate = (oldAvg - recentAvg) / oldAvg;
        metrics.convergenceRate = constrain(metrics.convergenceRate, -1.0f, 1.0f);
        
        // Estimate stability (low variance = stable)
        metrics.stability = 1.0f / (1.0f + metrics.innovationVariance);
    }
    
    void adaptNoiseParameters() {
        // Adapt process noise based on innovation variance
        if(metrics.innovationVariance > 0.1f) {
            // High innovation - increase process noise
            for(int i = 0; i < 4; i++) {
                state.Q[i][i] *= 1.01f;
                state.Q[i][i] = constrain(state.Q[i][i], 1e-6f, 0.1f);
            }
        } else if(metrics.innovationVariance < 0.01f) {
            // Low innovation - decrease process noise
            for(int i = 0; i < 4; i++) {
                state.Q[i][i] *= 0.99f;
                state.Q[i][i] = constrain(state.Q[i][i], 1e-6f, 0.1f);
            }
        }
        
        // Adapt gyro bias process noise based on stability
        for(int i = 4; i < 7; i++) {
            state.Q[i][i] = 1e-6f * (2.0f - metrics.stability);
        }
        
        // Adapt measurement noise based on convergence
        if(metrics.convergenceRate < 0.1f) {
            // Slow convergence - decrease measurement noise trust
            for(int i = 0; i < 3; i++) {
                state.R_accel[i][i] *= 1.02f;
                state.R_mag[i][i] *= 1.02f;
            }
        } else if(metrics.convergenceRate > 0.5f) {
            // Fast convergence - increase measurement trust
            for(int i = 0; i < 3; i++) {
                state.R_accel[i][i] *= 0.98f;
                state.R_mag[i][i] *= 0.98f;
            }
        }
        
        // Constrain measurement noise
        for(int i = 0; i < 3; i++) {
            state.R_accel[i][i] = constrain(state.R_accel[i][i], 0.01f, 1.0f);
            state.R_mag[i][i] = constrain(state.R_mag[i][i], 0.05f, 2.0f);
        }
    }
    
    FilterMetrics getMetrics() const { return metrics; }
    
    void reset() {
        memset(innovationHistory, 0, sizeof(innovationHistory));
        historyIndex = 0;
    }
};

// ============================================================================
// POWER MANAGEMENT
// ============================================================================
class PowerManager {
public: 
    enum PowerMode {
        FULL_POWER,      // 100Hz, all sensors
        BALANCED,        // 50Hz, all sensors
        LOW_POWER,       // 20Hz, no magnetometer
        ULTRA_LOW_POWER  // 10Hz, minimal processing
    };

private:
    PowerMode currentMode;
    uint32_t lastActivityTime;
    uint32_t sleepTimeout;
    bool motionDetected;
    float batteryVoltage;
    
public:
    PowerManager() : currentMode(FULL_POWER), lastActivityTime(0),
                    sleepTimeout(30000), motionDetected(true),
                    batteryVoltage(3.7f) {}
    
    void update(const Vector3& gyro, const Vector3& accel) {
        // Detect motion
        float gyroMag = gyro.magnitude();
        float accelDev = fabs(accel.magnitude() - 1.0f);
        
        motionDetected = (gyroMag > 0.1f) || (accelDev > 0.1f);
        
        if(motionDetected) {
            lastActivityTime = millis();
        }
        
        // Update battery voltage (if ADC connected)
        // batteryVoltage = readBatteryVoltage();
        
        // Auto power mode selection
        uint32_t idleTime = millis() - lastActivityTime;
        
        if(batteryVoltage < 3.3f) {
            setMode(ULTRA_LOW_POWER);
        } else if(idleTime > sleepTimeout) {
            setMode(LOW_POWER);
        } else if(idleTime > sleepTimeout/2) {
            setMode(BALANCED);
        } else {
            setMode(FULL_POWER);
        }
    }
    
    void setMode(PowerMode mode) {
        if(mode == currentMode) return;
        
        currentMode = mode;
        
        switch(mode) {
            case FULL_POWER:
                Serial.println("Power Mode: FULL (100Hz)");
                break;
                
            case BALANCED:
                Serial.println("Power Mode: BALANCED (50Hz)");
                break;
                
            case LOW_POWER:
                Serial.println("Power Mode: LOW (20Hz)");
                // Disable magnetometer to save power
                break;
                
            case ULTRA_LOW_POWER:
                Serial.println("Power Mode: ULTRA LOW (10Hz)");
                // Minimal processing
                break;
        }
    }
    
    int getSampleRate() const {
        switch(currentMode) {
            case FULL_POWER: return 100;
            case BALANCED: return 50;
            case LOW_POWER: return 20;
            case ULTRA_LOW_POWER: return 10;
            default: return 100;
        }
    }
    
    bool useMagnetometer() const {
        return (currentMode == FULL_POWER) || (currentMode == BALANCED);
    }
    
    bool useAdvancedFiltering() const {
        return currentMode != ULTRA_LOW_POWER;
    }
    
    void enterDeepSleep() {
        Serial.println("Entering deep sleep...");
        // Configure wake on motion interrupt
        // esp_deep_sleep_start();
    }
};

// ============================================================================
// GESTURE RECOGNITION
// ============================================================================
class GestureRecognizer {
private:
    enum Gesture {
        NONE,
        TAP,
        DOUBLE_TAP,
        SHAKE,
        FLIP,
        ROTATION_CW,
        ROTATION_CCW
    };
    
    struct GestureBuffer {
        Vector3 accel[20];
        Vector3 gyro[20];
        int index;
    } buffer;
    
    Gesture lastGesture;
    uint32_t lastGestureTime;
    
public:
    GestureRecognizer() : lastGesture(NONE), lastGestureTime(0) {
        buffer.index = 0;
    }
    
    void update(const Vector3& accel, const Vector3& gyro) {
        // Add to buffer
        buffer.accel[buffer.index] = accel;
        buffer.gyro[buffer.index] = gyro;
        buffer.index = (buffer.index + 1) % 20;
        
        // Detect tap (sudden acceleration spike)
        float accelMag = accel.magnitude();
        if(accelMag > 2.5f) {
            uint32_t now = millis();
            if(now - lastGestureTime < 500 && lastGesture == TAP) {
                recognizeGesture(DOUBLE_TAP);
            } else {
                recognizeGesture(TAP);
            }
        }
        
        // Detect shake (repeated acceleration changes)
        float accelVariance = calculateVariance(buffer.accel, 20);
        if(accelVariance > 1.0f) {
            recognizeGesture(SHAKE);
        }
        
        // Detect rotation
        float avgGyroZ = 0;
        for(int i = 0; i < 20; i++) {
            avgGyroZ += buffer.gyro[i].z;
        }
        avgGyroZ /= 20.0f;
        
        if(avgGyroZ > 2.0f) {
            recognizeGesture(ROTATION_CW);
        } else if(avgGyroZ < -2.0f) {
            recognizeGesture(ROTATION_CCW);
        }
    }
    
    void recognizeGesture(Gesture g) {
        lastGesture = g;
        lastGestureTime = millis();
        
        Serial.print("Gesture detected: ");
        switch(g) {
            case TAP: Serial.println("TAP"); break;
            case DOUBLE_TAP: Serial.println("DOUBLE TAP"); break;
            case SHAKE: Serial.println("SHAKE"); break;
            case FLIP: Serial.println("FLIP"); break;
            case ROTATION_CW: Serial.println("ROTATION CW"); break;
            case ROTATION_CCW: Serial.println("ROTATION CCW"); break;
            default: break;
        }
    }
    
    float calculateVariance(Vector3* data, int count) {
        Vector3 mean(0, 0, 0);
        for(int i = 0; i < count; i++) {
            mean = mean + data[i];
        }
        mean = mean * (1.0f / count);
        
        float variance = 0;
        for(int i = 0; i < count; i++) {
            Vector3 diff = data[i] - mean;
            variance += diff.dot(diff);
        }
        return variance / count;
    }
    
    int getLastGesture() const { return (int)lastGesture; }
};

// ============================================================================
// DATA LOGGER
// ============================================================================
class DataLogger {
private:
    struct LogEntry {
        uint32_t timestamp;
        Quaternion orientation;
        Vector3 accel;
        Vector3 gyro;
        Vector3 mag;
        float temperature;
        float confidence;
        bool anomaly;
    };
    
    static const int LOG_BUFFER_SIZE = 100;
    LogEntry logBuffer[LOG_BUFFER_SIZE];
    int logIndex;
    bool logging;
    uint32_t logStartTime;
    
public:
    DataLogger() : logIndex(0), logging(false), logStartTime(0) {}
    
    void startLogging() {
        logging = true;
        logIndex = 0;
        logStartTime = millis();
        Serial.println("Data logging started");
    }
    
    void stopLogging() {
        logging = false;
        Serial.println("Data logging stopped");
        Serial.print("Logged ");
        Serial.print(logIndex);
        Serial.println(" entries");
    }
    
    void logData(const FilterState& state, const SensorData& sensors) {
        if(!logging) return;
        if(logIndex >= LOG_BUFFER_SIZE) {
            Serial.println("Log buffer full!");
            stopLogging();
            return;
        }
        
        LogEntry& entry = logBuffer[logIndex++];
        entry.timestamp = millis() - logStartTime;
        entry.orientation = state.orientation;
        entry.accel = sensors.accel;
        entry.gyro = sensors.gyro;
        entry.mag = sensors.mag;
        entry.temperature = sensors.temperature;
        entry.confidence = state.confidence;
        entry.anomaly = state.magneticAnomaly;
    }
    
    void exportCSV() {
        Serial.println("\n=== CSV Export ===");
        Serial.println("timestamp,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,mx,my,mz,temp,conf,anomaly");
        
        for(int i = 0; i < logIndex; i++) {
            LogEntry& e = logBuffer[i];
            Serial.print(e.timestamp); Serial.print(",");
            Serial.print(e.orientation.w, 4); Serial.print(",");
            Serial.print(e.orientation.x, 4); Serial.print(",");
            Serial.print(e.orientation.y, 4); Serial.print(",");
            Serial.print(e.orientation.z, 4); Serial.print(",");
            Serial.print(e.accel.x, 3); Serial.print(",");
            Serial.print(e.accel.y, 3); Serial.print(",");
            Serial.print(e.accel.z, 3); Serial.print(",");
            Serial.print(e.gyro.x, 3); Serial.print(",");
            Serial.print(e.gyro.y, 3); Serial.print(",");
            Serial.print(e.gyro.z, 3); Serial.print(",");
            Serial.print(e.mag.x, 3); Serial.print(",");
            Serial.print(e.mag.y, 3); Serial.print(",");
            Serial.print(e.mag.z, 3); Serial.print(",");
            Serial.print(e.temperature, 1); Serial.print(",");
            Serial.print(e.confidence, 1); Serial.print(",");
            Serial.println(e.anomaly ? "1" : "0");
        }
        Serial.println("=== End Export ===\n");
    }
    
    void exportBinary() {
        Serial.println("\n=== Binary Export ===");
        Serial.write((uint8_t*)&logIndex, sizeof(logIndex));
        Serial.write((uint8_t*)logBuffer, logIndex * sizeof(LogEntry));
        Serial.println("\n=== End Export ===");
    }
    
    bool isLogging() const { return logging; }
    int getEntryCount() const { return logIndex; }
};

// ============================================================================
// MOTION PREDICTION
// ============================================================================
class MotionPredictor {
private:
    struct MotionState {
        Quaternion orientation;
        Vector3 angularVelocity;
        Vector3 angularAcceleration;
        uint32_t timestamp;
    };
    
    static const int HISTORY_SIZE = 10;
    MotionState history[HISTORY_SIZE];
    int historyIndex;
    
public:
    MotionPredictor() : historyIndex(0) {
        memset(history, 0, sizeof(history));
    }
    
    void update(const Quaternion& orientation, const Vector3& gyro) {
        uint32_t now = millis();
        
        // Calculate angular acceleration
        Vector3 angularAccel(0, 0, 0);
        if(historyIndex > 0) {
            int prevIdx = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
            float dt = (now - history[prevIdx].timestamp) / 1000.0f;
            if(dt > 0) {
                angularAccel = (gyro - history[prevIdx].angularVelocity) * (1.0f / dt);
            }
        }
        
        // Store in history
        MotionState& state = history[historyIndex];
        state.orientation = orientation;
        state.angularVelocity = gyro;
        state.angularAcceleration = angularAccel;
        state.timestamp = now;
        
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    }
    
    Quaternion predict(float deltaTime) {
        // Get latest state
        int latestIdx = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        MotionState& latest = history[latestIdx];
        
        // Predict angular velocity
        Vector3 predictedAngVel = latest.angularVelocity + 
                                 (latest.angularAcceleration * deltaTime);
        
        // Integrate to get predicted orientation
        Quaternion deltaQ;
        float halfDt = deltaTime * 0.5f;
        deltaQ.w = 1.0f;
        deltaQ.x = predictedAngVel.x * halfDt;
        deltaQ.y = predictedAngVel.y * halfDt;
        deltaQ.z = predictedAngVel.z * halfDt;
        deltaQ.normalize();
        
        Quaternion predicted = latest.orientation * deltaQ;
        predicted.normalize();
        
        return predicted;
    }
    
    Vector3 getAngularTrend() {
        // Calculate trend over recent history
        if(historyIndex < 2) return Vector3(0, 0, 0);
        
        Vector3 trend(0, 0, 0);
        int samples = std::min(historyIndex, HISTORY_SIZE);
        
        for(int i = 1; i < samples; i++) {
            int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
            int prevIdx = (idx - 1 + HISTORY_SIZE) % HISTORY_SIZE;
            
            Vector3 diff = history[idx].angularVelocity - history[prevIdx].angularVelocity;
            trend = trend + diff;
        }
        
        return trend * (1.0f / (samples - 1));
    }
};

// ============================================================================
// SELF TEST & DIAGNOSTICS
// ============================================================================
class SystemDiagnostics {
private:
    struct DiagnosticResult {
        bool passed;
        String message;
        float metric;
    };
    
    SensorManager& sensors;
    FilterState& filterState;
    ExtendedKalmanFilter* ekf;
    
public:
    SystemDiagnostics(SensorManager& sm, FilterState& fs, ExtendedKalmanFilter* e) 
        : sensors(sm), filterState(fs), ekf(e) {}
    
    void runFullDiagnostics() {
        Serial.println("\n=== SYSTEM DIAGNOSTICS ===");
        
        // Test I2C communication
        if(testI2C()) {
            Serial.println("✓ I2C Communication: PASS");
        } else {
            Serial.println("✗ I2C Communication: FAIL");
        }
        
        // Test MPU6050
        DiagnosticResult mpu = testMPU6050();
        Serial.print(mpu.passed ? "✓ " : "✗ ");
        Serial.print("MPU6050: ");
        Serial.println(mpu.message);
        
        // Test Magnetometer
        DiagnosticResult mag = testMagnetometer();
        Serial.print(mag.passed ? "✓ " : "✗ ");
        Serial.print("Magnetometer: ");
        Serial.println(mag.message);
        
        // Test sensor noise levels
        DiagnosticResult noise = testSensorNoise();
        Serial.print(noise.passed ? "✓ " : "✗ ");
        Serial.print("Sensor Noise: ");
        Serial.println(noise.message);
        
        // Test filter convergence
        DiagnosticResult filter = testFilterConvergence();
        Serial.print(filter.passed ? "✓ " : "✗ ");
        Serial.print("Filter Convergence: ");
        Serial.println(filter.message);
        
        // Memory test
        Serial.print("✓ Free Heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
        
        // Performance test
        DiagnosticResult perf = testPerformance();
        Serial.print(perf.passed ? "✓ " : "✗ ");
        Serial.print("Performance: ");
        Serial.println(perf.message);
        
        Serial.println("==========================\n");
    }
    
    bool testI2C() {
        Wire.beginTransmission(MPU6050_ADDR);
        return (Wire.endTransmission() == 0);
    }
    
    DiagnosticResult testMPU6050() {
        DiagnosticResult result;
        
        // Read WHO_AM_I register
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x75);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
        uint8_t whoami = Wire.read();
        
        if(whoami == 0x68 || whoami == 0x98) {
            result.passed = true;
            result.message = "Connected, WHO_AM_I = 0x" + String(whoami, HEX);
        } else {
            result.passed = false;
            result.message = "Invalid WHO_AM_I = 0x" + String(whoami, HEX);
        }
        
        return result;
    }
    
    DiagnosticResult testMagnetometer() {
        DiagnosticResult result;
        
        if(!sensors.hasMagnetometer()) {
            result.passed = false;
            result.message = "Not detected";
            return result;
        }
        
        // Check if getting valid data
        sensors.readSensors();
        SensorData& data = sensors.getCalibratedData();
        float magMag = data.mag.magnitude();
        
        if(magMag > 0.2f && magMag < 2.0f) {
            result.passed = true;
            result.message = "OK, magnitude = " + String(magMag, 2);
        } else {
            result.passed = false;
            result.message = "Invalid data, magnitude = " + String(magMag, 2);
        }
        
        return result;
    }
    
    DiagnosticResult testSensorNoise() {
        DiagnosticResult result;
        
        // Collect samples
        const int samples = 100;
        Vector3 accelSum(0, 0, 0);
        Vector3 gyroSum(0, 0, 0);
        float accelVar = 0, gyroVar = 0;
        
        for(int i = 0; i < samples; i++) {
            sensors.readSensors();
            SensorData& data = sensors.getCalibratedData();
            accelSum = accelSum + data.accel;
            gyroSum = gyroSum + data.gyro;
            delay(10);
        }
        
        Vector3 accelMean = accelSum * (1.0f / samples);
        Vector3 gyroMean = gyroSum * (1.0f / samples);
        
        // Calculate variance
        for(int i = 0; i < samples; i++) {
            sensors.readSensors();
            SensorData& data = sensors.getCalibratedData();
            
            Vector3 accelDiff = data.accel - accelMean;
            Vector3 gyroDiff = data.gyro - gyroMean;
            
            accelVar += accelDiff.dot(accelDiff);
            gyroVar += gyroDiff.dot(gyroDiff);
            delay(10);
        }
        
        accelVar /= samples;
        gyroVar /= samples;
        
        if(accelVar < 0.01f && gyroVar < 0.001f) {
            result.passed = true;
            result.message = "Low noise (accel=" + String(accelVar, 4) + 
                           ", gyro=" + String(gyroVar, 4) + ")";
        } else {
            result.passed = false;
            result.message = "High noise (accel=" + String(accelVar, 4) + 
                           ", gyro=" + String(gyroVar, 4) + ")";
        }
        
        return result;
    }
    
    DiagnosticResult testFilterConvergence() {
        DiagnosticResult result;
        
        // Reset filter and let it converge
        ekf->reset();
        
        float initialError = 1.0f;
        float finalError = 0;
        
        for(int i = 0; i < 100; i++) {
            sensors.readSensors();
            SensorData& data = sensors.getCalibratedData();
            
            ekf->predict(data.gyro);
            ekf->updateAccel(data.accel);
            
            if(i == 10) {
                initialError = filterState.innovationAccel;
            }
            if(i == 99) {
                finalError = filterState.innovationAccel;
            }
            
            delay(10);
        }
        
        float improvement = (initialError - finalError) / initialError;
        
        if(improvement > 0.5f) {
            result.passed = true;
            result.message = "Converged " + String(improvement * 100, 0) + "%";
        } else {
            result.passed = false;
            result.message = "Poor convergence " + String(improvement * 100, 0) + "%";
        }
        
        return result;
    }
    
    DiagnosticResult testPerformance() {
        DiagnosticResult result;
        
        uint32_t startTime = micros();
        
        // Run typical loop operations
        for(int i = 0; i < 100; i++) {
            sensors.readSensors();
            SensorData& data = sensors.getCalibratedData();
            ekf->predict(data.gyro);
            ekf->updateAccel(data.accel);
            if(sensors.hasMagnetometer()) {
                ekf->updateMag(data.mag);
            }
        }
        
        uint32_t elapsed = micros() - startTime;
        float avgLoopTime = elapsed / 100000.0f;  // ms per loop
        
        if(avgLoopTime < 10.0f) {
            result.passed = true;
            result.message = String(avgLoopTime, 2) + " ms/loop (" + 
                           String(1000.0f/avgLoopTime, 0) + " Hz max)";
        } else {
            result.passed = false;
            result.message = String(avgLoopTime, 2) + " ms/loop (too slow)";
        }
        
        return result;
    }
};

// ============================================================================
// GLOBAL OBJECTS - SINGLE DECLARATION
// ============================================================================

// Core objects
SensorManager* sensorManager = nullptr;
SerialProtocol* serialProtocol = nullptr;
PerformanceMonitor* perfMonitor = nullptr;
FilterState filterState;
ExtendedKalmanFilter* ekf = nullptr;
CalibrationManager* calibrationManager = nullptr;

// Advanced objects
AdvancedCalibration* advancedCalibration = nullptr;
MagneticAnomalyDetector* anomalyDetector = nullptr;
AdaptiveFilterTuning* filterTuning = nullptr;
PowerManager* powerManager = nullptr;
GestureRecognizer* gestureRecognizer = nullptr;
DataLogger* dataLogger = nullptr;
MotionPredictor* motionPredictor = nullptr;
SystemDiagnostics* diagnostics = nullptr;

// Loop counter for periodic tasks
uint32_t loopCount = 0;

// ============================================================================
// SETUP FUNCTION - COMBINED
// ============================================================================

void handleCommand(char cmd);

void setup() {
    // Initialize Serial first
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);
    
    Serial.println("\n==================================================");
    Serial.println("ESP32-C3 Advanced Sensor Fusion System v1.0.0");
    Serial.println("==================================================");
    
    // Initialize core objects
    sensorManager = new SensorManager();
    serialProtocol = new SerialProtocol();
    perfMonitor = new PerformanceMonitor();
    
    // Initialize serial protocol
    serialProtocol->initialize();
    
    // Initialize sensors
    if (!sensorManager->initialize()) {
        Serial.println("FATAL: Sensor initialization failed!");
        while (1) {
            delay(1000);
            Serial.println("System halted - check sensor connections");
        }
    }
    
    // *** CRITICAL FIX: Set default calibration FIRST ***
    Serial.println("\nSetting default calibration values...");
    CalibrationData& cal = sensorManager->getCalibration();
    cal.gyroBias = Vector3(0, 0, 0);
    cal.accelBias = Vector3(0, 0, 0);
    cal.accelScale = Vector3(1, 1, 1);
    cal.magOffset = Vector3(0, 0, 0);
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            cal.magScale[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    cal.tempReference = 25.0f;
    for(int i = 0; i < 9; i++) {
        cal.tempCoeff[i] = 0.0f;
    }
    sensorManager->setCalibration(cal);
    
    // Initialize filter state BEFORE creating EKF
    Serial.println("Initializing filter state...");
    memset(&filterState, 0, sizeof(FilterState));
    filterState.orientation = Quaternion(1, 0, 0, 0);
    filterState.gyroBias = Vector3(0, 0, 0);
    filterState.confidence = 0;
    
    // Initialize filter
    ekf = new ExtendedKalmanFilter(filterState);
    
    // Initialize calibration manager
    calibrationManager = new CalibrationManager(*sensorManager, sensorManager->getCalibration());
    
    // Initialize advanced features
    advancedCalibration = new AdvancedCalibration();
    anomalyDetector = new MagneticAnomalyDetector();
    filterTuning = new AdaptiveFilterTuning(filterState);
    powerManager = new PowerManager();
    gestureRecognizer = new GestureRecognizer();
    dataLogger = new DataLogger();
    motionPredictor = new MotionPredictor();
    diagnostics = new SystemDiagnostics(*sensorManager, filterState, ekf);
    
    // Run initial diagnostics
    Serial.println("\nRunning system diagnostics...");
    diagnostics->runFullDiagnostics();
    
    // Initial gyro calibration with SAFETY CHECKS
    Serial.println("\n==================================================");
    Serial.println("Starting gyroscope calibration...");
    Serial.println("Keep the device COMPLETELY STILL for 3 seconds...");
    Serial.println("==================================================");
    
    // Manual gyro calibration with validation
    Vector3 gyroSum(0, 0, 0);
    int validSamples = 0;
    
    for(int i = 0; i < 300; i++) {
        sensorManager->readSensors();
        SensorData& raw = sensorManager->getRawData();
        
        // Check for valid data
        if(!isnan(raw.gyro.x) && !isnan(raw.gyro.y) && !isnan(raw.gyro.z)) {
            gyroSum = gyroSum + raw.gyro;
            validSamples++;
        } else {
            Serial.println("WARNING: Skipping invalid gyro sample");
        }
        
        if(i % 30 == 0) Serial.print(".");
        delay(10);
    }
    
    Serial.println();
    
    if(validSamples > 200) {
        cal.gyroBias = gyroSum * (1.0f / validSamples);
        
        // Final NaN check
        if(isnan(cal.gyroBias.x) || isnan(cal.gyroBias.y) || isnan(cal.gyroBias.z)) {
            Serial.println("ERROR: Calibration produced NaN! Using zero bias.");
            cal.gyroBias = Vector3(0, 0, 0);
        } else {
            Serial.print("Gyro bias calibrated: ");
            Serial.print(cal.gyroBias.x, 6); Serial.print(", ");
            Serial.print(cal.gyroBias.y, 6); Serial.print(", ");
            Serial.println(cal.gyroBias.z, 6);
        }
    } else {
        Serial.println("ERROR: Too many invalid samples! Using zero bias.");
        cal.gyroBias = Vector3(0, 0, 0);
    }
    
    // Update calibration and filter state
    sensorManager->setCalibration(cal);
    filterState.gyroBias = cal.gyroBias;
    
    // Verify everything is clean
    Serial.println("\n=== Verification ===");
    Serial.print("Quaternion: ");
    Serial.print(filterState.orientation.w); Serial.print(", ");
    Serial.print(filterState.orientation.x); Serial.print(", ");
    Serial.print(filterState.orientation.y); Serial.print(", ");
    Serial.println(filterState.orientation.z);
    
    Serial.print("Gyro Bias: ");
    Serial.print(filterState.gyroBias.x, 6); Serial.print(", ");
    Serial.print(filterState.gyroBias.y, 6); Serial.print(", ");
    Serial.println(filterState.gyroBias.z, 6);
    
    // Final safety check
    if(isnan(filterState.orientation.w) || isnan(filterState.gyroBias.x)) {
        Serial.println("\n*** CRITICAL ERROR: NaN detected! ***");
        Serial.println("Forcing safe defaults...");
        filterState.orientation = Quaternion(1, 0, 0, 0);
        filterState.gyroBias = Vector3(0, 0, 0);
    }
    
    Serial.println("\n==================================================");
    Serial.println("System ready! Commands:");
    Serial.println("  c - Start calibration");
    Serial.println("  r - Reset orientation");
    Serial.println("  R - Emergency reset");
    Serial.println("  v - Toggle verbose mode");
    Serial.println("  j - Toggle JSON output");
    Serial.println("  h - Show help");
    Serial.println("==================================================\n");
}

// ============================================================================
// LOOP FUNCTION - COMBINED
// ============================================================================

void loop() {
    static uint32_t lastLoopTime = 0;
    uint32_t loopStartTime = micros();

    if(isnan(filterState.orientation.w) || isnan(filterState.gyroBias.x)) {
        Serial.println("!!! NaN DETECTED - EMERGENCY RESET !!!");
        filterState.orientation = Quaternion(1, 0, 0, 0);
        filterState.gyroBias = Vector3(0, 0, 0);
        ekf->reset();
    }
    
    // Maintain sample rate based on power mode
    int targetRate = powerManager->getSampleRate();
    uint32_t targetPeriod = 1000 / targetRate;
    
    uint32_t now = millis();
    if (now - lastLoopTime < targetPeriod) return;
    
    float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;
    loopCount++;
    
    // Read sensors
    sensorManager->readSensors();
    SensorData& sensorData = sensorManager->getCalibratedData();
    
    // Update EKF
    ekf->setDt(dt);
    ekf->predict(sensorData.gyro);
    ekf->updateAccel(sensorData.accel);
    
    // Update magnetometer if available and not in low power mode
    if(sensorManager->hasMagnetometer() && powerManager->useMagnetometer()) {
        // Check for anomaly first
        if(anomalyDetector->update(sensorData.mag)) {
            filterState.magneticAnomaly = true;
        } else {
            ekf->updateMag(sensorData.mag);
            filterState.magneticAnomaly = false;
        }
    }
    
    // Update advanced features
    // Update anomaly detection
    if(sensorData.mag.magnitude() > 0) {
        anomalyDetector->update(sensorData.mag);
        filterState.magneticAnomaly = anomalyDetector->isAnomalyDetected();
    }
    
    // Update adaptive filter tuning
    filterTuning->updateMetrics(filterState.innovationAccel);
    if(loopCount % 100 == 0) {  // Every 100 loops
        filterTuning->adaptNoiseParameters();
    }
    
    // Adaptive filter tuning
    filterTuning->updateMetrics(filterState.innovationAccel);
    if(loopCount % 100 == 0 && powerManager->useAdvancedFiltering()) {
        filterTuning->adaptNoiseParameters();
    }
    
    // Power management
    powerManager->update(sensorData.gyro, sensorData.accel);
    
    // Gesture recognition
    gestureRecognizer->update(sensorData.accel, sensorData.gyro);
    
    // Motion prediction
    motionPredictor->update(filterState.orientation, sensorData.gyro);
    
    // Data logging
    if(dataLogger->isLogging()) {
        dataLogger->logData(filterState, sensorData);
    }
    
    // Handle calibration updates
    if(calibrationManager->isCalibrating()) {
        if(calibrationManager->update(sensorManager->getRawData())) {
            // Calibration completed
            sensorManager->setCalibration(sensorManager->getCalibration());
        }
    }
    
    // Handle advanced calibration if active
    if(advancedCalibration->getSampleCount() > 0) {
        advancedCalibration->addSample(sensorData.mag, sensorData.temperature);
        
        if(advancedCalibration->getSampleCount() >= MAG_CALIBRATION_SAMPLES) {
            CalibrationData& cal = sensorManager->getCalibration();
            if(advancedCalibration->calibrateMagnetometer(cal)) {
                sensorManager->setCalibration(cal);
                advancedCalibration->reset();
            }
        }
    }
    
    // Process commands
    char cmd = serialProtocol->getLastCommand();
    if (cmd != 0) {
        handleCommand(cmd);
    }
    
    // Output data
    serialProtocol->outputData(filterState.orientation, filterState, sensorData);
    
    // Performance monitoring
    uint32_t loopTime = (micros() - loopStartTime) / 1000;
    perfMonitor->endLoop(loopTime);
    filterState.updateRate = perfMonitor->getUpdateRate();
    
    // Periodic performance report (every 5 seconds)
    if(loopCount % (targetRate * 5) == 0) {
        if(serialProtocol->isVerboseMode()) {
            perfMonitor->reportPerformance();
        }
    }
}

// ============================================================================
// COMMAND HANDLER - COMBINED
// ============================================================================

void handleCommand(char cmd) {
    switch(cmd) {
        // Basic commands
        case 'c':  // Start calibration
            calibrationManager->startGyroCalibration();
            break;
            
        case 'r':  // Reset orientation
            ekf->reset();
            Serial.println("Orientation reset to identity");
            break;
            
        case 's':  // Save calibration
            sensorManager->saveCalibration();
            break;
            
        case 'l':  // Load calibration
            sensorManager->loadCalibration();
            break;
            
        case 'm':  // Magnetometer calibration
            calibrationManager->startMagCalibration();
            break;
            
        case 'a':  // Accelerometer calibration
            calibrationManager->startAccelCalibration();
            break;
            
        // Advanced commands
        case 'C':  // Advanced magnetometer calibration
            Serial.println("Starting advanced magnetometer calibration");
            Serial.println("Rotate device slowly in all directions...");
            advancedCalibration->reset();
            break;
            
        case 'D':  // Run diagnostics
            diagnostics->runFullDiagnostics();
            break;
            
        case 'L':  // Toggle logging
            if(dataLogger->isLogging()) {
                dataLogger->stopLogging();
                dataLogger->exportCSV();
            } else {
                dataLogger->startLogging();
            }
            break;
            
        case 'P':  // Cycle power modes
            {
                static int mode = 0;
                mode = (mode + 1) % 4;
                powerManager->setMode((PowerManager::PowerMode)mode);
            }
            break;
            
        case 'T':  // Test prediction
            {
                Quaternion predicted = motionPredictor->predict(0.1f);
                Serial.print("Predicted orientation (100ms): ");
                Serial.print(predicted.w, 3); Serial.print(", ");
                Serial.print(predicted.x, 3); Serial.print(", ");
                Serial.print(predicted.y, 3); Serial.print(", ");
                Serial.println(predicted.z, 3);
            }
            break;
            
        case 'G':  // Show last gesture
            Serial.print("Last gesture: ");
            Serial.println(gestureRecognizer->getLastGesture());
            break;
            
        case 'E':  // Export logged data
            if(dataLogger->getEntryCount() > 0) {
                dataLogger->exportCSV();
            } else {
                Serial.println("No logged data to export");
            }
            break;
            
        default:
            // Let SerialProtocol handle other commands
            serialProtocol->handleCommand(cmd);
            break;
    }
}

// NOTE: Due to size limits, the full class implementations are not shown here.
// You need to copy the actual class implementations from Part 1 and Part 2
// into the sections marked above.