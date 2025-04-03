struct data{
  int gyro;
  bool start;
  bool solen;
};

data data;

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>

RF24 radio(9, 10);

const byte address[6] = "01001";

MPU6050 mpu;

typedef struct {
    float x, y, z;
} GyroData;

typedef struct {
    float x, y;
} AccelData;

typedef struct {
    float x, y, z;
} AngleData;

GyroData gyroOffsets = {0, 0, 0};
AngleData currentAngles = {0, 0, 0};
unsigned long prevTime = 0;

const float gyroScale = 131.0;
const float alpha = 0.98;
const float driftThreshold = 1.0;

bool initMPU() {
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return false;
    }
    
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    mpu.setRate(50);
    delay(100);
    
    return true;
}

void calibrateGyro(int samples = 500) {
    GyroData sum = {0, 0, 0};
    Serial.println("Калибровка гироскопа...");
    
    for (int i = 0; i < samples; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum.x += gx;
        sum.y += gy;
        sum.z += gz;
        delay(5);
    }
    
    gyroOffsets.x = sum.x / samples;
    gyroOffsets.y = sum.y / samples;
    gyroOffsets.z = sum.z / samples;
    
    Serial.println("Калибровка завершена");
}

GyroData readGyro() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    
    GyroData data;
    data.x = (gx - gyroOffsets.x) / gyroScale;
    data.y = (gy - gyroOffsets.y) / gyroScale;
    data.z = (gz - gyroOffsets.z) / gyroScale;
    
    if (isnan(data.x) || abs(data.x) > 1000) data.x = 0;
    if (isnan(data.y) || abs(data.y) > 1000) data.y = 0;
    if (isnan(data.z) || abs(data.z) > 1000) data.z = 0;
    
    return data;
}


AccelData readAccel() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    AccelData data;
    float denominator = sqrt(ay * ay + az * az);
    

    if (denominator < 0.1f) denominator = 0.1f;
    data.x = atan2(ay, denominator) * 180.0 / PI;
    
 
    float accel_y = -ax;
    float accel_xz = sqrt(ay * ay + az * az);
    if (accel_xz < 0.1f) accel_xz = 0.1f;
    data.y = atan2(accel_y, accel_xz) * 180.0 / PI;

    if (isnan(data.x)) data.x = 0;
    if (isnan(data.y)) data.y = 0;
    
    return data;
}

AngleData updateAngles(GyroData gyro, AccelData accel, float deltaTime) {
    static AngleData angles = {0, 0, 0};
    
    if (!isnan(accel.x) && !isnan(accel.y)) {
        angles.x = alpha * (angles.x + gyro.x * deltaTime) + (1 - alpha) * accel.x;
        angles.y = alpha * (angles.y + gyro.y * deltaTime) + (1 - alpha) * accel.y;
    } else {
        angles.x += gyro.x * deltaTime;
        angles.y += gyro.y * deltaTime;
    }
    
    angles.z += gyro.z * deltaTime;
    
    angles.x = constrain(angles.x, -180, 180);
    angles.y = constrain(angles.y, -90, 90);
    
    return angles;
}

void autoResetDrift(AngleData &angles, AccelData accel) {
    if (abs(accel.x) < driftThreshold && abs(accel.y) < driftThreshold) {
        angles.x = 0.99 * angles.x + 0.01 * accel.x;
        angles.y = 0.99 * angles.y + 0.01 * accel.y;
    }
    
    if (abs(angles.x) > 360) angles.x = 0;
    if (abs(angles.y) > 180) angles.y = 0;
}

void setup() {
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    Serial.println("Инициализация MPU6050...");
    if (!initMPU()) {
        Serial.println("Ошибка инициализации!");
        while (1);
    }
    calibrateGyro();
    prevTime = micros();
    Serial.println("Готов к работе");
}

bool start = 0;
bool solen = 0;

void loop() {
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - prevTime) / 1000000.0;
    prevTime = currentTime;
    
    GyroData gyro = readGyro();
    AccelData accel = readAccel();
    
    currentAngles = updateAngles(gyro, accel, deltaTime);
    autoResetDrift(currentAngles, accel);

    int v = constrain(map(currentAngles.z, 25, -25, 1, 3), 1, 3);

    Serial.println(v);

    data.gyro = v;
    data.start = start;
    data.solen = solen;

    radio.write(&data, sizeof(data));
    
    delay(10);
}