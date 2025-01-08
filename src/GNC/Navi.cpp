#include <Arduino.h>
#include "GNC/Navi.h"
#include "config.h"

Navi::Navi()
    : mpu(), gps(), 
      prev_ms_imu(0), prev_ms_gps(0), prev_pkt_gps(0),
      raw_acc_x(0), raw_acc_y(0), raw_acc_z(0),
      lin_acc_x(0), lin_acc_y(0), lin_acc_z(0),
      raw_ang_x(0), raw_ang_y(0), raw_ang_z(0),
      raw_mag_x(0), raw_mag_y(0), raw_mag_z(0),
      sats(0), hdop(0), lat(0), lng(0), age(0), alt_GPS(0),
      r_cur(0), p_cur(0), y_cur(0) {}

void Navi::init() {
    // for IMU
    Wire.begin();
    delay(2000);
    if (!mpu.setup(ADDR_MPU9250)) {
        while (1) {
            Serial.println("MPU connection failed");
            digitalWrite(BUZ, HIGH);
            delay(50);
            digitalWrite(BUZ, LOW);
            delay(50);
        }
    }
    loadCalibration();  // TODO : EEPROM begin is needed?

    // for GPS
    Serial5.begin(9600);    // for GPS
}

void Navi::loop() {
    uint32_t curr_ms = millis();

    if (curr_ms - prev_ms_gps >= 100) {   // 10Hz
        sats    = gps.satellites.value();   // default : 0
        hdop    = gps.hdop.hdop();          // default : 99.99  // 99.99->poor, 0.01->nice
        lat     = gps.location.lat();       // default : 0.0
        lng     = gps.location.lng();       // default : 0.0
        age     = gps.location.age();       // default : 4294967295 (ms)
        alt_GPS = gps.altitude.meters();    // default : 0.0
        prev_ms_gps = curr_ms;
    }

    while (Serial5.available()) {
        gps.encode(Serial5.read());
        prev_pkt_gps = curr_ms;
    }

    if (curr_ms - prev_ms_imu >= 10) {
        updateAttitude();
        prev_ms_imu = curr_ms;
    }
}

void Navi::updateAttitude() {
    if (mpu.update()) {
        raw_acc_x = mpu.getAccX();
        raw_acc_y = mpu.getAccY();
        raw_acc_z = mpu.getAccZ();
        lin_acc_x = mpu.getLinearAccX();
        lin_acc_y = mpu.getLinearAccY();
        lin_acc_z = mpu.getLinearAccZ();
        raw_ang_x = mpu.getGyroX();
        raw_ang_y = mpu.getGyroY();
        raw_ang_z = mpu.getGyroZ();
        raw_mag_x = mpu.getMagX();
        raw_mag_y = mpu.getMagY();
        raw_mag_z = mpu.getMagZ();

        r_cur  = mpu.getEulerX();   // TODO : Align with attached course
        p_cur  = mpu.getEulerY();   // TODO : Align with attached course
        y_cur  = mpu.getEulerZ();   // TODO : Align with attached course
    }
}

void Navi::calibrate() {
    Serial.println("Accel Gyro calibration will start.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);

    // Accel & Gyro Calibration
    for (int i = 0; i < 3; ++i) {
        digitalWrite(BUZ, HIGH);
        delay(100);
        digitalWrite(BUZ, LOW);
        delay(1000);
    }
    mpu.calibrateAccelGyro();
    digitalWrite(BUZ, HIGH);
    delay(1000);
    digitalWrite(BUZ, LOW);

    Serial.println("Accel Gyro calibration Done.");
    Serial.println("Mag calibration will start.");
    Serial.println("Please wave device in a figure eight until done.");

    // Magnetometer Calibration
    for (int i = 0; i < 2; ++i) {
        digitalWrite(BUZ, HIGH);
        delay(50);
        digitalWrite(BUZ, LOW);
        delay(50);
    }
    mpu.calibrateMag();
    digitalWrite(BUZ, HIGH);
    delay(1000);
    digitalWrite(BUZ, LOW);

    // Calibration Save
    saveCalibration();
    Serial.println("Mag calibration Done.");
}



void Navi::writeByte(int address, byte value) {
    EEPROM.put(address, value);
}

void Navi::writeFloat(int address, float value) {
    EEPROM.put(address, value);
}

byte Navi::readByte(int address) {
    byte valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

float Navi::readFloat(int address) {
    float valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

void Navi::clearCalibration() {
    writeByte(EEP_CALIB_FLAG, 0);
}

bool Navi::isCalibrated() {
    return (readByte(EEP_CALIB_FLAG) == 0x01);
}

void Navi::saveCalibration() {
    Serial.println("Write calibrated parameters to EEPROM");
    writeByte(EEP_CALIB_FLAG, 1);
    writeFloat(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    writeFloat(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    writeFloat(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    writeFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    writeFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    writeFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    writeFloat(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    writeFloat(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    writeFloat(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    writeFloat(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    writeFloat(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    writeFloat(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
#if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.commit();    // TODO : working test
#endif
}

void Navi::loadCalibration() {
    Serial.println("Load calibrated parameters from EEPROM");
    if (isCalibrated()) {
        Serial.println("calibrated? : YES");
        Serial.println("load calibrated values");
        mpu.setAccBias(
            readFloat(EEP_ACC_BIAS + 0),
            readFloat(EEP_ACC_BIAS + 4),
            readFloat(EEP_ACC_BIAS + 8));
        mpu.setGyroBias(
            readFloat(EEP_GYRO_BIAS + 0),
            readFloat(EEP_GYRO_BIAS + 4),
            readFloat(EEP_GYRO_BIAS + 8));
        mpu.setMagBias(
            readFloat(EEP_MAG_BIAS + 0),
            readFloat(EEP_MAG_BIAS + 4),
            readFloat(EEP_MAG_BIAS + 8));
        mpu.setMagScale(
            readFloat(EEP_MAG_SCALE + 0),
            readFloat(EEP_MAG_SCALE + 4),
            readFloat(EEP_MAG_SCALE + 8));
    }
    else {
        Serial.println("calibrated? : NO");
        Serial.println("load default values");
        mpu.setAccBias(0., 0., 0.);
        mpu.setGyroBias(0., 0., 0.);
        mpu.setMagBias(0., 0., 0.);
        mpu.setMagScale(1., 1., 1.);
    }
}

void Navi::printCalibration() {
    Serial.println("< calibration parameters >");
    Serial.print("calibrated? : ");
    Serial.println(readByte(EEP_CALIB_FLAG) ? "YES" : "NO");
    Serial.print("acc bias x  : ");
    Serial.println(readFloat(EEP_ACC_BIAS + 0) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print("acc bias y  : ");
    Serial.println(readFloat(EEP_ACC_BIAS + 4) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print("acc bias z  : ");
    Serial.println(readFloat(EEP_ACC_BIAS + 8) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print("gyro bias x : ");
    Serial.println(readFloat(EEP_GYRO_BIAS + 0) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print("gyro bias y : ");
    Serial.println(readFloat(EEP_GYRO_BIAS + 4) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print("gyro bias z : ");
    Serial.println(readFloat(EEP_GYRO_BIAS + 8) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print("mag bias x  : ");
    Serial.println(readFloat(EEP_MAG_BIAS + 0));
    Serial.print("mag bias y  : ");
    Serial.println(readFloat(EEP_MAG_BIAS + 4));
    Serial.print("mag bias z  : ");
    Serial.println(readFloat(EEP_MAG_BIAS + 8));
    Serial.print("mag scale x : ");
    Serial.println(readFloat(EEP_MAG_SCALE + 0));
    Serial.print("mag scale y : ");
    Serial.println(readFloat(EEP_MAG_SCALE + 4));
    Serial.print("mag scale z : ");
    Serial.println(readFloat(EEP_MAG_SCALE + 8));
}

void Navi::printBytes() {
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial.println(readByte(i), HEX);
}

void Navi::setupEEPROM() {
    Serial.println("EEPROM start");

    if (!isCalibrated()) {
        Serial.println("Need Calibration!!");
    }
    Serial.println("EEPROM calibration value is : ");
    printCalibration();
    Serial.println("Loaded calibration value is : ");
    loadCalibration();
}
