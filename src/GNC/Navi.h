#ifndef NAVI_H
#define NAVI_H

#include <Arduino.h>
#include <EEPROM.h>
#include <MPU9250.h>
#include <TinyGPSPlus.h>

class Navi {
public:
    Navi();

    void init();
    void loop();

    void calibrate();

    // IMU getter
    double getRawAccX() const { return raw_acc_x; }
    double getRawAccY() const { return raw_acc_y; }
    double getRawAccZ() const { return raw_acc_z; }
    double getLinAccX() const { return lin_acc_x; }
    double getLinAccY() const { return lin_acc_y; }
    double getLinAccZ() const { return lin_acc_z; }
    double getRawAngX() const { return raw_ang_x; }
    double getRawAngY() const { return raw_ang_y; }
    double getRawAngZ() const { return raw_ang_z; }
    double getRawMagX() const { return raw_mag_x; }
    double getRawMagY() const { return raw_mag_y; }
    double getRawMagZ() const { return raw_mag_z; }

    // GPS getter
    double getSats() const { return sats; }
    double getHdop() const { return hdop; }
    double getLat()  const { return lat; }
    double getLng()  const { return lng; }
    double getAge()  const { return age; }
    double getAlt()  const { return alt_GPS; }
    double getGPSLast() const { return millis() - prev_pkt_gps; }

    // Navigation Solution
    double getR() const { return r_cur; }
    double getP() const { return p_cur; }
    double getY() const { return y_cur; }

private:
    void updateAttitude();

    MPU9250 mpu;
    TinyGPSPlus gps;

    uint32_t prev_ms_imu;
    uint32_t prev_ms_gps;
    uint32_t prev_pkt_gps;

    // IMU Sensor's Raw Output
    float raw_acc_x, raw_acc_y, raw_acc_z;  // TOTO : scale (deg, rad) identify
    float lin_acc_x, lin_acc_y, lin_acc_z;  // TOTO : scale (deg, rad) identify
    float raw_ang_x, raw_ang_y, raw_ang_z;  // TOTO : scale (deg, rad) identify
    float raw_mag_x, raw_mag_y, raw_mag_z;  // TOTO : scale (deg, rad) identify

    // GPS Sensor's Decoded Output
    int sats;
    float hdop, lat, lng;
    uint32_t age;
    float alt_GPS;

    // Navigation Solution
    float r_cur, p_cur, y_cur; // Euler angles

    // About EEPROM
    static const uint8_t EEPROM_SIZE = 1 + sizeof(float) * 3 * 4;   // TODO : check is this OK?

    enum EEP_ADDR {
        EEP_CALIB_FLAG = 0x00,
        EEP_ACC_BIAS   = 0x01,
        EEP_GYRO_BIAS  = 0x0D,
        EEP_MAG_BIAS   = 0x19,
        EEP_MAG_SCALE  = 0x25
    };

    void writeByte(int address, byte value);
    void writeFloat(int address, float value);
    byte readByte(int address);
    float readFloat(int address);
    void clearCalibration();
    bool isCalibrated();
    void saveCalibration();
    void loadCalibration();
    void printCalibration();
    void printBytes();
    void setupEEPROM();
};

#endif
