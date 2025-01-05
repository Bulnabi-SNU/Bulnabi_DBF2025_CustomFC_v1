#include <Arduino.h>
#include "GNC/Ctrl.h"
#include "GNC/Navi.h"
#include "config.h"

Ctrl::Ctrl(Navi& navi)
    : navi(navi), state_arm(false), prev_ms_ctrl(0),
      t_tar(0), r_tar(0), p_tar(0), y_tar(0),
      itg_r(0), itg_p(0), itg_y(0),
      drv_r(0), drv_p(0), drv_y(0),
      prev_err_r(0), prev_err_p(0), prev_err_y(0) {}

void Ctrl::init()
{
    pinMode(MTR_1, OUTPUT);
    pinMode(MTR_2, OUTPUT);
    pinMode(MTR_3, OUTPUT);
    pinMode(MTR_4, OUTPUT);
    stopMotor();
}

void Ctrl::loop()
{
    uint32_t curr_ms = millis();
    if (curr_ms - prev_ms_ctrl >= 10) {
        if (state_arm)
        {
            updateMotor();
        }
        else
        {
            stopMotor();
        }
        
        prev_ms_ctrl = curr_ms;
    }
}

void Ctrl::updateMotor()
{
    double dt = 0.01;

    // 현재 상태와 목표 상태의 오류 계산
    double err_r = r_tar - navi.getR();
    double err_p = p_tar - navi.getP();
    double err_y = y_tar - navi.getY();

    // PID 계산: Roll
    itg_r += err_r * dt;
    itg_r = constrain(itg_r, -ITG_LIMIT_R, ITG_LIMIT_R); // 적분 항 제한
    double drv_r = (err_r - prev_err_r) / dt;
    double control_r = GAIN_R_Kp * err_r + GAIN_R_Ki * itg_r + GAIN_R_Kd * drv_r;

    // PID 계산: Pitch
    itg_p += err_p * dt;
    itg_p = constrain(itg_p, -ITG_LIMIT_P, ITG_LIMIT_P); // 적분 항 제한
    double drv_p = (err_p - prev_err_p) / dt;
    double control_p = GAIN_P_Kp * err_p + GAIN_P_Ki * itg_p + GAIN_P_Kd * drv_p;

    // PID 계산: Yaw
    itg_y += err_y * dt;
    itg_y = constrain(itg_y, -ITG_LIMIT_Y, ITG_LIMIT_Y); // 적분 항 제한
    double drv_y = (err_y - prev_err_y) / dt;
    double control_y = GAIN_Y_Kp * err_y + GAIN_Y_Ki * itg_y + GAIN_Y_Kd * drv_y;

    // 믹싱 (쿼드콥터 4개의 모터 출력으로 분배)
    double m1 = t_tar + control_r + control_p - control_y; // Front-right
    double m2 = t_tar - control_r + control_p + control_y; // Front-left
    double m3 = t_tar - control_r - control_p - control_y; // Rear-left
    double m4 = t_tar + control_r - control_p + control_y; // Rear-right

    // 모터 출력 범위 제한 (0 ~ 255)
    m1 = constrain(m1, 0, 255);
    m2 = constrain(m2, 0, 255);
    m3 = constrain(m3, 0, 255);
    m4 = constrain(m4, 0, 255);

    // 모터 출력
    analogWrite(MTR_1, (int)m1);
    analogWrite(MTR_2, (int)m2);
    analogWrite(MTR_3, (int)m3);
    analogWrite(MTR_4, (int)m4);

    // 이전 값 업데이트
    prev_err_r = err_r;
    prev_err_p = err_p;
    prev_err_y = err_y;
}

void Ctrl::stopMotor()
{
    analogWrite(MTR_1, 0);
    analogWrite(MTR_2, 0);
    analogWrite(MTR_3, 0);
    analogWrite(MTR_4, 0);
}

void Ctrl::arm()
{
    state_arm = true;
}

void Ctrl::disarm()
{
    state_arm = false;
}

void Ctrl::setTarget(double throttle, double roll, double pitch, double yaw)
{
    t_tar = map(throttle, 1000, 2000,           0,        255);
    r_tar = map(roll,     1000, 2000, -CTRL_MAX_R, CTRL_MAX_R);
    p_tar = map(pitch,    1000, 2000, -CTRL_MAX_P, CTRL_MAX_P);
    y_tar = map(yaw,      1000, 2000, -CTRL_MAX_Y, CTRL_MAX_Y);
}
