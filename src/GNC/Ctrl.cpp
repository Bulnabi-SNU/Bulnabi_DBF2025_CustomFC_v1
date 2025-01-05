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
    pinMode(MTR_PUSH,     OUTPUT);
    pinMode(MTR_AILERON,  OUTPUT);
    pinMode(MTR_ELEVATOR, OUTPUT);
    pinMode(MTR_RUDDER,   OUTPUT);
    stopMotor();
}

void Ctrl::loop()
{
    uint32_t curr_ms = millis();
    if (curr_ms - prev_ms_ctrl >= 10) {
        if (state_arm)
        {
            updateController();
        }
        else
        {
            stopMotor();
        }
        
        prev_ms_ctrl = curr_ms;
    }
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
    t_tar = throttle;
    r_tar = roll;
    p_tar = pitch;
    y_tar = yaw;
}

void Ctrl::updateController()
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

    // 믹싱
    double m_push     = t_tar;
    double m_aileron  = control_r;
    double m_elevator = control_p;
    double m_rudder   = control_y;

    // TODO : 이거는 DC모터 제어. pwm을 esc 및 서보와 맞춰야함
    // 모터 출력 범위 제한 (0 ~ 255)
    m_push     = constrain(m_push,     0, 255);
    m_aileron  = constrain(m_aileron,  0, 255);
    m_elevator = constrain(m_elevator, 0, 255);
    m_rudder   = constrain(m_rudder,   0, 255);

    // 이전 값 업데이트
    prev_err_r = err_r;
    prev_err_p = err_p;
    prev_err_y = err_y;

    updateMotor(m_push, m_aileron, m_elevator, m_rudder);
}

void Ctrl::updateMotor(double m_push, double m_aileron, double m_elevator, double m_rudder)
{
    if (state_arm) { analogWrite(MTR_PUSH, (int)m_push); }
    else { stopMotor(); }
    analogWrite(MTR_AILERON,  (int)m_aileron);
    analogWrite(MTR_ELEVATOR, (int)m_elevator);
    analogWrite(MTR_RUDDER,   (int)m_rudder);
}

void Ctrl::stopMotor()
{
    analogWrite(MTR_PUSH, 0);
}
