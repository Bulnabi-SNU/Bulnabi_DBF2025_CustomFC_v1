#ifndef CTRL_H
#define CTRL_H

#include <Arduino.h>
#include "GNC/Navi.h"

class Ctrl {
public:
    Ctrl(Navi& navi);

    void init();
    void loop();
    
    void arm();
    void disarm();

    void setTarget(double throttle, double roll, double pitch, double yaw);
    void updateMotor(double m_push, double m_aileron, double m_elevator, double m_rudder);

private:
    Navi& navi;

    bool state_arm;
    uint32_t prev_ms_ctrl;

    double t_tar;
    double r_tar;
    double p_tar;
    double y_tar;

    double itg_r;
    double itg_p;
    double itg_y;
    double drv_r;
    double drv_p;
    double drv_y;
    double prev_err_r;
    double prev_err_p;
    double prev_err_y;

    void updateController();
    void stopMotor();
};

#endif
