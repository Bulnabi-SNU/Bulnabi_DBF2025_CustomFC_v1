#include "R8FM.h"

R8FM::R8FM(int ppmPin, int channels) 
    : ppm(ppmPin, channels),
      A(0), E(0), T(0), R(0),
      U1(0), U2(0), U3(0), U4(0),
      conn(0), armed(0), mode(0),
      t_tar_rx(0), r_tar_rx(0), p_tar_rx(0), y_tar_rx(0) {}

void R8FM::updatePPM() {
    A  = ppm.latestValidChannelValue(1, 0);
    E  = ppm.latestValidChannelValue(2, 0);
    T  = ppm.latestValidChannelValue(3, 0);
    R  = ppm.latestValidChannelValue(4, 0);
    U1 = ppm.latestValidChannelValue(5, 0);
    U2 = ppm.latestValidChannelValue(6, 0);
    U3 = ppm.latestValidChannelValue(7, 0);
    U4 = ppm.latestValidChannelValue(8, 0);
    
    // Connection Check
    conn = (T == 0) ? 0 : 1;

    // Arming Check
    armed = (U3 > 1500) ? 1 : 0;
    if (!conn) { armed = 0; }

    // Mode Check   // TODO : mode index determine required
    if                    (U1 < 1250) { mode = 0; } // Maybe Full Manual
    else if (1250 <= U1 && U1 < 1750) { mode = 1; } // Maybe Gyro Stabilized
    else                              { mode = 2; } // Maybe Mission
    if (!conn) { mode = 0; }

    // Mixing
    if (conn && armed) { t_tar_rx = map(T, 1000, 2000, 0, 100); }
    else               { t_tar_rx = 0; }
    r_tar_rx = map(A, 1000, 2000, -20, 20);
    p_tar_rx = map(E, 1000, 2000, -20, 20);
    y_tar_rx = map(R, 1000, 2000, -20, 20);
}

double R8FM::getThrottleTarget() const {
    return t_tar_rx;
}

double R8FM::getRollTarget() const {
    return r_tar_rx;
}

double R8FM::getPitchTarget() const {
    return p_tar_rx;
}

double R8FM::getYawTarget() const {
    return y_tar_rx;
}

bool R8FM::isConnected() const {
    return conn == 1;
}

bool R8FM::isArmed() const {
    return armed == 1;
}

unsigned R8FM::getMode() const {
    return mode;
}
