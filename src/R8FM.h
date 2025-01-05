#ifndef R8FM_H
#define R8FM_H

#include <Arduino.h>
#include "PPMReader.h"

class R8FM {
private:
    PPMReader ppm;
    unsigned A, E, T, R;
    unsigned U1, U2, U3, U4;

    unsigned conn;
    unsigned armed;
    unsigned mode;

    double t_tar_rx;
    double r_tar_rx;
    double p_tar_rx;
    double y_tar_rx;

public:
    R8FM(int ppmPin, int channels);
    void updatePPM();
    double getThrottleTarget() const;
    double getRollTarget() const;
    double getPitchTarget() const;
    double getYawTarget() const;
    bool isConnected() const;
    bool isArmed() const;
    unsigned getMode() const;
};

#endif // R8FM_H