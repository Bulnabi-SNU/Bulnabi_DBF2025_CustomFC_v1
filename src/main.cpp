#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "GNC/Navi.h"
#include "GNC/Ctrl.h"
#include "R8FM.h"

Navi navi;
Ctrl ctrl(navi);
uint32_t prev_ms_log = 0;
String buf_serial = "";
R8FM r8fm(PPM_INT, PPM_CH);

void processSerialInput(String input) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, input);

    if (!error)
    {
        if (doc["cmd_cali"].is<int>()) {
            int cmdCali = doc["cmd_cali"];
            if (cmdCali) {
                Serial.println("Calibration Start.");
                navi.calibrate();
                Serial.println("Calibration Done.");
            }
        }
        else {
            Serial.println("Unknown Command");
        }
    }
    else {
        Serial.print("Invalid JSON.");
        Serial.print(error.f_str());
        Serial.println();
    }

    buf_serial = "";
}

/* ================================
loop
    if conn & arm
        if autopilot
            유도명령 -> (목표자세) -> 제어기
        if manual
            조종기값 -> (목표자세) -> 제어기
    else
        kill : pusher


    always
        navigation update : "compute attitude" with "imu"
                            "compute position" with "gps"
            imu comm (100Hz)
            gps comm (10Hz)
        
        guidance   update : compute "required attitude" to "achieve waypoint" with "attitude, position"
            mission & sequence handling
        
        controller update : compute "required input for actuators" to "achieve attitude" with ""
            pusher control
            servo control
        
        peripheral
            logging (internal memory)
            telemetry (to GCS)
================================ */

void setup()
{
    navi.init();
}

void loop()
{
    navi.loop();
    r8fm.loop();

    if (r8fm.isConnected() && r8fm.isConnected()) {
        ctrl.arm();
    }
    else {
        ctrl.disarm();
    }

    if (r8fm.getMode() == 2) {  // 2, Mission Mode
        // TODO : implement guidance algorithm & waypoint sequence handler
        // guid.loop();
        // ctrl.setTarget(THROTTLE_COMMAND_FROM_GUIDANCE,
        //                Roll_____COMMAND_FROM_GUIDANCE,
        //                PITCH____COMMAND_FROM_GUIDANCE,
        //                YAW______COMMAND_FROM_GUIDANCE);
        // ctrl.loop();
    }
    else if (r8fm.getMode() == 1) { // 1, Gyro Stabilized Mode
        ctrl.setTarget(r8fm.getThrottleTarget(),
                       r8fm.getRollTarget(),
                       r8fm.getPitchTarget(),
                       r8fm.getYawTarget());
        ctrl.loop();
    }
    else if (r8fm.getMode() == 0) { // 0, Full Manual Mode
        ctrl.updateMotor(r8fm.getRawThrottleTarget(),
                         r8fm.getRawRollTarget(),
                         r8fm.getRawPitchTarget(),
                         r8fm.getRawYawTarget());
    }
    else {  // Out of State Machine, 대책없음
        
    }





    uint32_t curr_ms = millis();
    if (curr_ms - prev_ms_log >= 100) {
        prev_ms_log = curr_ms;

        JsonDocument doc;
        doc["mx"] = navi.getRawMagX();
        doc["my"] = navi.getRawMagY();
        doc["mz"] = navi.getRawMagZ();
        doc["r"]  = navi.getR();
        doc["p"]  = navi.getP();
        doc["y"]  = navi.getY();

        doc["T"]  = r8fm.getThrottleTarget();
        doc["A"]  = r8fm.getRollTarget();
        doc["E"]  = r8fm.getPitchTarget();
        doc["R"]  = r8fm.getYawTarget();
        doc["conn"] = r8fm.isConnected();
        doc["arm"]  = r8fm.isArmed();
        doc["mode"] = r8fm.getMode();

        doc["ctrl_T"] = r8fm.getRawThrottleTarget();
        doc["ctrl_A"] = r8fm.getRawRollTarget();
        doc["ctrl_E"] = r8fm.getRawPitchTarget();
        doc["ctrl_R"] = r8fm.getRawYawTarget();

        serializeJson(doc, Serial);
        Serial.println();
    }

    while (Serial.available()) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n') {
            processSerialInput(buf_serial);
        }
        else {
            buf_serial += incomingChar;
        }
    }
}
