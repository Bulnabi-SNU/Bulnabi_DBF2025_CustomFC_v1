#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "GNC/Navi.h"

Navi navi;
uint32_t prev_ms_log = 0;
String buf_serial = "";

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
