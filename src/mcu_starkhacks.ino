#include <SCServo.h>
#include "Arduino_RouterBridge.h"

const int hall_sensor = A0;
SMS_STS st;
const int enable_pin = 5;
const int in1 = 4;  
const int in2 = 3;  

void setup() {
    pinMode(enable_pin, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    Serial.begin(1000000);
    st.pSerial = &Serial;
    delay(3000);

    Bridge.begin();
    Bridge.provide("grasp", pinch);
    Bridge.provide("suction", suction);  
    Bridge.provide("get_pos", get_pos);
    Bridge.provide("read_hall_value", read_hall_value);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    delay(1000);
}

void loop() {}

float read_hall_value() {
    int M = analogRead(hall_sensor);  // raw hall value
    float F = -0.016 * M + 11.4;      // calibrated force (N)
    return F;
}

void pinch(int position) {
    st.WritePosEx(1, position, 1500, 50);
}

void suction(int voltage) {
    voltage = constrain(voltage, 0, 255);
    analogWrite(enable_pin, voltage);  
    delay(100);
}

int get_pos() {
    return st.ReadPos(1);
}