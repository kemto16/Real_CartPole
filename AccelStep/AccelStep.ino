#include <AccelStepper.h>

AccelStepper stepper(1, 9, 8);

void setup(){
    Serial.begin(115200);
    stepper.setMaxSpeed(400);
    stepper.setAcceleration(200);
}

void loop(){
    //現在位置
    Serial.print("now position: ");
    Serial.println(stepper.currentPosition());
    stepper.runToNewPosition(0);
    delay(500);
    Serial.print("now position: ");
    Serial.println(stepper.currentPosition());
    stepper.runToNewPosition(200);
    delay(500);
}