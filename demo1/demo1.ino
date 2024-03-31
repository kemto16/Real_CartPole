#include <A4988.h>
 
const int MOTOR_STEPS = 200;
const int DIR  =  2;
const int STEP =  3;

float rpm      = 470;
int microsteps = 1;

A4988 stepper(MOTOR_STEPS, DIR, STEP);

void setup() {
    Serial.begin(115200);
    stepper.begin(rpm, microsteps);
}
 
void loop() {
    int speed = analogRead(A0);
    for(int i=0; i<3; i++){
        stepper.rotate(360*3);
        delay(5);

        stepper.rotate(-360*3);
        delay(5);
    }

    for(int i=0; i<1; i++){
        stepper.rotate(-360*2);
        delay(5);

        stepper.rotate(-360*2);
        delay(5);
    }

    stepper.rotate(1);
    delay(10000);
}

// 値をマッピングする関数
int mapValue(int inputValue) {
    // 入力値と出力値の範囲を定義
    int inputMin = 0, inputMax = 1023;
    int outputMin = 0, outputMax = 470;

    // マッピング処理
    int outputValue = (inputValue - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;

    return outputValue;
}