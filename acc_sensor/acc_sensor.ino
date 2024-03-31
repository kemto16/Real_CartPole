
int x_sum = 0;
int x_offset = 0;
int x_calib;


void setup(){
    Serial.begin(115200);
    for(int i=0; i<10; i++){
        x_sum += analogRead(A0);
    }
    x_offset = x_sum / 10;
}

void loop(){
    x_calib = analogRead(A0) - x_offset;
    
    Serial.println(x_calib);

    delay(100);

}