#include <A4988.h>
#include <MsTimer2.h>

// 定義されたピンと変数
#define ENC_A_PIN 2
#define ENC_B_PIN 3
#define LIMIT_PIN 4
#define DIR_PIN 5
#define STEP_PIN 6
const int MOTOR_STEPS = 200;
bool limit_status = false;
volatile long encCount = 0;
int num_steps = 0;
const float RPM_MAX = 470;
int microsteps = 2;
int acc_offset = 0;
int acc_val = 0;
long datanumber = 0;
int pos_cart = 0;
int state_cart = 0;
int rotate = 40;


// ステッピングモータ制御のインスタンス
A4988 stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN);

// 割り込みルーチンとその他の関数宣言
void encItrA();
void encItrB();
void calib_motor();
void calibAcc();
void obs();

void setup() {
  Serial.begin(115200);
  stepper.begin(30, microsteps);
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encItrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encItrB, CHANGE);
  calib_motor();
  delay(1000);
  calibAcc();
  stepper.setRPM(400);
  MsTimer2::set(100, obs);
  MsTimer2::start();
}

void loop() {

  if (pos_cart > 500 || pos_cart < -500) {
    state_cart = 1;
  }
  // シリアル通信でデータが利用可能か確認
  if (Serial.available() > 0) {
    int received = Serial.parseInt();
    if (received == 0) {  //左？（正）
      // PCからのコマンドに従ってモータを正の方向に動かす
      //stepper.setRPM(400);
      stepper.rotate(rotate);
      pos_cart += rotate;
    } else if (received == 1) {
      // PCからのコマンドに従ってモータを負の方向に動かす
      //stepper.setRPM(400);
      stepper.rotate(-rotate);
      pos_cart -= rotate;
    } else if (received == -1) {
      // リセットコマンドを受け取った場合、キャリブレーションを行う
      calib_motor();  // モータのキャリブレーションを実行
      calibAcc();     // 加速度センサのキャリブレーションを実行
      // キャリブレーションが完了したことをPCに通知
      //Serial.println("calibrat);
      // エンコーダのカウントをリセット
      encCount = 0;
      pos_cart = 0;
      state_cart = 0;
      datanumber = 0;
    }
    // 受信したコマンドが0、1、-1いずれでもない場合の処理
    // ... ここにコマンドに応じたその他の処理を追加
  }
}


//エンコーダA相割り込み
void encItrA() {
  encCount += digitalRead(ENC_A_PIN) == digitalRead(ENC_B_PIN) ? -1 : 1;
}
//エンコーダB相割り込み
void encItrB() {
  encCount += digitalRead(ENC_A_PIN) == digitalRead(ENC_B_PIN) ? 1 : -1;
}

void calibAcc(void) {
  long acc_sum = 0;
  for (int i = 0; i < 100; i++) {
    acc_sum += analogRead(A0);
  }
  acc_offset = acc_sum / 100;
}
void obs(void) {
  acc_val = analogRead(A0) - acc_offset;
  Serial.print(pos_cart);
  Serial.print(",");
  Serial.print(state_cart);
  Serial.print(",");
  Serial.print(datanumber);
  Serial.print(",");
  Serial.print(encCount);
  Serial.print(",");
  Serial.println(acc_val);

  datanumber++;
}

void calib_motor(void) {
  while (digitalRead(LIMIT_PIN) == HIGH) {

    stepper.setRPM(100);
    stepper.rotate(-10);
    delayMicroseconds(5000);
  }
  stepper.rotate(10);
  delay(1000);
  stepper.rotate(500);
  Serial.println("ce");
}