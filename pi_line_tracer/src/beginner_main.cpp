#include <Arduino.h>

// IO ピン
const int PIN_PWMA = 5;   // モータAの回転速度を決めるPWM信号を送るピン(左)
const int PIN_PWMB = 6;   // モータBの回転速度を決めるPWM信号を送るピン(右)
const int PIN_AIN1 = 9;   // モータAの回転方向を決める信号を送るピンその1
const int PIN_AIN2 = 8;   // モータAの回転方向を決める信号を送るピンその2
const int PIN_BIN1 = 10;  // モータBの回転方向を決める信号を送るピンその1
const int PIN_BIN2 = 11;  // モータBの回転方向を決める信号を送るピンその2

// センサ
const int PIN_SENSOR_LEFT = A5;
const int PIN_SENSOR_RIGHT = A4;

// 制御用の定数
const int BASE_SPEED = 65;
const int MAX_SPEED = 100;
const int MIN_SPEED = 0;
const int SENSOR_THRESHOLD = 100;  // 白線の閾値

// PID ゲイン
const float Kp = 3;  // 比例ゲイン (0.1 - 1.0)
const float Ki = 0.018;   // 積分ゲイン (0.01 - 0.1)
const float Kd = 0.437;   // 微分ゲイン (0.1 - 0.5)

// PID 制御用
float integral = 0;
float previousError = 0;
const float MAX_INTEGRAL =
    300;  // 積分値の最大値 (Kp*errorが取りうる値の数倍など)

void setup() {
  // シリアル通信を開始（デバッグ用・通信速度を 115200bps に設定）
  // Serial.begin(115200);

  // IOピンの入出力モード設定
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_SENSOR_LEFT, INPUT);
  pinMode(PIN_SENSOR_RIGHT, INPUT);

  // 起動してすぐに暴走しないように初期状態をモータA/モータBいずれも停止で確定させる（これを初期化という）
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);

  // モータの回転速度を設定しておく。ここで必ずしなければならないわけではない
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);

  // すぐに動き出さないように 1 秒待つ
  // delay(10);
  // Serial.println("PID Line Tracer Ready!");
}

/**
 * @brief モータを回転させる
 *
 * @param motor モータの番号（0: モータ A（左）, 1: モータ B（右））
 * @param direction 回転方向（0: 正転, 1: 後転）
 * @param speed 回転速度（0~255）
 */
void rotateMotor(int motor, int direction, int speed) {
  // 速度を0-255の範囲に制限
  speed = constrain(speed, 0, 255);

  if (motor == 0) {        // モータA (左)
    if (direction == 0) {  // 正転
      digitalWrite(PIN_AIN1, HIGH);
      digitalWrite(PIN_AIN2, LOW);
    } else {  // 後転
      digitalWrite(PIN_AIN1, LOW);
      digitalWrite(PIN_AIN2, HIGH);
    }
    analogWrite(PIN_PWMA, speed);  // モータAの回転速度を設定
  } else {                         // モータB (右)
    if (direction == 0) {          // 正転
      digitalWrite(PIN_BIN1, LOW);
      digitalWrite(PIN_BIN2, HIGH);
    } else {  // 後転
      digitalWrite(PIN_BIN1, HIGH);
      digitalWrite(PIN_BIN2, LOW);
    }
    analogWrite(PIN_PWMB, speed);  // モータBの回転速度を設定
  }
}

void loop() {
  // センサからの電圧を 0~1023 の 1024 段階で読み取る
  // 数字が大きければ大きいほど、センサが白を検知していることを示す
  // 実測で白が 200、黒が 0 付近の場合を想定
  int sensLeft = analogRead(PIN_SENSOR_LEFT);
  int sensRight = analogRead(PIN_SENSOR_RIGHT);

  // デバッグ用シリアル出力
  // Serial.print("L:");
  // Serial.print(sensLeft);
  // Serial.print(",R:");
  // Serial.print(sensRight);

  // PID制御の計算
  // 誤差(error)の定義：
  // ラインが中央にあるとき、sensLeft と sensRight は近い値になる
  // (理想は白い線の上で両方高い値)。 右にずれた場合 (車体が右、白い線が左):
  // 左センサーが白(高)、右センサーが黒(低) -> error は正・左に曲がる
  // 左にずれた場合 (車体が左、白い線が右):
  // 左センサーが黒(低)、右センサーが白(高) -> error は負・右に曲がる
  int error = sensLeft - sensRight;

  // 積分項 (I)
  integral = constrain(integral + error, -MAX_INTEGRAL, MAX_INTEGRAL);
  // integral += error;

  // 微分項 (D)
  float derivative = error - previousError;

  // 制御量 (correction) の計算 (PID)
  // P項: 現在の誤差に比例した制御
  // I項: 過去の誤差の蓄積に基づいた制御（定常偏差をなくす効果）
  // D項: 誤差の変化率に基づいた制御（振動を抑え、応答を速くする効果）
  // TODO: Kd * derivative を引き算に変える？
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // 次のループのために現在のエラーを保存
  previousError = error;

  // モーター速度の計算
  int motorSpeedLeft = BASE_SPEED - correction;
  int motorSpeedRight = BASE_SPEED + correction;

  // デバッグ用シリアル出力
  // Serial.print(" | E:");
  // Serial.print(error);
  // Serial.print(",I:");
  // Serial.print(integral);
  // Serial.print(",D:");
  // Serial.print(derivative);
  // Serial.print(",C:");
  // Serial.print(correction);

  // 両方が白線の場合（クロス部分）
  if (sensLeft >= SENSOR_THRESHOLD && sensRight >= SENSOR_THRESHOLD) {
    // Serial.print(" | Both White -> Straight");
    rotateMotor(0, 0, BASE_SPEED);
    rotateMotor(1, 0, BASE_SPEED);

    // I が大きくなりすぎないようにリセット
    integral = 0;
  }
  // 両方が黒の場合（直線部分）
  // else if (sensLeft < SENSOR_THRESHOLD && sensRight < SENSOR_THRESHOLD) {
  //   Serial.print(" | Both Black -> Speed up");
  //   rotateMotor(0, 0, MAX_SPEED);
  //   rotateMotor(1, 0, MAX_SPEED);
  // }
  else {
    // PID 制御
    // Serial.print(" | PID Control");

    // 速度を MIN_SPEED から MAX_SPEED の範囲に制限
    motorSpeedLeft = constrain(motorSpeedLeft, MIN_SPEED, MAX_SPEED);
    motorSpeedRight = constrain(motorSpeedRight, MIN_SPEED, MAX_SPEED);

    rotateMotor(0, 0, motorSpeedLeft);   // モータ A (左) を計算された速度で正転
    rotateMotor(1, 0, motorSpeedRight);  // モータ B (右) を計算された速度で正転
  }

  // Serial.print(" | SL:");
  // Serial.print(motorSpeedLeft);
  // Serial.print(",SR:");
  // Serial.println(motorSpeedRight);

  delay(10);  // 制御周期 (10ミリ秒)
}
