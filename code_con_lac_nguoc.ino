#define IN1 5
#define IN2 4
#define IN3 6
#define IN4 7
#define ENA 9 // phai
#define ENB 3 // trai

const byte dungyen = 0;
const byte tien = 1;
const byte lui = 2;
float dt = 0.01; //delay(10)
float sum_err_goc = 0;
float last_err_goc = 0;

void test_motor(byte chieu, int v) { //kiem tra toc do dong co DC
  if (v >= 255) v = 255;
  if (v <= -255) v = -255;
  switch (chieu) {
    case 0:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, v);
      analogWrite(ENB, v);
      break;
    case 1:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, v);
      analogWrite(ENB, v);
      break;
    case 2:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, v);
      analogWrite(ENB, v);
      break;
    default:
      break;
  }
}

void control_wheel(int v) { //dieu khien cung luc 2 dong co DC
  if (v >= 255) v = 255;
  if (v <= -255) v = -255;
  if (v >= 0) { //v > 0 -> tien; v < 0 -> lui
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, v);
    analogWrite(ENB, v);
  }
  else {
    v = -v;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, v);
    analogWrite(ENB, v);
  }
}
void control_wheel_left(int v) { //dieu khien dong co DC trai
  if (v >= 255) v = 255;
  if (v <= -255) v = -255;
  if (v >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, v);
  }
  else {
    v = -v;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, v);
  }
}
void control_wheel_right(int v) { //dieu khien dong co DC phai
  if (v >= 255) v = 255;
  if (v <= -255) v = -255;
  if (v >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, v);
  }
  else {
    v = -v;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, v);
  }
}

float PID(float error, float kp, float ki, float kd, float& sum_err, float& last_err) { //ham PID
  sum_err += error * dt;
  float p = kp * error;
  float i = ki * sum_err;
  float d;
  if (last_err == 0){
    d = 0;
  }
  else{
    d = kd * (error - last_err) / dt;
  }
  last_err = error;
  float ut = p + i + d;
  return ut;
}

void test_bien_tro() { //kiem tra bien tro
  int value = analogRead(A0);
  Serial.println(value);
  int goc;
  goc = map(value, 217 ,615, 0, 90); //quy doi gia tri bien tro thanh gia tri goc
  Serial.println(goc);
  Serial.println();
  delay(200);
}

void con_lac_nguoc(float threshold) { //chuong trinh cua con lac
  float value = analogRead(A0);
  float x;
  x = map(value, 217, 615, 0, 90);
  float error = x - threshold;
  int v;
  if(x < 30 || x > 120 || x >89 && x < 91){
    control_wheel(0); //xe dung yen
  }
  else{
    v = (int)PID(error, 300, 0.1, 15, sum_err_goc, last_err_goc);
    control_wheel_left(v);
    control_wheel_right(v);
  }
  Serial.println(v);
  delay(10);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
//  control_wheel(255);
//  test_bien_tro();
  con_lac_nguoc(90);
}
