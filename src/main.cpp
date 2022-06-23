#include<Arduino.h>
#include<Wire.h>
#include<Adafruit_PWMServoDriver.h>
#include<PS2X_lib.h>
#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define pressures false
#define rumble false
#define leftSensor 0
#define leftMiddleSensor 2
#define middleSensor 39
#define rightMiddleSensor 23
#define rightSensor 32

PS2X ps2x;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void motorInit() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  // Xóa nếu mạch ko chạy
  Wire.setClock(400000);
}
void ps2Init() {
  int error = -1;
  for (int i = 0; i < 10; i++) // thử kết nối với tay cầm ps2 trong 10 lần
  {
    delay(1000); // đợi 1 giây
    // cài đặt chân và các chế độ: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }

  switch (error) // kiểm tra lỗi nếu sau 10 lần không kết nối được
  {
  case 0:
    Serial.println(" Ket noi tay cam PS2 thanh cong");
    break;
  case 1:
    Serial.println(" LOI: Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
    break;
  case 2:
    Serial.println(" LOI: khong gui duoc lenh");
    break;
  case 3:
    Serial.println(" LOI: Khong vao duoc Pressures mode ");
    break;
  }
}
void setPWM(int portOn, int portOff, int value){
    if(value != 0) {
      pwm.setPWM(portOff, 4096, 0);
      pwm.setPWM(portOn, 0, value);
    }
    if(value == 0) {
      pwm.setPWM(portOff, 0, 0);
      pwm.setPWM(portOn, 0, 0);
    }
}
bool autonomousFinish() {
  return (digitalRead(leftSensor) && digitalRead(leftMiddleSensor) && digitalRead(middleSensor) && digitalRead(rightMiddleSensor) && digitalRead(rightSensor)) || (!(digitalRead(leftSensor) && digitalRead(leftMiddleSensor) && digitalRead(middleSensor) && digitalRead(rightMiddleSensor) && digitalRead(rightSensor)));
}
void autoExit() {
  if(autonomousFinish() == 0){
    setPWM(9, 8, 0);
    setPWM(11, 10, 0);
  }
  else {
    setPWM(9, 8, 4000);
    setPWM(11, 10, 4000);
    autoExit();
  }
}
void ps2Controller() {
  if(ps2x.ButtonPressed(PSB_L1))  setPWM(8, 9, 4000);
  if(ps2x.ButtonPressed(PSB_L2))  setPWM(9, 8, 4000);
  if(ps2x.ButtonPressed(PSB_R1))  setPWM(10, 11, 4000);
  if(ps2x.ButtonPressed(PSB_R2))  setPWM(11, 10, 4000);
  if(ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_L2))  setPWM(8, 9, 0);
  if(ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonReleased(PSB_R2))  setPWM(10, 11, 0);
}
void setup(){
  motorInit();
  ps2Init();
}
void loop(){
  ps2x.read_gamepad(false, false);
  ps2Controller();
	delay(50);
} 	
