#include<Arduino.h>
#include<Wire.h>
#include<Adafruit_PWMServoDriver.h>
#include<PS2X_lib.h>
#include <PID_v1.h>

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

uint8_t speedAutonomous = 2048;
double setPoint = 0, input, output,
			 Kp=20, Ki=0.04555555, Kd=11.898989;
int sensor;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
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
void autonomousInit() {
  pinMode(leftSensor,input);
  pinMode(leftMiddleSensor,input);
  pinMode(middleSensor,input);
  pinMode(rightMiddleSensor,input);
  pinMode(rightSensor,input);
  input=0; // mặc định line giữa =0, => luôn luôn điều chỉnh xe để input=0
  
  myPID.SetSampleTime(1); // thời gian lấy mẫu phụ thuộc tốc độ xe, lấy mẫu càng nhanh càng tốt
  myPID.SetMode(AUTOMATIC); 
  myPID.SetoutputLimits(-speedAutonomous,speedAutonomous); // giá trị tốc độ, -speed tức bánh bên trái quay max, bên phải ngừng quay
  
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
//Autonormous
bool autonomousFinish() {
  return (digitalRead(leftSensor) && digitalRead(leftMiddleSensor) && digitalRead(middleSensor) && digitalRead(rightMiddleSensor) && digitalRead(rightSensor)) || (!(digitalRead(leftSensor) && digitalRead(leftMiddleSensor) && digitalRead(middleSensor) && digitalRead(rightMiddleSensor) && digitalRead(rightSensor)));
}
void motorControl(int16_t duty_value)
{
  int16_t speed_a, speed_b;
  int speed_zero;
  speed_zero = speed_robot/2;
  if(duty_value>1) 
  {
    speed_b=-speed_zero;
    speed_a=duty_value;
  }
  else if(duty_value==0) 
  {
    speed_a=speed_b=0;
    
  }
  else if(duty_value<-1) 
  {
    speed_a=-speed_zero;
    speed_b=-duty_value;
  } 
    setPWM(8, 8, speed_b + speed_zero);
    setPWM(10, 11, speed_a + speed_zero);
}     
void scan_sensor() {
  if(digitalRead(rightSensor)==1) sensor=4;
  else if((digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==1)) sensor=3;
  else if(digitalRead(rightMiddleSensor)==1) sensor=2;
  else if((digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==1)) sensor=1;
  else if(digitalRead(middleSensor)==1) sensor=0; // vào line giữa
  else if((digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==1)) sensor=-1;
  else if(digitalRead(leftMiddleSensor)==1) sensor=-2;
  else if((digitalRead(leftSensor)==1)&&(digitalRead(leftMiddleSensor)==1)) sensor=-3;
  else if(digitalRead(leftSensor)==1) sensor=-4;
}
void autoExit() {
  if(autonomousFinish() == 0){
    setPWM(9, 8, 0);
    setPWM(11, 10, 0);
  } else if(ps2x.ButtonPressed(PSB_L3)) 
  else {
    setPWM(9, 8, 4000);
    setPWM(11, 10, 4000);
    autoExit();
  }
}
void pid() {
	if(autonomousFinish()) return autoExit();
	Setpoint=0;
  scan_sensor(); // đọc dữ liệu từ cảm biến
  Input = sensor;
  myPID.Compute(); // tính toán
  motorControl(Output); // điều khiển bánh xe để robot đi đúng đường
	return pid();
}
void pidStartup() {
	
	pid();
}
//Manual
void ps2Controller() {
  if(ps2x.ButtonPressed(PSB_L1))  setPWM(8, 9, 4000);
  if(ps2x.ButtonPressed(PSB_L2))  setPWM(9, 8, 4000);
  if(ps2x.ButtonPressed(PSB_R1))  setPWM(10, 11, 4000);
  if(ps2x.ButtonPressed(PSB_R2))  setPWM(11, 10, 4000);
	if(ps2x.ButtonPressed(PSP_PAD_UP)) setPWM(12, 13, 4000);
	if(ps2x.ButtonPressed(PSP_PAD_DOWN)) setPWM(13, 12, 4000);
	if(ps2x.ButtonPressed(PSP_TRIANGLE)) setPWM(14, 15, 4000);
	if(ps2x.ButtonPressed(PSP_CROSS)) setPWM(15, 14, 4000);
	if(ps2x.ButtonPressed(PSP_START)) pid();
  if(ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonReleased(PSB_L2))  setPWM(8, 9, 0);
  if(ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonReleased(PSB_R2))  setPWM(10, 11, 0);
  if(ps2x.ButtonReleased(PSP_PAD_UP) || ps2x.ButtonReleased(PSP_PAD_DOWN))  setPWM(12, 13, 0);
  if(ps2x.ButtonReleased(PSP_TRIANGLE) || ps2x.ButtonReleased(PSP_CROSS))  setPWM(12, 13, 0);
}
void setup(){
  motorInit();
  ps2Init();
	autonomousInit();
}
void loop(){
  ps2x.read_gamepad(false, false);
  ps2Controller();
	delay(50);
} 	
