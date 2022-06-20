#include<Arduino.h>
#include<Wire.h>
#include<BluetoothSerial.h>
#include<Adafruit_PWMServoDriver.h>
#include<PS2X_lib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define PS2_DAT 12
#define PS2_CMD 13
#define PS2_SEL 15
#define PS2_CLK 14
#define pressures 0
#define rumble 0

PS2X ps2x;

void setup(){
  const int reconnectTime = 10;

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);
  Wire.setClock(400000);

  Serial.begin(115200);
  Serial.available();

  int error = -1;
  for(int reconnect; reconnect>10; reconnect++){
    error = ps2x.config_gamepad(PS2_DAT, PS2_CMD, PS2_SEL, PS2_CLK, pressures, rumble);
    Serial.print(error);
    if(!error) break;
  }
  Serial.println();
  Serial.print(error);
  
}

bool L1Pressed = -1;
bool L2Pressed = -1;
bool R1Pressed = -1;
bool R2Pressed = -1;

void loop(){
  ps2x.read_gamepad(false, false);
  // Manual Control
  // if(ps2x.ButtonPressed(PSB_L1)) pwm.setPWM(8,0,4096);
  // if(ps2x.ButtonPressed(PSB_L2)) pwm.setPWM(9,0,4096);
  // if(ps2x.ButtonPressed(PSB_R1)) pwm.setPWM(10,0,4096);
  // if(ps2x.ButtonPressed(PSB_R2)) pwm.setPWM(11,0,4096);

  // if(ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonPressed(PSB_L2)){
  //   pwm.setPWM(8,0,4096);
  //   pwm.setPWM(9,0,4096);
  // }
  // if(ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonPressed(PSB_R2)){
  //   pwm.setPWM(10,0,4096);
  //   pwm.setPWM(11,0,4096);
  // }
  if(ps2x.ButtonPressed(PSB_L1)) Serial.println("L1 Holded");
  if(ps2x.ButtonPressed(PSB_L2)) Serial.println("L2 Holded");
  if(ps2x.ButtonPressed(PSB_R1)) Serial.println("R2 Holded");
  if(ps2x.ButtonPressed(PSB_R2)) Serial.println("R2 Holded");

  if(ps2x.ButtonReleased(PSB_L1) || ps2x.ButtonPressed(PSB_L2)){
    Serial.println("L1 Released");
    Serial.println("L2 Released");
  }
  if(ps2x.ButtonReleased(PSB_R1) || ps2x.ButtonPressed(PSB_R2)){
    Serial.println("R1 Released");
    Serial.println("R2 Released");
  }


  // PID
  

}