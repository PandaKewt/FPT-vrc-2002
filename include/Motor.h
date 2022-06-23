#include<Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void motorInit() {
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(1600);
    Wire.setClock(400000);
}

void setPWM(uint16_t c1, uint16_t c2, uint16_t c3, uint16_t c4) {
    pwm.setPWM()
}