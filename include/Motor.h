#include<Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


class Driver{
    public:
        int OscillatorFrequency;
        int PWMFrequency;
        Driver(int Oscillator = 27000000,int PWMFreq = 1600){
            OscillatorFrequency = Oscillator;
            PWMFrequency = PWMFreq;
            pwm.begin();
            pwm.setOscillatorFrequency(OscillatorFrequency);
            pwm.setPWMFreq(PWMFrequency);
        };
        void setSpeed(int port, int persent){
            pwm.setPWM(port, 0, persent/100*4098);
        };
};