#ifndef _MOTOR_H
#define _MOTOR_H

class motor_c {
  public:
    motor_c(int which_pwm_pin, int which_dir_pin);
    void motorDriver(float what_power);
  private:
    int pwm_pin;
    int dir_pin;
    int power;  
};

motor_c::motor_c(int which_pwm_pin, int which_dir_pin) {
  pwm_pin = which_pwm_pin;
  dir_pin = which_dir_pin;
  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
};
    
void motor_c::motorDriver(float what_power) {
  power = what_power;
  //when input power is negative, chang the motor direction
  //then make the power to be the absoulte value
  if (power>0){
    digitalWrite(dir_pin, LOW);
    analogWrite(pwm_pin, power);  
  }else{
    digitalWrite(dir_pin, HIGH);
    analogWrite(pwm_pin, abs(power));
  }
};
#endif
