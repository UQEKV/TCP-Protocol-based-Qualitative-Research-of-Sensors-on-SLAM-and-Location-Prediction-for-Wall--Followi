#ifndef _MOTOR_C_H
#define _MOTOR_C_H

#define PWM_L 10
#define PWM_R 9
#define DIR_L 16
#define DIR_R 15
#define PWM_MAX 100
#define PWM_MIN 0

class motor_c {
  public:
    int pwm_pin;
    int dir_pin;
    motor_c( int pwm_pin_in, int dir_pin_in );
    void setPower( float power_in );
};

motor_c::motor_c( int pwm_pin_in, int dir_pin_in ) {
  pwm_pin = pwm_pin_in;
  dir_pin = dir_pin_in;

  pinMode( pwm_pin, OUTPUT );
  pinMode( dir_pin, OUTPUT );

  digitalWrite( pwm_pin, LOW );
  digitalWrite( dir_pin, LOW );
}

void motor_c::setPower( float power_in ) {

  // check direction
  if ( power_in < 0) {
    digitalWrite( dir_pin, HIGH);// backwards
  } else if ( power_in > 0) {
    digitalWrite( dir_pin, LOW);// forwards
  } else {
    // nothing to do here!
  }
  if ( power_in > PWM_MAX ) {
    power_in = PWM_MAX;
  }

  if ( power_in < 0 ) {
    power_in *= -1 ; // power_in = abs( power_in );
  }
  analogWrite( pwm_pin, power_in );
}
#endif
