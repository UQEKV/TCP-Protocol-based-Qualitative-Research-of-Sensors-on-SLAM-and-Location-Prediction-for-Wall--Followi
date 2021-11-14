#include "error.h"
#include "motor_c.h"
#define Buzzer_Pin 6


error error_z(0);
error error_x(1);
error error_y(2);
error error_zz(3);

motor_c l_motor( PWM_L, DIR_L );
motor_c r_motor( PWM_R, DIR_R );

float last_gz;
float last_ax;
float last_ay;

float error_gz;
float error_ax;
float error_ay;
float error_az;

float angle;
float speed_x;
float speed_y;

unsigned long timeStamp = 0;
unsigned long elapsedTime = 0;
unsigned long timeNow = 0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  imu.read();

  error_gz = error_z.error_cal();
  error_ax = error_x.error_cal();
  error_ay = error_y.error_cal();
  error_az = error_zz.error_cal();

  CountDown(3, 1);
}

void loop()
{
  l_motor.setPower( 0 );
  r_motor.setPower( 0 );
  imu.read();

  float gz = lowpass_filter( imu.g.z - error_gz, last_gz, 0.1 );
  float ax = lowpass_filter( imu.a.x - error_ax, last_ax, 0.1 );
  float ay = lowpass_filter( imu.a.y - error_ay, last_ay, 0.1 );

  last_gz = gz;
  last_ax = ax;
  last_ay = ay;

  Serial.print(imu.a.z);
  Serial.print(",");
  Serial.print(imu.a.x);
  Serial.print(",");
  Serial.print(imu.a.y);
  Serial.print(",");
  Serial.print(imu.g.z);
  Serial.print(",");
  Serial.print(imu.g.x);
  Serial.print(",");
  Serial.println(imu.g.y);

  if ( millis() - timeStamp  > 50) {
    timeStamp = millis();
    angle += gz / 1000 * 8.75 * 50 / 1000;

    //    speed_x += ax / 1000 * 0.061 * 9.8 * 50 / 1000;
    //
    //    speed_y += ay / 1000 * 0.061 * 9.8 * 50 / 1000;

    //   Serial.println(angle);

  }
}

float lowpass_filter(float reading, float last_output, float alpha) {

  last_output = ( alpha * reading ) + ( (1 - alpha) * last_output );

  return last_output;
}


void CountDown(int times, int volume) {
  pinMode(Buzzer_Pin, OUTPUT);
  for (int i = 0; i < times; i++)
  {
    analogWrite(Buzzer_Pin, volume);
    delay(100);
    analogWrite(Buzzer_Pin, 0);
    delay(100);
  }
}
