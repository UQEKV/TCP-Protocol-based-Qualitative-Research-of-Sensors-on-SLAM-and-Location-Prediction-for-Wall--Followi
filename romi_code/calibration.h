#ifndef _ERROR_H
#define _ERROR_H

#include <Wire.h>
#include <LSM6.h>
class error {
  public:
    float ini_val1;
    float ini_val;
    float sensor_read;
    error(int port);
    float error_cal();
  private:
    int port_imu;
};

error::error( int port) {
  port_imu = port;
}

float error::error_cal()
{
  for (int i = 0; i < 100; i++) {
    imu.read();
    if (port_imu == 0) sensor_read=imu.g.z;
    if (port_imu == 1) sensor_read=imu.a.x;
    if (port_imu == 2) sensor_read=imu.a.y;
    ini_val1 = sensor_read + ini_val;
    ini_val = ini_val1;
//    Serial.println(sensor_read);
    delay(10);
  }
  ini_val = ini_val1 / 100;

  return ini_val;
}

#endif
