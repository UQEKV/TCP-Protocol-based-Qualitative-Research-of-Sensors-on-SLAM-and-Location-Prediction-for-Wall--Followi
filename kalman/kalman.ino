#include "error.h"

unsigned long now, lastTime = 0;
float dt;                                  

int16_t ax, ay, az, gx, gy, gz;            
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; 
long axo = 0, ayo = 0, azo = 0;             
long gxo = 0, gyo = 0, gzo = 0;             
float pi = 3.1415926;
float AcceRatio = 0.061 * 9.8118 / 1000;             
float GyroRatio = 8.75 / 1000;                  

uint8_t n_sample = 8;                       
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};       
long aax_sum, aay_sum, aaz_sum;                     

float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0} , g_x[10] = {0} , g_y[10] = {0}, g_z[10] = {0}; 
float Px = 1, Rx, Kx, Sx, Vx, Qx;          
float Py = 1, Ry, Ky, Sy, Vy, Qy;          
float Pz = 1, Rz, Kz, Sz, Vz, Qz;           

error error_x(0);
error error_y(1);
error error_z(2);
error error_xx(3);
error error_yy(4);
error error_zz(5);

float error_ax;
float error_ay;
float error_az;
float error_gx;
float error_gy;
float error_gz;

float last_ax;
float last_ay;
float last_az;
float last_gx;
float last_gy;
float last_gz;


void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  imu.read();

  error_ax = error_x.error_cal();
  error_ay = error_y.error_cal();
  error_az = error_z.error_cal();
  error_gx = error_xx.error_cal();
  error_gy = error_yy.error_cal();
  error_gz = error_zz.error_cal();
}

void loop() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;          
  lastTime = now;                           

  imu.read();


  float ax0 = 1.00392051864409 * imu.a.x - 0.00739319226137609 * imu.a.y + 0.0172441491760619 * imu.a.z + 0.166395998962675;
  float ay0 = 0.0114511655914696 * imu.a.x + 0.997243972133797 * imu.a.y - 0.00440474354210289 * imu.a.z - 0.0233185497723911;
  float az0 = -0.0228738868570610 * imu.a.x - 0.0240227251433753 * imu.a.y + 0.996251519494944 * imu.a.z + 0.308890402212792;

  ax = lowpass_filter( ax0 , last_ax, 0.1 );
  ay = lowpass_filter( ay0 , last_ay, 0.1 );
  az = lowpass_filter( az0 , last_az, 0.1 );
  gx = lowpass_filter( imu.g.x - error_gx, last_gx, 0.1 );
  gy = lowpass_filter( imu.g.y - error_gy, last_gy, 0.1 );
  gz = lowpass_filter( imu.g.z - error_gz, last_gz, 0.1 );

  last_ax = ax;
  last_ay = ay;
  last_az = az;
  last_gx = gx;
  last_gy = gy;
  last_gz = gz;

  float accx = (ax0) * AcceRatio;            
  float accy = (ay0) * AcceRatio;             
  float accz = (az0) * AcceRatio;             

  aax = atan(accy / accz) * (-180) / pi;    
  aay = atan(accx / accz) * 180 / pi;       
  aaz = atan(accz / accy) * 180 / pi;     

  aax_sum = 0;                              
  aay_sum = 0;
  aaz_sum = 0;

  for (int i = 1; i < n_sample; i++)
  {
    aaxs[i - 1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    aays[i - 1] = aays[i];
    aay_sum += aays[i] * i;
    aazs[i - 1] = aazs[i];
    aaz_sum += aazs[i] * i;

  }

  aaxs[n_sample - 1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.33; 
  aays[n_sample - 1] = aay;                     
  aay_sum += aay * n_sample;                     
  aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.33;
  aazs[n_sample - 1] = aaz;
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.33;

  float gyrox = - (gx) * GyroRatio * dt; 
  float gyroy = - (gy) * GyroRatio * dt; 
  float gyroz = - (gz) * GyroRatio * dt; 

  agx += gyrox;                             
  agy += gyroy;
  agz += gyroz;

  /* kalman start */
  Sx = 0; Rx = 0;
  Sy = 0; Ry = 0;
  Sz = 0; Rz = 0;

  for (int i = 1; i < 10; i++)
  { //测量值平均值运算
    a_x[i - 1] = a_x[i];                    
    Sx += a_x[i];
    a_y[i - 1] = a_y[i];
    Sy += a_y[i];
    a_z[i - 1] = a_z[i];
    Sz += a_z[i];
  }

  a_x[9] = aax;
  Sx += aax;
  Sx /= 10;                                 
  a_y[9] = aay;
  Sy += aay;
  Sy /= 10;                                 
  a_z[9] = aaz;
  Sz += aaz;
  Sz /= 10;

  for (int i = 0; i < 10; i++)
  {
    Rx += sq(a_x[i] - Sx);
    Ry += sq(a_y[i] - Sy);
    Rz += sq(a_z[i] - Sz);

  }

  Rx = Rx / 9;                              
  Ry = Ry / 9;
  Rz = Rz / 9;

  Px = Px + 0.0025;                         
  Kx = Px / (Px + Rx);                      
  agx = agx + Kx * (aax - agx);             
  Px = (1 - Kx) * Px;                       

  Py = Py + 0.0025;
  Ky = Py / (Py + Ry);
  agy = agy + Ky * (aay - agy);
  Py = (1 - Ky) * Py;

  Pz = Pz + 0.0025;
  Kz = Pz / (Pz + Rz);
  agz = agz + Kz * (aaz - agz);
  Pz = (1 - Kz) * Pz;

  /* kalman end */
  //
  Serial.print(agx); Serial.print(",");
  Serial.print(agy); Serial.print(",");
  Serial.print(agz); Serial.println();

}

float lowpass_filter(float reading, float last_output, float alpha) {

  last_output = ( alpha * reading ) + ( (1 - alpha) * last_output );

  return last_output;
}
