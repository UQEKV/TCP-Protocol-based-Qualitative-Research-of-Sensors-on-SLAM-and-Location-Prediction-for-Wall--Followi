/*******Motor define********/
#include "motor.h"
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15
motor_c motor_left(L_PWM_PIN,L_DIR_PIN);
motor_c motor_right(R_PWM_PIN,R_DIR_PIN);
float l_power;
float r_power;


/*******Buzzer define********/
#define Buzzer_Pin 6


/*******proximities define********/
#include <Wire.h>
#include <LSM6.h>
#include "utils.h"
#include "proximities.h"
#define FRONT_VIN_PIN 5
float d_f;
float d_s;
float powerMax = 18;
float thres_f = 182.5;
//float thres_f = 182.5;
float thres_s = 182.5;
Proxi_c ProxiSensor(FRONT_VIN_PIN, thres_f, thres_s);


/*******16*2 LCD define********/
#include <LiquidCrystal.h>
const int rs = 4, en = 11, d4 = 14, d5 = 12, d6 = 13, d7 = 30;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


/*******WIFI module define********/
#define WIFISerial Serial1
#define DEBUGSerial Serial1
const unsigned int WIFIRxBufferLength = 100;
char WIFIRxBuffer[WIFIRxBufferLength];
unsigned int WIFIBufferCount = 0;


/*******PID define********/
#include "pid.h"
PID_c PID_l(10, 0.1, 2);
PID_c PID_r(10, 0.1, 2);
PID_c PID_h(0.8, 0, 1.2);// pid for heading
float output_r,output_l; //PID outputs
float l_pwr,r_pwr;


/*******state machine define********/
#define STATE_INITIAL         0
#define STATE_WAIT            1
#define STATE_START           2
#define STATE_COMPLETE        3
#define STATE_GO_HOME         4
int state;


/*******IMU declear********/
LSM6 imu;
#include "calibration.h"
error error_z(0);
error error_x(1);
error error_y(2);

//previous sensor reading
float last_gz;
float last_ax;
float last_ay;
//sensor error for calibration
float error_gz;
float error_ax;
float error_ay;
//calibrated sensor readings
float gz;
float ax;
float ay;
float angle;


/*******Encoder define********/
#include "encoder.h"
volatile long p_r;
volatile long p_l;

/*******timestape declear********/
unsigned long last_timestamp_s;//standard
unsigned long last_timestamp_m;//motor
unsigned long last_timestamp_t;//turning
unsigned long last_timestamp_d;//turning
unsigned long spd_update_ts;//encoder speed


/*******kinematics********/
#include "kinematics.h"
kinematics_c km;


/*******Romi initialisation********/
void setup()
{

  setupEncoder1();
  setupEncoder0();
  
  //MOTOR CONTROL
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  //set direction of motors
  digitalWrite(L_DIR_PIN, 0);
  digitalWrite(R_DIR_PIN, 0);

  //Buzzer pinmode
  pinMode(Buzzer_Pin, OUTPUT);

  //Proximeter setup
  Wire.begin();
  ProxiSensor.setupSensor();
  
  lcd.begin(16, 2);
//  lcd.print("Ready!");

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

  Serial.begin(9600);
  WIFISerial.begin(9600);
  DEBUGSerial.begin(9600);
  DEBUGSerial.println("ready....");
  CountDown(1, 5);
  delay(500);

  last_timestamp_s = last_timestamp_m = last_timestamp_t = millis();
  spd_update_ts = millis();
}

void loop(){
//  unsigned long time_now = millis();
  unsigned long spd_update_dt = millis() - spd_update_ts;
  unsigned long elapsed_time_s = millis() - last_timestamp_s;
  unsigned long elapsed_time_d = millis() - last_timestamp_d;

  while (WIFISerial.available()){
    char Wbuffer = WIFISerial.read();
    WIFIRxBuffer[WIFIBufferCount++] = Wbuffer;
    if (WIFIBufferCount == WIFIRxBufferLength)clrRxBuffer();
    DEBUGSerial.write(Wbuffer);
//    Serial.println(Wbuffer);
  }
  
  /*******gyoscope compuatation********/
  if (elapsed_time_s > 50 ) {
    last_timestamp_s = millis();
    imu.read();
    
    gz = lowpass_filter( imu.g.z - error_gz, last_gz, 0.1 );
    ax = lowpass_filter( imu.a.x - error_ax, last_ax, 0.1 );
    ay = lowpass_filter( imu.a.y - error_ay, last_ay, 0.1 );
    last_gz = gz;
    last_ax = ax;
    last_ay = ay;
    angle += gz / 1000 * 8.75 * 50 / 1000;

//    Serial.println(gz / 1000 * 8.75);
//    Serial.print(",");
//    Serial.print(ax / 1000 * 0.061 * 9.8);
//    Serial.print(",");
//    Serial.println(ay / 1000 * 0.061 * 9.8);

//    Serial.print(ax / 1000 * 0.061 * 9.81);
//    Serial.print(",");
//    Serial.println(ay / 1000 * 0.061 * 9.81);
  }

  /*******gyoscope compuatation********/
  if (elapsed_time_s > 40 ) {
    ProxiSensor.updateDistance();
    d_f = ProxiSensor.d_f;
    d_s = ProxiSensor.update_d_s;
    ProxiSensor.mUpdate();
    ProxiSensor.wallBangbang(powerMax);
  }

  /*******encoder speed compuatation********/
  if (spd_update_dt>20) {
    spd_update_ts = millis();
    encount_r= count_Encoder_r-old_count_r;
    encount_l= count_Encoder_l-old_count_l;
    old_count_r = count_Encoder_r;
    old_count_l = count_Encoder_l;
    speed_r=(float)encount_r/(float)spd_update_dt;
    speed_l=(float)encount_l/(float)spd_update_dt;
    km.update(count_Encoder_r,count_Encoder_l);
//    km2.update(count_Encoder_r,count_Encoder_l);
//    if (abs(km2.theta-PI/2) >= PI/2 ){
//      p_r=0;
//      p_l=0;
//    }
//    p_r+=encount_r;
//    p_l+=encount_l;

    lcd.setCursor(0, 0);
//    lcd.print("X:");
//    lcd.print(km.x);
//    lcd.print(" ");
//    lcd.print("Y:");
//    lcd.print(km.y);
    lcd.print("ds:");
    lcd.print(d_s); 
    
    lcd.setCursor(0, 1);
//    lcd.print("H:");
//    lcd.print(km.theta*180/PI); 
//    lcd.print(" ");
//    lcd.print("A:");
//    lcd.print(angle+90); 
    lcd.print("df:");
    lcd.print(d_f); 
    
  }

  /*******state machine transform********/  
  if( state == STATE_INITIAL ) {
    if(strstr(WIFIRxBuffer, "000")!= NULL) { //calibration
      error_gz = error_z.error_cal();
      error_ax = error_x.error_cal();
      error_ay = error_y.error_cal();
      CountDown(3, 2);
//      MoveDelayForLowpass();
      DEBUGSerial.println("Calibration completed");
      clrRxBuffer();
//      PID_r.reset();
//      PID_l.reset();
//      PID_h.reset();
      state = STATE_WAIT;
    } else {
      state =  STATE_INITIAL;
    }

  } 
  else if( state == STATE_WAIT ) {
    if(strstr(WIFIRxBuffer, "111")!= NULL) { //calibration
      CountDown(1, 3);
      CountDown(1, 5);
      motor_left.motorDriver(0.8);
      motor_right.motorDriver(0.8);
      DEBUGSerial.println("Romi starts");
      clrRxBuffer();
      state = STATE_START;
    } else { 
      PID_r.reset();
      PID_l.reset();
      PID_h.reset();
      state =  STATE_WAIT;
    }     

  } 
  else if( state == STATE_START ) {
    if(strstr(WIFIRxBuffer, "999")!= NULL) { //calibration
      motor_left.motorDriver( 0 );
      motor_right.motorDriver( 0 );
      CountDown(1, 5);
      CountDown(1, 3);
      state = STATE_WAIT;
      last_timestamp_s = millis();
      last_timestamp_m = millis();
      last_timestamp_t=millis();
      DEBUGSerial.println("Romi pauses");
      clrRxBuffer();
    } else if (strstr(WIFIRxBuffer, "888")!= NULL) { //turning test
      state = STATE_COMPLETE;
      CountDown(1, 5);
      CountDown(1, 3);
      clrRxBuffer();
      PID_r.reset();
      PID_l.reset();
      PID_h.reset();
      motor_left.motorDriver( 0 );
      motor_right.motorDriver( 0 );
    } else {
      Map();
    } 
    
  } 
  else if( state == STATE_COMPLETE) {
    if(strstr(WIFIRxBuffer, "999")!= NULL) { //calibration
      motor_left.motorDriver( 0 );
      motor_right.motorDriver( 0 );
      CountDown(1, 5);
      CountDown(1, 3);
      state = STATE_WAIT;
      last_timestamp_s = millis();
      last_timestamp_m = millis();
      last_timestamp_t=millis();
      DEBUGSerial.println("Romi pauses");
      clrRxBuffer();
    } else {
      if (elapsed_time_d > 50 ) {
        last_timestamp_d= millis();
        turning(PI);
      }
    } 
  } 
  else if( state == STATE_GO_HOME) {
    
  } 
  else{
    Serial.println("System Error, Unknown state!");
  }
}

void Map(){
  unsigned long elapsed_time_m = millis() - last_timestamp_m;
  unsigned long elapsed_time_t = millis() - last_timestamp_t;
  /*******pid compuatation for constant acceleraction********/
  if (ProxiSensor.state == 0){
    if (elapsed_time_m > 10 ) {
      last_timestamp_m=millis();
      float steering = PID_h.update(0,ProxiSensor.m);
      l_pwr = PID_l.update( 0.7+steering*2.8, speed_l);   // speed = 2, factor = 3
      r_pwr = PID_r.update( 0.7-steering*2.8, speed_r);
    }
  }else if (ProxiSensor.state == 1){
    if (elapsed_time_t > 50 ) { //turing pid
      last_timestamp_t=millis();
      l_pwr = PID_l.update( -0.7, speed_l);   // speed = 2, factor = 3
      r_pwr = PID_r.update( 0.7, speed_r);
    }
  }
  motor_left.motorDriver( l_pwr );
  motor_right.motorDriver( r_pwr );
      DEBUGSerial.print(km.x);
      DEBUGSerial.print(" ");
      DEBUGSerial.print(km.y);
//      DEBUGSerial.print(" ");
  //    DEBUGSerial.print(ax/1000 * 0.061 * 9.81);
  //    DEBUGSerial.print(" ");
  //    DEBUGSerial.print(ProxiSensor.state);
  //    DEBUGSerial.print(" ");
//      DEBUGSerial.println(km.theta);
  //    DEBUGSerial.print(" ");
//      DEBUGSerial.println(d_s);
  //    DEBUGSerial.print(" ");
  //    DEBUGSerial.print(d_f);
  //    DEBUGSerial.print(" ");
  //    DEBUGSerial.print(ProxiSensor.m);
      DEBUGSerial.println(";");

}

void clrRxBuffer(void){
  memset(WIFIRxBuffer,0,WIFIRxBufferLength);
  WIFIBufferCount=0;
}

float error_calcul(float sensor_read) {
  float ini_val1;
  float ini_val;
  for (int i = 0; i < 100; i++) {
    imu.read();
    ini_val1 = sensor_read + ini_val;
    ini_val = ini_val1;
    delay(10);
  }
  ini_val = ini_val1 / 100;
  return ini_val;
}

void CountDown(int times, int volume) { //Buzzer
  for (int i = 0; i < times; i++)
  {
    analogWrite(Buzzer_Pin, volume);
    delay(100);
    analogWrite(Buzzer_Pin, 0);
    delay(100);
  }
}

void turning(float theta_demand){
  float diff = atan2(sin(theta_demand-km.theta),cos(theta_demand-km.theta));
  float bearing = PID_h.update(0,diff);
  float l_pwr = PID_l.update(bearing,speed_l);
  float r_pwr = PID_r.update(-bearing,speed_r);
  motor_left.motorDriver(l_pwr);
  motor_right.motorDriver(r_pwr); 
}

void MoveDelayForLowpass(){

      for (int i = 0; i < 100; i++){

          ProxiSensor.updateDistance();
          if (i==99) CountDown(2,4);
        
      }
  
}
