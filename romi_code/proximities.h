#ifndef _PROXIMITIES_H
#define _PROXIMITIES_H

#include <Wire.h>
#include <VL6180X.h>
#include "utils.h"
#include "kinematics.h"
//kinematics_c km2;
//#include "motor.h"

class Proxi_c{

      public:
          int f_prox_pin;
          VL6180X sensor_front;
          VL6180X sensor_side;
          
          float d_f;
          float d_s;
          float last_output_f;
          float last_output_s;
          float update_d_f;
          float update_d_s;
          float m;
          float l_power;
          float r_power;
          float threshold_f;
          float threshold_s;
          float theta;
          int state;
          float wheel_width;
          float romi_radius;
          float ds_offset;
          float df_offset;
          float d_to_wheel;
          
          
          Proxi_c(int f_prox_pin_in, float t_f, float t_s ){
              
             //for front sensor, starting by making sure front sensor is switchhed off
             //VIN of front sensor is attached to PIN 5 (digital pin)
              f_prox_pin = f_prox_pin_in;
              threshold_f = t_f;
              threshold_s = t_s;
              d_f = 255;
              d_s = 0;
              last_output_f = 0;
              last_output_s = 0;
              update_d_f = 0;
              update_d_s = 0;
              m = 0;
              l_power = 0;
              r_power = 0;
              state = 0;
              theta = 0;
              wheel_width = 147.0; //73.5mm
              romi_radius = 165.0; //82.5mm
              d_to_wheel = 33.5;
              ds_offset = romi_radius / 2.0 - ( d_to_wheel + (romi_radius - wheel_width) / 2.0  );
              df_offset = romi_radius / 2.0;
              
            }

          void updateDistance();
          void setupSensor();
          void mUpdate();
          void wallBangbang(float power_max);
  
  
  
  };

void Proxi_c::updateDistance(){
//    theta=km2.theta;
//    
//    if (theta<0) {  
//      theta +=PI/2;
//    }
//    if (theta>PI) {
//      theta -=PI/2;
//    }
    
    d_f = sensor_front.readRangeSingleMillimeters() + df_offset;
//    d_s = (sensor_side.readRangeSingleMillimeters() + ds_offset)*cos(theta - PI/2);
    d_s = sensor_side.readRangeSingleMillimeters() + ds_offset;
//    update_d_f = lowpass_filter(d_f, last_output_f, 0.1);
//    last_output_f = update_d_f;
     
    update_d_s = lowpass_filter(d_s, last_output_s, 0.1);
    last_output_s = update_d_s;
      
  }

void Proxi_c::mUpdate(){
    
    m = (threshold_s - update_d_s) / threshold_s;
  
}

//Motor_c l_motor(PWM_L, DIR_L);
//Motor_c r_motor(PWM_R, DIR_R);

void Proxi_c::wallBangbang(float power_max){

      if (d_f < threshold_f){

            state = 1; // collision avoidance
        
      }

      if (state == 1){

        l_power = -power_max*0.7; 
        r_power = power_max*0.7;
        
        if (d_f > 255+82.5-10){

            state = 0;
          
        }    
              
//        if (abs(km2.theta-PI/2) >= PI/2 ){
//  
//             km2.reset();
//             state = 0;    
//        }

      }

      if (state == 0){
//            l_power = power_max * (1 - m * 0.7);
//            r_power = power_max * (1 + m * 0.7);
            
//            l_motor.setPower(l_power);
//            r_motor.setPower(r_power);
        
      }
  
}
  


void Proxi_c::setupSensor(){

              pinMode(f_prox_pin, OUTPUT);                         
              digitalWrite(f_prox_pin, LOW); 
              
              Wire.begin();
              //setup right sensor(VIN attached to 5V), we will change the address of right sensor
              //initialising the right sensor class
              sensor_side.init(); 
              //the following configure the right sensor
              sensor_side.configureDefault();
              //which is achieved through I2C
              //sensor_right.setScaling(SCALING);
             
              //protocol. Therefore , it is necessary to call Wire.begin() first
              sensor_side.setTimeout(500);
          
              //change the address of right sensor to something other than default
              sensor_side.setAddress(0x54);
  
              //setup the left sensor (VIN attached to pin5) 
              digitalWrite(f_prox_pin, HIGH);
              delay(200);
              sensor_front.init();
              sensor_front.configureDefault();
              sensor_front.setTimeout(500);

  }




#endif
