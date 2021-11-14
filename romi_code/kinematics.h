#ifndef _KINEMATICS_H
#define _KINEMATICS_H


class kinematics_c {
  public:
    kinematics_c();   // constructor 
    void update(volatile long encoder_r,volatile long encoder_l);    // update kinematics
    void reset();
    void moveToPosition();
    float headingChange();
    float positionChange();
    float distance;
    float angle;
    float x,y,theta;
    long last_el,last_er;
  private:

    //Control gains
    int epm=12;
    int mpw=120;
    int wheelradius=35;
    float d_l; //Proportional
    float d_r; //Integral
    float wheelSeperation;
    float theta_new,theta_diff,theta_old;
    float x_new,y_new;
    float dpe;
    
}; // End of class definition.


kinematics_c::kinematics_c() { 
  reset();
} // end of constructor.

void kinematics_c::reset() {
  x      = 0;
  y      = 0;
  x_new  = 0;
  y_new  = 0;
  theta_new  = PI/2; //Romi initial angle is 90 degrees
  wheelSeperation = 143;
  dpe = 2*wheelradius*PI/(epm*mpw);
}


float kinematics_c::headingChange() {
  theta_diff = (d_r-d_l)/wheelSeperation;
  theta_old = theta_new;
  theta_new = theta_new+theta_diff;
  if (theta_new<0) {  //maintain the range of theta within 0 to 2PI
    theta_new +=TWO_PI;
  }
  if (theta_new>TWO_PI) {
    theta_new -=TWO_PI;
  }
  return theta_new;
}
float kinematics_c::positionChange() {
  float d;
  d = (d_l+d_r)/2;
  y_new = y_new + d*sin(theta_new);
  x_new = x_new + d*cos(theta_new);
  return x_new, y_new;
}

// Routine to execute the update to
// kinematics 
void kinematics_c::update(volatile long encoder_r,volatile long encoder_l) {
  long delta_el = encoder_l - last_el;
  long delta_er = encoder_r - last_er;
  last_el = encoder_l;
  last_er = encoder_r;
  d_r = (float)delta_er*dpe;
  d_l = (float)delta_el*dpe;
  headingChange();
  positionChange();
  theta = theta_new;
  x=x_new;
  y=y_new; 
}

//funtion calculates angle and distance away from home
void kinematics_c::moveToPosition() {
  distance = sqrt(x*x+y*y);
  angle = PI+atan2(y,x);
}


#endif
