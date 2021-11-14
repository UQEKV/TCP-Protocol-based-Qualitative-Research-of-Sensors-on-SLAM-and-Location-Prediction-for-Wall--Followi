#ifndef _UTILS_H
#define _UTILS_H



float lowpass_filter(float reading, float last_output, float alpha){

     last_output = ( alpha * reading ) + ( (1 - alpha) * last_output );

      return last_output;
}




#endif
