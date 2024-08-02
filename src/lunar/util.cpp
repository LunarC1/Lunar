#include "lunar/api.hpp"
#include "math.h"
#include <cmath>

float reduce_0_to_360(float angle) {
  while(!(angle >= 0 && angle < 360)) {
    if( angle < 0 ) { angle += 360; }
    if(angle >= 360) { angle -= 360; }
  }
  return(angle);
}


float reduce_negative_180_to_180(float angle) {
  while(!(angle >= -180 && angle < 180)) {
    if( angle < -180 ) { angle += 360; }
    if(angle >= 180) { angle -= 360; }
  }
  return(angle);
}

float reduce_negative_90_to_90(float angle) {
  while(!(angle >= -90 && angle < 90)) {
    if( angle < -90 ) { angle += 180; }
    if(angle >= 90) { angle -= 180; }
  }
  return(angle);
}

float to_rad(float angle_deg){
  return(angle_deg/(180.0/3.14));
}


float to_deg(float angle_rad){
  return(angle_rad*(180.0/3.14));
}

float clamp(float input, float min, float max){
  if( input > max ){ return(max); }
  if(input < min){ return(min); }
  return(input);
}

float deadband(float input, float width){
  if (std::abs(input)<width){
    return(0);
  }
  return(input);
}

float limit(float input, float min, float max){
  if (input > max) input = max;
  else if (input < -max) input = -max;
  if (input < 0 && input > -min) input = -min;
  else if (input > 0 && input < min) input = min;

  return (input);
}

float avg(std::vector<float> values) {
  float sum = 0;
  for (float value : values) { sum += value; }
  return sum / values.size();
}