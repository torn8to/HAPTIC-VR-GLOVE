#if USING_FORCE_FEEDBACK
//#include "HX711.h"
#if defined(ESP32)
  #include "ESP32Servo.h"
#else
  #include "Servo.h"
#endif
float integral = 0.0;
Servo pinkyServo;
Servo ringServo;
Servo middleServo;
Servo indexServo;
Servo thumbServo;

void setupServoHaptics(){
  pinkyServo.attach(PIN_PINKY_MOTOR);
  ringServo.attach(PIN_RING_MOTOR);
  middleServo.attach(PIN_MIDDLE_MOTOR);
  indexServo.attach(PIN_INDEX_MOTOR);
  thumbServo.attach(PIN_THUMB_MOTOR);
}

//static scaling, maps to entire range of servo
void scaleLimits(int* hapticLimits, float* scaledLimits){
  for (int i = 0; i < 5; i++){
    #if FLIP_FORCE_FEEDBACK
    scaledLimits[i] = hapticLimits[i] / 1000.0f * 180.0f;
    #else
    scaledLimits[i] = 180.0f - hapticLimits[i] / 1000.0f * 180.0f;
    #endif
  }
  
}
//I have no idea what is going on here i thinkk i need t o calculate this 
//dynamic scaling, maps to the limits calibrated from your finger
void dynScaleLimits(int* hapticLimits, float* scaledLimits){
  //will be refactored to take min and max as an argument

  /* this implementation of dynamic scaling relies on the assumption 
   * that the servo reaches 2/3 of the potentiometer's range, 
   * and that 0 degrees is geared to the start of the potentiometer.
   * Different hardware types may need to handle dynamic scaling differently.
   */
  for (int i = 0; i < sizeof(hapticLimits); i++){
    scaledLimits[i] = hapticLimits[i] / 1000.0f * 180.0f;
  }
}

void PIDhaptics(int* currentServoLocation, int* goalServoLocation, float* currentServoForce, float* currentStrainForce, float* goalForceFingers,float* errorIntegral )
{
  //Make sure current servoForce and as well as strain data  is calculated to capture tensile strength of the material the methods that map the values to closer to real
  // also make sure the integral is reset about every half second of main loop or else finger damage is possible.
  float kp = .0753;
  float ki = .0047;
  float p = 0;  
  for(int i  = 0; i < sizeof(current_servo_output); i++){
    float p = goalForceFingers[i] - currentServoFingers[i] - currentStrainForce[i];
    integral[i] += p;
    float pi = kp * (p) + ki * integral
    if(abs(currentServoLocation[i] - goalServoLocation[i]) >10)
    {
      currentServoLocation[i] =+ (int) pi
    }
    else{
      
    }
  }
}

void writeServoHaptics(int* hapticLimits){
  
  float scaledLimits[5];
  scaleLimits(hapticLimits, scaledLimits);
  if(hapticLimits[0] >= 0) thumbServo.write(scaledLimits[0]);
  if(hapticLimits[1] >= 0) indexServo.write(scaledLimits[1]);
  if(hapticLimits[2] >= 0) middleServo.write(scaledLimits[2]);
  if(hapticLimits[3] >= 0) ringServo.write(scaledLimits[3]);
  if(hapticLimits[4] >= 0) pinkyServo.write(scaledLimits[4]);
}

#endif
