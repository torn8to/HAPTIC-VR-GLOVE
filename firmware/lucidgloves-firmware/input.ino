#include "esp_adc_cal.h"
// Requires RunningMedian library by Rob Tillaart
#if (ENABLE_MEDIAN_FILTER || ((INTERFILTER_MODE != INTERFILTER_NONE) && (FLEXION_MIXING != MIXING_NONE)))
  #include <RunningMedian.h>
#endif




int gauge_reading[5];
float servo_force[5];

#if ENABLE_MEDIAN_FILTER
  RunningMedian rmSamples[10] = {
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES),
      RunningMedian(MEDIAN_SAMPLES)
  };
#endif

#if ((INTERFILTER_MODE != INTERFILTER_NONE) && (FLEXION_MIXING == MIXING_SINCOS))
  RunningMedian sinSamples[5] = {
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES)
  };

    RunningMedian cosSamples[5] = {
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES),
      RunningMedian(INTERFILTER_SAMPLES)
  };
#endif

#define DEFAULT_VREF 1100
esp_adc_cal_characteristics_t *adc_chars;

byte selectPins[] = {PINS_MUX_SELECT};

int maxFingers[10] = {0,0,0,0,0,0,0,0,0,0};
int minFingers[10] = {ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX};

#if FLEXION_MIXING == MIXING_SINCOS
  #if INTERMEDIATE_CALIBRATION
  int sinMin[5] = {ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX};
  int sinMax[5] = {0,0,0,0,0};

  int cosMin[5] = {ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX, ANALOG_MAX};
  int cosMax[5] = {0,0,0,0,0};
  #else
  int sinMax[5] = {INTER_MAX, INTER_MAX, INTER_MAX, INTER_MAX, INTER_MAX};
  int sinMin[5] = {INTER_MIN, INTER_MIN, INTER_MIN, INTER_MIN, INTER_MIN};

  int cosMax[5] = {INTER_MAX, INTER_MAX, INTER_MAX, INTER_MAX, INTER_MAX};
  int cosMin[5] = {INTER_MIN, INTER_MIN, INTER_MIN, INTER_MIN, INTER_MIN};
  #endif

  bool atanPositive[8] = {true, true, true, true, true, true, true, true};


  int totalOffset1[5] = {0,0,0,0,0};
#endif



void setupInputs(){

  pinMode(PIN_MENU_BTN, INPUT_PULLUP);
  
  #if !TRIGGER_GESTURE
  pinMode(PIN_TRIG_BTN, INPUT_PULLUP);
  #endif

  #if !GRAB_GESTURE
  pinMode(PIN_GRAB_BTN, INPUT_PULLUP);
  #endif

  #if !PINCH_GESTURE
  pinMode(PIN_PNCH_BTN, INPUT_PULLUP);
  #endif

  #if USING_CALIB_PIN
  pinMode(PIN_CALIB, INPUT_PULLUP);
  #endif

  #if USING_STRAIN_GAGUE && USING_MULTIPLEXER
  int readStrainGauge()
  { 
  int current reading
  int reading = 0;
  for(int i = 0; i <24;i++)
    {
      
    }
  }
  
  
  #endif

  #if USING_MULTIPLEXER
  byte selectPins[] = {PINS_MUX_SELECT};
  //pinMode(MUX_INPUT, INPUT);
  for (int i = 0; i < sizeof(selectPins); i++){
    pinMode(selectPins[i], OUTPUT);
  }
  #endif
}


int analogPinRead(int pin){
  #if USING_MULTIPLEXER
  if (ISMUX(pin)){
    return readMux(UNMUX(pin));
  }
  else{
    return analogRead(pin);
  }
  #else
   return analogRead(UNMUX(pin));
  #endif
}

#if USING_MULTIPLEXER
int readMux(byte pin){
  /*byte selectPins[] = {PINS_MUX_SELECT}; //get the array of select pins for the mux

  for (int i = sizeof(selectPins - 1); i > -1; i--){
    digitalWrite(selectPins[i], ((int)pow(2,i) & (pin)) == 0 ? LOW:HIGH); //convert the pin number to binary, and set each digit to it's corresponsing select pin.
  }

  */
  switch(pin){
    case 0:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], LOW);
      break;
    case 1:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], LOW);
      break;
    case 2:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], LOW);
      break;
   case 3:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], LOW);
      break;
   case 4:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], LOW);
      break;
    case 5:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], LOW);
      break;
    case 6:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], LOW);
      break;
    case 7:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], LOW);
      break;
    case 8:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 9:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], HIGH);
      break;
   case 10:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 11:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 12:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 13:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], LOW);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 14:
      digitalWrite(selectPins[0], LOW);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], HIGH);
      break;
   case 15:
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], HIGH);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 16:
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], LOW);
      break;
    case 17:
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], LOW);
      break;
    case 18:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], LOW);
      break;
   case 19:
      readSecondMux();
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], LOW);
      break;
   case 20:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], LOW);
      break;
    case 21:
      readSecondMux();
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], LOW);
      break;
    case 22:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], LOW);
      break;
    case 23:
      readSecondMux();
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], LOW);
      break;
    case 24:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], HIGH);
      break;
    case 25:
      readSecondMux();
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], HIGH);
      break;
   case 26:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], LOW);
      digitalWrite(selectPins[7], HIGH);
      break;
    case 27:
      readSecondMux();
      digitalWrite(selectPins[0], HIGH);
      digitalWrite(selectPins[1], HIGH);
      digitalWrite(selectPins[2], LOW);
      digitalWrite(selectPins[3], HIGH);
      break;
    case 28:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], HIGH);
      break;
    case 29:
      readSecondMux();
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], LOW);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], HIGH);
      break;
    case 30:
      readSecondMux();
      digitalWrite(selectPins[4], LOW);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], HIGH);
      break;
   case 31:
      readSecondMux();      
      digitalWrite(selectPins[4], HIGH);
      digitalWrite(selectPins[5], HIGH);
      digitalWrite(selectPins[6], HIGH);
      digitalWrite(selectPins[7], HIGH);
      break;
    
    

  }
  delayMicroseconds(MULTIPLEXER_DELAY);
  return analogRead(MUX_INPUT);
}


void readSecondMux()
{
  digitalWrite(selectPins[0], HIGH);
  digitalWrite(selectPins[1], LOW);
  digitalWrite(selectPins[3], HIGH);
  digitalWrite(selectPins[2], HIGH);
}
#endif

void getFingerPositions(bool calibrating, bool reset){
  //int fingerPos[5] = {0,0,0,0}
  #if FLEXION_MIXING == MIXING_NONE //no mixing, just linear
  int rawFingersFlexion[5] = {NO_THUMB?0:analogPinRead(PIN_THUMB), analogPinRead(PIN_INDEX), analogPinRead(PIN_MIDDLE), analogPinRead(PIN_RING), analogPinRead(PIN_PINKY)};
  
  #elif FLEXION_MIXING == MIXING_SINCOS
  int rawFingersFlexion[5] = {NO_THUMB?0:sinCosMix(PIN_THUMB, PIN_THUMB_SECOND, 0 ), 
                                  sinCosMix(PIN_INDEX, PIN_INDEX_SECOND, 1 ), 
                                  sinCosMix(PIN_MIDDLE,PIN_MIDDLE_SECOND,2 ), 
                                  sinCosMix(PIN_RING,  PIN_RING_SECOND,  3 ), 
                                  sinCosMix(PIN_PINKY, PIN_PINKY_SECOND, 4 )};

  #endif

  int rawFingers[10];

  #if USING_SPLAY
    int rawFingersSplay[5] = {NO_THUMB?0:analogPinRead(PIN_THUMB_SPLAY), 
                              analogPinRead(PIN_INDEX_SPLAY), 
                              analogPinRead(PIN_MIDDLE_SPLAY), 
                              analogPinRead(PIN_RING_SPLAY), 
                              analogPinRead(PIN_PINKY_SPLAY)};
  #else
    int rawFingersSplay[5] = {0,0,0,0,0};
  #endif
    //memcpy(rawFingers, rawFingersFlexion, 5); //memcpy doesn't seem to work here
    //memcpy(&rawFingers[5], rawFingersSplay, 5); 

  for (int i = 0; i < 5; i++){
    rawFingers[i] = rawFingersFlexion[i];
    rawFingers[i+5] = rawFingersSplay[i];
  }
  
  
  //flip pot values if needed
  #if FLIP_FLEXION
  for (int i = 0; i < 5; i++){
    rawFingers[i] = ANALOG_MAX - rawFingers[i];
  }
  #endif
  
  #if FLIP_SPLAY
  for (int i = 5; i < 10; i++){
    rawFingers[i] = ANALOG_MAX - rawFingers[i];
  }
  #endif
  
  #if ENABLE_MEDIAN_FILTER
  for (int i = 0; i < 10; i++){
    rmSamples[i].add( rawFingers[i] );
    rawFingers[i] = rmSamples[i].getMedian();
  }
  #endif

  //reset max and mins as needed
  if (reset){
    for (int i = 0; i <10; i++){
      #if FLEXION_MIXING == MIXING_SINCOS
      if (i < 5)
        totalOffset1[i] = 0;
      #endif
      maxFingers[i] = INT_MIN;
      minFingers[i] = INT_MAX;
    }
  }
  
  //if during the calibration sequence, make sure to update max and mins
  if (calibrating){
    for (int i = 0; i <10; i++){
      if (rawFingers[i] > maxFingers[i])
        #if CLAMP_SENSORS
          maxFingers[i] = ( rawFingers[i] <= CLAMP_MAX )? rawFingers[i] : CLAMP_MAX;

        #else
          maxFingers[i] = rawFingers[i];
        #endif
      if (rawFingers[i] < minFingers[i])
        #if CLAMP_SENSORS
          minFingers[i] = ( rawFingers[i] >= CLAMP_MIN )? rawFingers[i] : CLAMP_MIN;
        #else
          minFingers[i] = rawFingers[i];
        #endif
    }
  }
  
  for (int i = 0; i<10; i++){
    if (minFingers[i] != maxFingers[i]){
      fingerPos[i] = map( rawFingers[i], minFingers[i], maxFingers[i], 0, ANALOG_MAX );
      #if CLAMP_ANALOG_MAP
        if (fingerPos[i] < 0)
          fingerPos[i] = 0;
        if (fingerPos[i] > ANALOG_MAX)
          fingerPos[i] = ANALOG_MAX;
      #endif
    }
    else {
      fingerPos[i] = ANALOG_MAX / 2;
    }
    
  }
}


int analogReadDeadzone(int pin){
  int raw = analogPinRead(pin);
  if (abs(ANALOG_MAX/2 - raw) < JOYSTICK_DEADZONE * ANALOG_MAX / 100)
    return ANALOG_MAX/2;
  else
    return raw;
}

int getJoyX(){
  #if JOYSTICK_BLANK
  return ANALOG_MAX/2;
  #elif JOY_FLIP_X
  return ANALOG_MAX - joyDeadzone(d(JOY_X_DICTIONARY).toInt()));
  #else
  return joyDeadzone(d(JOY_X_DICTIONARY).toInt())
  #endif
}

int getJoyY(){
  #if JOYSTICK_BLANK
  return ANALOG_MAX/2;
  #elif JOY_FLIP_Y
  return ANALOG_MAX - joyDeadzone(d(JOY_Y_DICTIONARY).toInt());
  #else
  return joyDeadzone(d(JOY_Y_DICTIONARY).toInt());
  #endif
}
// TODO CHANGE FOR THE A AND B BUTTON AND UPDATES BASED ON THE LAST DICTIONASRY INPUT
bool getButton(byte pin){ 
  return digitalRead(pin) != HIGH;
}

#if FLEXION_MIXING == MIXING_SINCOS
//mixing
int sinCosMix(int sinPin, int cosPin, int i){

  int sinRaw = analogPinRead(sinPin);
  int cosRaw = analogPinRead(cosPin);


  #if INTERFILTER_MODE != INTERFILTER_NONE
    sinSamples[i].add(sinRaw);
    cosSamples[i].add(cosRaw);
    int sinCalib = sinSamples[i].getMedian();
    int cosCalib = cosSamples[i].getMedian();
    #if INTERFILTER_MODE == INTERFILTER_ALL
      sinRaw = sinCalib;
      cosRaw = cosCalib;
    #endif
  #else
    int sinCalib = sinRaw;
    int cosCalib = cosRaw;
  #endif 

  #if INTERMEDIATE_CALIBRATION
  //scaling
  sinMin[i] = min(sinCalib, sinMin[i]);
  sinMax[i] = max(sinCalib, sinMax[i]);

  cosMin[i] = min(cosCalib, cosMin[i]);
  cosMax[i] = max(cosCalib, cosMax[i]);
  #endif

  int sinScaled = map(sinRaw, sinMin[i], sinMax[i], -ANALOG_MAX, ANALOG_MAX);
  int cosScaled = map(cosRaw, cosMin[i], cosMax[i], -ANALOG_MAX, ANALOG_MAX);


  //trigonometry stuffs
  double angleRaw = atan2(sinScaled, cosScaled);

  //counting rotations
  if (((angleRaw > 0) != atanPositive[i]) && sinScaled > cosScaled){
    totalOffset1[i] += atanPositive[i]?1:-1;
  }
  atanPositive[i] = angleRaw > 0;
  double totalAngle = angleRaw + 2*PI * totalOffset1[i];
  

  return (int)(totalAngle * ANALOG_MAX);
  
}
#endif
 

#if USING_CURRENT_SENSOR
float* getCurrentSensorsRaw(){
  float current_readings[5];
  readMux(PIN_CURRENT_SENSOR_THUMB);
  servo_force[0] = servo_processing(current_processing(analogRead(MUX_INPUT)));
  readMux(PIN_CURRENT_SENSOR_INDEX);
  servo_force[1] = servo_processing(current_processing(analogRead(MUX_INPUT)));
  readMux(PIN_CURRENT_SENSOR_MIDDLE);
  servo_force[2] = servo_processing(current_processing(analogRead(MUX_INPUT)));
  readMux(PIN_CURRENT_SENSOR_RING);
  servo_force[3] = servo_processing(current_processing(analogRead(MUX_INPUT)));
  readMux(PIN_CURRENT_SENSOR_PINKY);
  servo_force[4] = servo_processing(current_processing(analogRead(MUX_INPUT)));
  return current_readings;
}

float current_processing(int adc_value){
    return(((float) adc_value / 4096.0) * 4.2) * 1000 *.8;
}

float servo_processing(float servo_amperage)
{
  float servo_stall_torque = 11.40;
  float servo_unload_current = .300;
  float servo_load_current = 3.000;
  return 11.40 * (servo_amperage - servo_unload_current)/(servo_load_current - servo_unload_current);
}
#endif

#if USING_STRAIN_GAUGE
uint8_t _offset = 0;
uint8_t _scale = 1;
uint8_t _gain = 128;
uint8_t _mode = 0x04;
uint8_t _lastRead = 0;


void strainReset()
{
  powerDown();
  powerUp();
  _offset   = 0;
  _scale    = 1;
  _gain     = 128;
  _lastRead = 0;
  _mode     = 0x04;
}
bool strainIsReady()
{
  return digitalRead(PIN_DOUT) == LOW;
}
void powerDown(){
  //  at least 60 us HIGH
  digitalWrite(MUX_INPUT, HIGH);
  delayMicroseconds(64);
}
void powerUp(){
  digitalWrite(MUX_INPUT, LOW);
}


void strainBegin()
{
  pinMode(PIN_DOUT, INPUT);
  powerDown();
  powerUp();
}


uint8_t strainShiftIn()
{
  //  local variables are faster.
  pinMode(MUX_INPUT,OUTPUT);
  uint8_t clk   = MUX_INPUT;
  uint8_t data  = PIN_DOUT;
  uint8_t value = 0;
  uint8_t mask  = 0x80;
  while (mask > 0)
  {
    digitalWrite(clk, HIGH);
    delayMicroseconds(1);   //  T2  >= 0.2 us
    if (digitalRead(data) == HIGH)
    {
      value |= mask;
    }
    digitalWrite(clk, LOW);
    delayMicroseconds(1);   //  keep duty cycle ~50%
    mask >>= 1;
  }
  pinMode(MUX_INPUT,INPUT);
  return value;
}

float read()
{
  //  this BLOCKING wait takes most time...
  while (digitalRead(PIN_DOUT) == HIGH) yield();

  union
  {
    long value = 0;
    uint8_t data[4];
  } v;

  //  blocking part ...
  noInterrupts();

  //  Pulse the clock pin 24 times to read the data.
  //  v.data[2] = shiftIn(PIN_DOUT, MUX_INPUT, MSBFIRST);
  //  v.data[1] = shiftIn(PIN_DOUT, MUX_INPUT, MSBFIRST);
  //  v.data[0] = shiftIn(PIN_DOUT, MUX_INPUT, MSBFIRST);
  v.data[2] = strainShiftIn();
  v.data[1] = strainShiftIn();
  v.data[0] = strainShiftIn();

  //  TABLE 3 page 4 datasheet
  //
  //  CLOCK      CHANNEL      GAIN      m
  //  ------------------------------------
  //   25           A         128       1    //  default
  //   26           B          32       2
  //   27           A          64       3
  //
  //  only default 128 verified,
  //  selection goes through the set_gain(gain)
  //
  uint8_t m = 1;

  while (m > 0)
  {
    delayMicroseconds(1); //needed for fast processors?
    digitalWrite(MUX_INPUT, HIGH);
    digitalWrite(MUX_INPUT, LOW);
    m--;
  }
}

 void gaugeIntitialize(){
      readMux(PIN_STRAIN_GAGUE_THUMB);
      strainBegin();
      readMux(PIN_STRAIN_GAGUE_INDEX);
      strainBegin();
      readMux(PIN_STRAIN_GAGUE_MIDDLE);
      strainBegin();
      readMux(PIN_STRAIN_GAGUE_RING);
      strainBegin();
      readMux(PIN_STRAIN_GAGUE_PINKY);
      strainBegin();
    }

  int gaugeGetUnits(){
      uint32_t raw_readings[5];
      readMux(PIN_STRAIN_GAGUE_THUMB);
      raw_readings[0] = read();
      readMux(PIN_STRAIN_GAGUE_INDEX);
      raw_readings[1] = read();
      readMux(PIN_STRAIN_GAGUE_MIDDLE);
      raw_readings[2] = read();
      readMux(PIN_STRAIN_GAGUE_RING);
      raw_readings[3] = read();
      readMux(PIN_STRAIN_GAGUE_PINKY);
      raw_readings[4] = read();
  }

  



#endif
