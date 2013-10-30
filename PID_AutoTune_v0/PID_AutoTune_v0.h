#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION 0.0.1

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_types.h"

// verbose debug option
// requires open Serial port
#undef AUTOTUNE_DEBUG

// defining this option implements relay bias
// this is useful to adjust the relay output values
// during the auto tuning to recover symmetric
// oscillations 
// this can compensate for load disturbance
// and equivalent signals arising from nonlinear
// or non-stationary processes 
// any improvement in the tunings seems quite modest 
// but sometimes unbalanced oscillations can be 
// persuaded to converge where they might not 
// otherwise have done so
#undef AUTOTUNE_RELAY_BIAS

// by default we do not want to implement dither
// this is probably not a very useful option
// except for highly granular, quantized process variables
// perhaps a more generallly useful option would be to 
// smooth using some kind of finite impulse response filter
// for noisy or unstable inputs
#undef AUTOTUNE_DITHER

// average amplitude of successive peaks must differ by no more than this proportion
// relative to half the difference between maximum and minimum of last 2 cycles
#define AUTOTUNE_PEAK_AMPLITUDE_TOLERANCE 0.05

// ratio of up/down relay step duration should differ by no more than this tolerance
// biasing the relay con give more accurate estimates of the tuning parameters but
// setting the tolerance too low will prolong the autotune procedure unnecessarily
// this parameter also sets the minimum bias in the relay as a proportion of its amplitude
#define AUTOTUNE_STEP_ASYMMETRY_TOLERANCE 0.20

// auto tune terminates if waiting too long between peaks or relay steps
// set larger value for processes with long delays or time constants
#define AUTOTUNE_MAX_WAIT_MINUTES 5

class PID_ATune
{

public:
  //commonly used functions ********************************************************************
  PID_ATune(double*, double*);          // * Constructor.  links the Autotune to a given PID
  bool Runtime();                       // * Similar to the PID Compute function, 
                                        //   returns true when done, otherwise returns false
  void Cancel();                        // * Stops the AutoTune 

  void SetOutputStep(double);           // * how far above and below the starting value will 
                                        //   the output step?   
  double GetOutputStep();               // 

  void SetControlType(enum Control);    // * Determines tuning algorithm
  int GetControlType();                 // * Returns tuning algorithm

  void SetLookbackSec(int);             // * how far back are we looking to identify peaks
  int GetLookbackSec();                 //

  void SetNoiseBand(double);            // * the autotune will ignore signal chatter smaller 
                                        //   than this value
  double GetNoiseBand();                //   this should be accurately set

#ifdef DITHER
  void SetDither( double );             // * noise added to input to smooth quantization errors
  double GetDither();                   //   set to smallest step value in input range
#endif

  double GetKp();                       // * once autotune is complete, these functions contain the
  double GetKi();                       //   computed tuning parameters.  
  double GetKd();                       //

private:
  bool CheckStable();                   // * check whether recent inputs have similar values
  double processValueOffset();          // * returns an estimate of the process value offset
                                        //   as a proportion of the amplitude

  double *input;
  double *output;
  double setpoint;

  double oStep;
  double noiseBand;
  byte nLookBack;
  enum Control controlType;             // * selects autotune algorithm

  enum AutoTunerState state;            // * state of autotuner finite state machine
  unsigned long lastTime;
  unsigned long sampleTime;
  enum Peak peakType;
  unsigned long lastPeakTime[5];        // * peak time, most recent in array element 0
  double lastPeaks[5];                  // * peak value, most recent in array element 0
  byte peakCount;
  double lastInputs[101];               // * process values, most recent in array element 0
  byte inputCount;
  double outputStart;
  double originalNoiseBand;
  double newNoiseBand;
  double K_process, Kp, Ti, Td;
  
#ifdef AUTOTUNE_RELAY_BIAS  
  double relayBias;
  double avgStep1;
  double avgStep2;
  unsigned long lastStepTime[5];        // * step time, most recent in array element 0
  double sumInputSinceLastStep[5];      // * integrated process values, most recent in array element 0
  byte stepCount;
#endif  

#ifdef DITHER
  double dither;
#endif
};

#endif
