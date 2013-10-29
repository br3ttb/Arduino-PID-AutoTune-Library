#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION 0.0.1

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_types.h"

// by default we do not want to implement dither
// this is probably not a very useful option
// except for highly granular, quantized process variables
// perhaps a better option would be to smooth
// using some kind of finite impulse response filter
// for noisy or unstable inputs
#undef DITHER

// average amplitude of successive peaks must differ by no more than this proportion
// relative to half the difference between maximum and minimum of last 2 cycles
#define PEAK_AMPLITUDE_TOLERANCE 0.05

// ratio of up/down relay step duration should differ by no more than this tolerance
// biasing the relay con give more accurate estimates of the tuning parameters but
// setting the tolerance too low will prolong the autotune procedure unnecessarily
#define STEP_ASYMMETRY_TOLERANCE 0.20

// terminate if too long between peaks or relay steps
#define MAX_WAIT_MINUTES 5

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
  unsigned long peakTime[4];            // * peak time, most recent in array element 0
  double peaks[4];                      // * peak value, most recent in array element 0
  byte peakCount;
  unsigned long stepTime[5];            // * step time, most recent in array element 0
  double sumInputSinceLastStep[5];      // * integrated process values, most recent in array element 0
  byte stepCount;
  double lastInputs[101];               // * process values, most recent in array element 0
  byte inputCount;
  double outputStart;
  double originalNoiseBand;
  double relayBias;
  double newNoiseBand;
  double K_process, Kp, Ti, Td;

#ifdef DITHER
  double dither;
#endif
};

#endif
