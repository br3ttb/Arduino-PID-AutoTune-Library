#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION 0.0.1

#include "PID_types.h"

// by default we do not want to implement dither
#undef DITHER

// amplitude of successive peaks must differ by no more than this proportion
#define PEAK_AMPLITUDE_TOLERANCE 0.05

// ratio of up/down relay step duration should differ by no more than this tolerance
#define STEP_ASYMMETRY_TOLERANCE 0.15

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
  unsigned long peakTime[3];
  double peaks[20];
  byte peakCount;
  unsigned long stepTime[5];
  double sumInputSinceLastStep[5];
  byte stepCount;
  double lastInputs[101];
  byte inputCount;
  double outputStart;
  double originalNoiseBand;
  double relayBias;
  double K_process, Kp, Ti, Td;

#ifdef DITHER
  double dither;
#endif
};

#endif
