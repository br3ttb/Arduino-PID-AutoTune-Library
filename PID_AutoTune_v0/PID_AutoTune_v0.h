#if !defined PID_AutoTune_v0
#define PID_AutoTune_v0
#define AUTOTUNE_LIBRARY_VERSION 0.0.2

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// verbose debug option
// requires open Serial port
#undef AUTOTUNE_DEBUG

#define USE_SIMULATION

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

// Ziegler-Nichols type auto tune rules
// in tabular form
struct Tuning
{
  byte _divisor[3];
  
  bool PI_controller()
  {
    return pgm_read_byte_near(&_divisor[2]) == 0;
  }
  
  double divisor(byte index)  
  {
    return (double)pgm_read_byte_near(&_divisor[index]) * 0.05;
  }
};

class PID_ATune
{

public:
  
  // constants ***********************************************************************************

  // auto tune method
  enum 
  {
    ZIEGLER_NICHOLS_PI = 0,	
    ZIEGLER_NICHOLS_PID = 1,
    TYREUS_LUYBEN_PI,
    TYREUS_LUYBEN_PID,
    CIANCONE_MARLIN_PI,
    CIANCONE_MARLIN_PID,
    AMIGOF_PI,
    PESSEN_INTEGRAL_PID,
    SOME_OVERSHOOT_PID,
    NO_OVERSHOOT_PID
  };

  // peak type
  enum Peak
  {
    MINIMUM = -1,
    NOT_A_PEAK = 0,
    MAXIMUM = 1
  };

  // auto tuner state
  enum AutoTunerState
  {
    AUTOTUNER_OFF = 0, 
    STEADY_STATE_AT_BASELINE = 1,
    STEADY_STATE_AFTER_STEP_UP = 2,
    RELAY_STEP_UP = 4,
    RELAY_STEP_DOWN = 8,
    CONVERGED = 16,
    FAILED = 128
  }; 

  // tuning rule divisor
  enum
  {
    KP_DIVISOR = 0,
    TI_DIVISOR = 1,
    TD_DIVISOR = 2
  };

  // irrational constants
  static const double CONST_PI          = 3.14159265358979323846;
  static const double CONST_SQRT2_DIV_2 = 0.70710678118654752440;

  // commonly used methods ***********************************************************************
  PID_ATune(double*, double*);          // * Constructor.  links the Autotune to a given PID
  bool Runtime();                       // * Similar to the PID Compute function, 
                                        //   returns true when done, otherwise returns false
  void Cancel();                        // * Stops the AutoTune 

  void SetOutputStep(double);           // * how far above and below the starting value will 
                                        //   the output step?   
  double GetOutputStep();               // 

  void SetControlType(byte);            // * Determines tuning algorithm
  byte GetControlType();                // * Returns tuning algorithm

  void SetLookbackSec(int);             // * how far back are we looking to identify peaks
  int GetLookbackSec();                 //

  void SetNoiseBand(double);            // * the autotune will ignore signal chatter smaller 
                                        //   than this value
  double GetNoiseBand();                //   this should be accurately set

  double GetKp();                       // * once autotune is complete, these functions contain the
  double GetKi();                       //   computed tuning parameters.  
  double GetKd();                       //


private:

  double processValueOffset(double,     // * returns an estimate of the process value offset
      double);                          //   as a proportion of the amplitude                                        

  double *input;
  double *output;
  double setpoint;

  double oStep;
  double noiseBand;
  byte nLookBack;
  byte controlType;                     // * selects autotune algorithm

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
  double workingNoiseBand;
  double workingOstep;
  double inducedAmplitude;
  double Kp, Ti, Td;

  // used by AMIGOf tuning rule
  double calculatePhaseLag(double);     // * calculate phase lag from noiseBand and inducedAmplitude
  double fastArcTan(double);
  double newWorkingNoiseBand;
  double K_process;
  
#if defined AUTOTUNE_RELAY_BIAS  
  double relayBias;
  unsigned long lastStepTime[5];        // * step time, most recent in array element 0
  double sumInputSinceLastStep[5];      // * integrated process values, most recent in array element 0
  byte stepCount;
#endif  

};

#endif
