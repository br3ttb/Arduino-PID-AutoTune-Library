#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION	0.0.1

#include "PID_types.h"

// by default we do not want to implement dither
#undef DITHER

// amplitude of successive peaks must differ by no more than this proportion
#define PEAK_AMPLITUDE_TOLERANCE 0.05

class PID_ATune
{

public:
  //commonly used functions ********************************************************************
  PID_ATune(double*, double*);        	// * Constructor.  links the Autotune to a given PID
  bool Runtime();	 	   	// * Similar to the PID Compute function, 
					//   returns true when done, otherwise returns false
  void Cancel();		   	// * Stops the AutoTune	

  void SetOutputStep(double);	  	// * how far above and below the starting value will 
					//   the output step?	
  double GetOutputStep();	 	// 

  void SetControlType(enum Control);	// * Determines if the tuning parameters returned 
   					//   will be PI (0) or PID (1)
  int GetControlType();		   	//   Returns controller type (0=PI, 1=PID)

  void SetLookbackSec(int);		// * how far back are we looking to identify peaks
  int GetLookbackSec();			//

  void SetNoiseBand(double);		// * the autotune will ignore signal chatter smaller 
					//   than this value
  double GetNoiseBand();		//   this should be accurately set

#ifdef DITHER
  void SetDither( double );		// * noise added to input to smooth quantization errors
  double GetDither();			//   set to smallest step value in input range
#endif

  double GetKp();			// * once autotune is complete, these functions contain the
  double GetKi();			//   computed tuning parameters.  
  double GetKd();			//

private:
  void FinishUp();
  bool isMax, isMin;
  double *input, *output;
  double setpoint;
  double noiseBand;
  enum Control controlType;
  bool running;
  unsigned long peak1, peak2, lastTime;
  unsigned long sampleTime;
  int nLookBack;
  enum Peak peakType;
  double lastInputs[101];
  double peaks[10];
  int peakCount;
  bool justchanged;
  bool justevaled;
  double absMax, absMin;
  double oStep;
  double outputStart;
  double Ku, Pu, Kp, Ti, Td;

#ifdef DITHER
  double dither;
#endif
};

#endif
