#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_AutoTune_v0.h"


PID_ATune::PID_ATune(double* Input, double* Output)
{
  input = Input;
  output = Output;
  controlType = CLASSIC_PI;
  noiseBand = 0.5;
  running = false;
  oStep = 30;
  SetLookbackSec(10);
  lastTime = millis();
  Dither = 0.0;
}



void PID_ATune::Cancel()
{
  running = false;
} 

int PID_ATune::Runtime()
{
  justevaled=false;
  if(peakCount>9 && running)
  {
    running = false;
    FinishUp();
    return 1;
  }
  unsigned long now = millis();

  if((now-lastTime)<sampleTime) 
  {
    return false;
  }
  lastTime = now;
  double refVal = *input;

  // dither input value to smooth quantization error
  if ( Dither > 0.0 )
  {
    // add random noise from triangular probability density function 
    // centred on 0 and with range (-Dither, Dither)
    refVal += ( random( -Dither, Dither ) + random( -Dither, Dither ) ) / 2.0;
  }

  justevaled=true;
  if(!running)
  { //initialize working variables the first time around
    peakType = NON_PEAK;
    peakCount = 0;
    justchanged=false;
    absMax=refVal;
    absMin=refVal;
    setpoint = refVal;
    running = true;
    outputStart = *output;
    *output = outputStart+oStep;
  }
  else
  {
    if(refVal>absMax)absMax=refVal;
    if(refVal<absMin)absMin=refVal;
  }

  //oscillate the output base on the input's relation to the setpoint

  if(refVal>setpoint+noiseBand) *output = outputStart-oStep;
  else if (refVal<setpoint-noiseBand) *output = outputStart+oStep;


  //bool isMax=true, isMin=true;
  isMax=true;
  isMin=true;
  //id peaks
  for(int i=nLookBack-1;i>=0;i--)
  {
    double val = lastInputs[i];
    if(isMax) isMax = refVal>val;
    if(isMin) isMin = refVal<val;
    lastInputs[i+1] = lastInputs[i];
  }
  lastInputs[0] = refVal;  

  if(isMax)
  {
    if(peakType==NON_PEAK)
    {
      peakType=MAXIMUM;
    }
    if(peakType==MINIMUM)
    {
      peakType = MAXIMUM;
      justchanged=true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;

  }
  else if(isMin)
  {
    if(peakType==NON_PEAK)
    {
      peakType=MINIMUM;
    }
    if(peakType==MAXIMUM)
    {
      peakType=MINIMUM;
      peakCount++;
      justchanged=true;
    }

    if(peakCount<10)
    {
      peaks[peakCount] = refVal;
    }
  }

  if(justchanged && peakCount>2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    double avgSeparation = (abs(peaks[peakCount-1]-peaks[peakCount-2])+abs(peaks[peakCount-2]-peaks[peakCount-3]))/2;
    if( avgSeparation < 0.05*(absMax-absMin))
    {
      FinishUp();
      running = false;
      return 1;
    }
  }
  justchanged=false;
  return 0;
}
void PID_ATune::FinishUp()
{
  *output = outputStart;

  // generate tuning parameters
  // using Second Ziegler-Nichols method (closed loop)
  // NB peak-to-peak amplitude of relay signal is 2 * p->Relay_amp
  Ku = 4.0 * ( 2.0 * oStep ) / ( ( absMax - absMin ) * 3.14159265358979 ); // ultimate gain
  Pu = (double) ( peak1 - peak2 ) / 1000.0; // ultimate period in seconds

  // calculate gain parameters
  if ( controlType == CLASSIC_PID )
  {
    Kp = 0.6   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.125 * Pu;
  }
  else if ( controlType == PESSEN )
  {
    Kp = 0.7   * Ku;
    Ti = 0.4   * Pu;
    Td = 0.15  * Pu;
  }
  else if ( controlType == SOME_OVERSHOOT )
  {
    Kp = 0.33  * Ku;
    Ti = 0.5   * Pu;
    Td = 0.33  * Pu;
  }
  else if ( controlType == NO_OVERSHOOT )
  {
    Kp = 0.2   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.33  * Pu;
  }
  else // controlType == CLASSIC_PI
  {
    Kp = 0.4   * Ku;
    Ti = 0.45  * Pu;
    Td = 0.0;
  }
}

double PID_ATune::GetKp()
{
  return Kp;
}

double PID_ATune::GetKi()
{
  return Kp / Ti;
}

double PID_ATune::GetKd()
{
  return Kp * Td; 
}

void PID_ATune::SetOutputStep(double Step)
{
  oStep = Step;
}

double PID_ATune::GetOutputStep()
{
  return oStep;
}

void PID_ATune::SetControlType(enum Control Type) 
{
  controlType = Type;
}
int PID_ATune::GetControlType()
{
  return controlType;
}

void PID_ATune::SetNoiseBand(double Band)
{
  noiseBand = Band;
}

double PID_ATune::GetNoiseBand()
{
  return noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
  if (value<1) value = 1;

  if(value<25)
  {
    nLookBack = value * 4;
    sampleTime = 250;
  }
  else
  {
    nLookBack = 100;
    sampleTime = value*10;
  }
}

int PID_ATune::GetLookbackSec()
{
  return nLookBack * sampleTime / 1000;
}

void PID_ATune::SetDither( double NewDither )
{
  if ( NewDither >= 0.0 )
  {
    Dither = NewDither;
  }
}	