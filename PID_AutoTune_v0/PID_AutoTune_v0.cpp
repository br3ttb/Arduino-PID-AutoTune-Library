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
  controlType = ZIEGLER_NICHOLS_PI;
  noiseBand = 0.5;
  running = false;
  oStep = 30;
  SetLookbackSec(10);
  lastTime = millis();
#ifdef DITHER
  dither = 0.0;
#endif
}

void PID_ATune::Cancel()
{
  running = false;
} 

bool PID_ATune::Runtime()
{
  justevaled = false;
  if ((peakCount > 9) && running)
  {
    running = false;
    FinishUp();
    return true;
  }
  unsigned long now = millis();

  if ((now - lastTime) < sampleTime) 
  {
    return false;
  }
  lastTime = now;
  double refVal = *input;

#ifdef DITHER
  // dither input value to smooth quantization error
  if (dither > 0.0)
  {
    // add random noise from triangular probability density function 
    // centred on 0 and with range (-dither, dither)
    refVal += ( random(-dither, dither) + random(-dither, dither) ) / 2.0;
  }
#endif

  justevaled = true;
  if (!running)
  { //initialize working variables the first time around
    peakType = NON_PEAK;
    peakCount = 0;
    justchanged = false;
    absMax = refVal;
    absMin = refVal;
    setpoint = refVal;
    running = true;
    outputStart = *output;
    *output = outputStart + oStep;
  }
  else
  {
    if (refVal > absMax)
    {
      absMax = refVal;
    }
    if (refVal < absMin)
    {
      absMin = refVal;
    }
  }

  // oscillate the output base on the input's relation to the setpoint

  if (refVal > setpoint + noiseBand)
  {
    *output = outputStart - oStep;
  }
  else if (refVal < setpoint - noiseBand)
  {
    *output = outputStart + oStep;
  }
  isMax = true;
  isMin = true;

  // identify peaks
  for (int i = nLookBack - 1; i >= 0; i--)
  {
    double val = lastInputs[i];
    if (isMax)
    {
      isMax = (refVal > val);
    }
    if (isMin) 
    {
      isMin = (refVal < val);
    }
    lastInputs[i+1] = lastInputs[i];
  }
  lastInputs[0] = refVal;  

  if (nLookBack < 9)
  {  
    // we don't want to trust the maxes or mins until the inputs array has been filled	
    return false;
  }
  
  if (isMax)
  {
    if (peakType == NON_PEAK)
    {
      peakType = MAXIMUM;
    }
    if (peakType == MINIMUM)
    {
      peakType = MAXIMUM;
      justchanged = true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;

  }
  else if (isMin)
  {
    if (peakType == NON_PEAK)
    {
      peakType = MINIMUM;
    }
    if (peakType == MAXIMUM)
    {
      peakType = MINIMUM;
      peakCount++;
      justchanged = true;
    }

    if (peakCount < 10)
    {
      peaks[peakCount] = refVal;
    }
  }

  if (justchanged && peakCount>2)
  { 
    //we've transitioned.  check if we can autotune based on the last peaks
    double avgSeparation = ( abs( peaks[peakCount - 1] - peaks[peakCount - 2] ) + 
                             abs( peaks[peakCount - 2] - peaks[peakCount - 3] ) ) / 2;
    if (avgSeparation < PEAK_AMPLITUDE_TOLERANCE * (absMax - absMin))
    {
      FinishUp();
      running = false;
      return true;
    }
  }
  justchanged = false;
  return false;
}

void PID_ATune::FinishUp()
{
  *output = outputStart;

  // generate tuning parameters
  // using Second Ziegler-Nichols method (closed loop)
  
#ifdef DITHER
  // calculate swing from highest to lowest input
  // net of dither range
  double induced_amplitude = (absMax - dither) - (absMin + dither);
#else
  double induced_amplitude = (absMax - absMin);
#endif
  
  // calculate relay amplitude as twice oStep
  double relay_amplitude = (2.0 * oStep);
  
  // NB peak-to-peak amplitude of relay signal is 2 * p->Relay_amp
  Ku = 4.0 *  relay_amplitude / (induced_amplitude * 3.14159265358979); // ultimate gain
  Pu = (double) (peak1 - peak2) / 1000.0; // ultimate period in seconds

  // calculate gain parameters
  switch(controlType)
  {
  case ZIEGLER_NICHOLS_PID:
    Kp = 0.6   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.125 * Pu;
    break;
  case PESSEN:
    Kp = 0.7   * Ku;
    Ti = 0.4   * Pu;
    Td = 0.15  * Pu;
    break;
  case SOME_OVERSHOOT:
    Kp = 0.33  * Ku;
    Ti = 0.5   * Pu;
    Td = 0.33  * Pu;
    break;
  case NO_OVERSHOOT:
    Kp = 0.2   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.33  * Pu;
    break;
  case ZIEGLER_NICHOLS_PI:
  default:
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

void PID_ATune::SetControlType(enum Control type) 
{
  controlType = type;
}

int PID_ATune::GetControlType()
{
  return controlType;
}

void PID_ATune::SetNoiseBand(double band)
{
  noiseBand = band;
}

double PID_ATune::GetNoiseBand()
{
  return noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
  if (value < 1) 
  {
    value = 1;
  }
  else if(value < 25)
  {
    nLookBack = value * 4;
    sampleTime = 250;
  }
  else
  {
    nLookBack = 100;
    sampleTime = value * 10;
  }
}

int PID_ATune::GetLookbackSec()
{
  return nLookBack * sampleTime / 1000;
}

void PID_ATune::SetDither(double newDither)
{
  if (newDither >= 0.0)
  {
    dither = newDither;
  }
}
