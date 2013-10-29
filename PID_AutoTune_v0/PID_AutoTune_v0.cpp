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

  // constructor defaults
  controlType = ZIEGLER_NICHOLS_PI;
  noiseBand = 0.5;
  state = AUTOTUNER_OFF;
  oStep = 10;
  SetLookbackSec(10);
#ifdef DITHER
  dither = 0.0;
#endif
}

void PID_ATune::Cancel()
{
  state = AUTOTUNER_OFF;
}

// check that recent inputs are equal
// give or take expected noise
bool PID_ATune::CheckStable()
{
  double max = inputCount[0];
  double min = inputCount[0];
  for (byte i = inputCount; i > 0; i--)
  {
    if (max < inputCount[i])
    {
      max = inputCount[i];
    }
    if (min < inputCount[i])
    {
      min = inputCount[i];
    }
  } 
#ifdef DITHER
  return ((max - min) <= noiseBand + 2 * dither)
#else
  return ((max - min) <= noiseBand)
#endif
}

bool PID_ATune::Runtime()
{
  unsigned long now = millis();

  if (state == AUTOTUNER_OFF)
  { 
    // initialize working variables the first time around
    peakType = NOT_A_PEAK;
    inputCount = 0;
    peakCount = 0;
    stepCount = 0;
    setpoint = refVal;
    outputStart = *output;
    originalNoiseBand = noiseBand;
    relayBias = 0;
    peakTime[0] = now;
    stepTime[0] = now;
    sumInputSinceLastStep[0] = 0;
    
    // move to new state
    if (type == AMIGOF_PI)
    {
      state = STEADY_STATE_AT_BASELINE;
    }
    else
    {
      state = RELAY_STEP_UP;
    }
  }

  // otherwise check ready for new input
  else if ((now - lastTime) < sampleTime) 
  {
    return false;
  }

  // get new input
  lastTime = now;
  double refVal = *input;

#ifdef DITHER
  // dither input value to smooth quantization error
  if (dither > 0.0)
  {
    // add random noise from triangular probability density function 
    // centred on 0 and with range (-dither, dither)
    refVal += ( random(-dither, dither) + random(-dither, dither) ) * 0.5;
  }
#endif
 
  // used to calculate relay bias
  sumInputSinceLastStep[0] += refVal;

  // local flag variable
  bool justChanged = false; 

  // check input and change relay state if necessary
  if (state == RELAY_STEP_UP) && (refVal > setpoint + noiseBand))
  {
    state = RELAY_STEP_DOWN;
    justChanged = true;
  }
  else if ((state == RELAY_STEP_DOWN) && (refVal < setpoint - noiseBand))
  {
    state = RELAY_STEP_UP;
    justChanged = true;
  }
  if (justChanged)
  {
    for (byte i = (stepCount > 5 ? 5 : stepCount); i > 0; i--)
    {
      stepTime[i] = stepTime[i - 1];
      sumInputSinceLastStep[i] = sumInputSinceLastStep[i - 1];
    }
    stepCount++;
    stepTime[0] = now;
    sumInputSinceLastStep[0] = 0;
  }

  // set output
  // FIXME need to respect output limits
  if (state & (STEADY_STATE_AFTER_STEP_UP | RELAY_STEP_UP))
  {
    *output = outputStart + oStep + relayBias;
  }
  else if 
  if (state == RELAY_STEP_DOWN)
  {
    *output = outputStart - oStep + relayBias;
  }

  // store initial inputs
  // we don't want to trust the maxes or mins
  // until the input array is full
  inputCount++;
  if (inputCount <= nLookBack)
  {
    lastInputs[nLookBack - inputCount] = refVal;
    return false;
  }

  // identify peaks
  // and shift lastInputs[]
  inputCount = nLookBack;
  bool isMax = true;
  bool isMin = true;
  for (byte i = inputCount - 1; i >= 0; i--)
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
    lastInputs[i + 1] = lastInputs[i];
  }
  lastInputs[0] = refVal; 

  // for AMIGOf tuning rule, perform an initial
  // step change to calculate process gain K_process
  // this may be very slow for lag-dominated processes
  // and may never terminate for integrating processes 
  if (state & (STEADY_STATE_AT_BASELINE | STEADY_STATE_AFTER_STEP_UP))
  {
    // check that all the recent inputs are 
    // equal give or take expected noise
    if CheckStable()
    {
      stepTime[0] = now;
      double avgInput = 0.0;
      for (byte i = 0; i < inputCount; i++)
      {
        avgInput += lastInputs[i];
      }
      avgInput /= inputCount + 1;
      if (state == STEADY_STATE_AT_BASELINE)
      {
        state == STEADY_STATE_AFTER_STEP_UP;
        peaks[0] = avgInput;   
        return false;
      }
      // else state == STEADY_STATE_AFTER_STEP_UP
      // calculate process gain
      K_process = (avgInput - peaks[0]) / oStep;
      state == RELAY_STEP_DOWN;
      sumInputSinceLastStep[0] = 0;
      return false;
    }
  }
  
  // increment peak count 
  // and record peak time 
  // for both maxima and minima 
  justChanged = false;
  if (isMax)
  {
    if (peakType == NOT_A_PEAK)
    {
      peakType = MAXIMUM;
    }
    if (peakType == MINIMUM)
    {
      peakType = MAXIMUM;
      justChanged = true;
      peakCount++;
    }
  }
  else if (isMin)
  {
    if (peakType == NOT_A_PEAK)
    {
      peakType = MINIMUM;
    }
    if (peakType == MAXIMUM)
    {
      peakType = MINIMUM;
      justChanged = true;
      peakCount++;
    }
  }
  if (isMax || isMin)
  {
    peakTime[0] = now;
    if (peakCount < 20)
    {
      peaks[peakCount] = refVal;
    }
  }
  if (justChanged)
  {
    for (byte i = (peakCount > 2 ? 2 : peakCount); i > 0; i--)
    {
      peakTime[i] = PeakTime[i - 1];
    }
  }

  // check for convergence of induced oscillation
  // convergence of amplitude assessed on last 4 peaks (1.5 cycles)
  double inducedAmplitude = 0.0;
  double phaseLag;
  if (justChanged && (peakCount > 3) && (stepCount > 4))
  { 
    // convergence assessed based on last 4 peaks (1.5 cycles)
    byte i;
    double absMax = peaks[peakCount - 1];
    double absMin = peaks[peakCount - 1];
    for (i = peakCount - 1; i > peakCount - 3; i--)
    {
      inducedAmplitude += abs(peaks[i] - peaks[i - 1]); 
      if (absMax < peaks[i - 1])
      {
         absMax = peaks[i - 1];
      }
      if (absMin > peaks[i - 1])
      {
         absMin = peaks[i - 1];
      }
    }
    inducedAmplitude /= 6.0;

    // check convergence criterion for symmetry of oscillation
    unsigned long avgStep1 = 0.5 * ((stepTime[0] - stepTime[1]) + (stepTime[2] - stepTime[3]));
    unsigned long avgStep2 = 0.5 * ((stepTime[1] - stepTime[2]) + (stepTime[3] - stepTime[4]));
    double asymmetry = (avgStep1 > avgStep2) ?
                       (avgStep1 - avgStep2) / avgStep1 : (avgStep2 - avgStep1) / avgStep2;
    if (asymmetry > STEP_ASYMMETRY_TOLERANCE)
    {
      // relay steps are asymmetric
      // calculate relay bias using
      // "Autotuning of PID Controllers: A Relay Feedback Approach",
      //  by Cheng-Ching Yu, 2nd Edition, equation 7.39, p. 148

      // calculate offset of oscillation in process value
      // this is an approximation assuming a square wave oscillation 
      double Delta_a = 0.0;
      for (byte i = 0; i < 4; i++)
      {
        Delta_a += ((i & 1) ? 0.25 : -0.25) * 
                   sumInputSinceLastStep[i + 1] / ((stepTime[i] - stepTime[i + 1]) / sampleTime);
      }

      // calculate relay bias
      double relayBias = - (Delta_a / inducedAmplitude) * oStep;
      if (state == RELAY_STEP_UP)
      {
        relayBias = -relayBias;
      }
      return false;
    }

    // source for AMIGOf PI auto tuning method:
    // "Revisiting the Ziegler-Nichols tuning rules for PI control — 
    //  Part II. The frequency response method."
    // T. Hägglund and K. J. Åström
    // Asian Journal of Control, Vol. 6, No. 4, pp. 469-482, December 2004
    // http://www.ajc.org.tw/pages/paper/6.4PD/AC0604-P469-FR0371.pdf
    if (type == AMIGOF_PI)
    {
      // calculate phase lag
      // NB hysteresis = 2 * noiseBand;
      phaseLag = CONST_PI - asin(2.0 * noiseBand / inducedAmplitude); 
    }

    // check that phase lag is within acceptable bounds
    else if ((type == AMIGOF_PI) && (abs(phaseLag / CONST_PI * 180.0 - 130.0) > 10.0))
    {
      // phase lag outside the desired range between 120° and 140°
      // set noiseBand to halfway between old value and new estimate
      double newHysteresis = inducedAmplitude * sin(phaseLag);
      noiseBand = 0.5 * (newHysteresis * 0.5 + noiseBand);
      return false;
    }

    // check convergence criterion for amplitude of induced oscillation
    if ((0.5 * (absMax - absMin) - inducedAmplitude) / inducedAmplitude < PEAK_AMPLITUDE_TOLERANCE)
    {
      state = CONVERGED;
    }
  }
    
  // if the autotune has not already converged
  // terminate after 10 cycles 
  // or if too long between peaks
  // or if too long between relay steps
  if (
    (peakCount == 20) ||
    (peakTime[0] - now > MAX_WAIT_MINUTES * 60000) ||
    (stepTime[0] - now > MAX_WAIT_MINUTES * 60000) 
  )
  {
    state = FAILED;
  }
  
  if (((byte) state & (CONVERGED | FAILED)) == 0)
  {
    return false;
  }

  // autotune algorithm has terminated 
  // reset autotuner variables

  state == AUTOTUNER_OFF;
  *output = outputStart;
  noiseBand = originalNoiseBand;

  if (state == FAILED)
  {
    // do not calculate gain parameters
    return true;
  }

  // finish up by calculating tuning parameters
  
#ifdef DITHER
  // calculate amplitude of variation in input
  // net of dither range
  inducedAmplitude -= dither;
#endif
  
  // calculate ultimate gain
  double Ku = 4.0 * oStep / (inducedAmplitude * CONST_PI); 

  // calculate ultimate period in seconds
  double Pu = (double) (peakTime[0] - peakTime[2]) / 1000.0; 

  // calculate gain parameters
  switch(controlType)
  {
  case AMIGOF_PI:
    double kappa_phi = Ku / K_process;
    double a =  2.50 - 0.92 * phaseLag;
    double b = 10.75 - 4.01 * phaseLag;
    double c = -3.05 + 1.72 * phaseLag;
    double d = -6.10 + 3.44 * phaseLag;
    Kp = a / (1.0 + b * kappa_phi)     * Ku;
    Ti = c / (1.0 + d * kappa_phi)**2  * Pu;
    Td = 0.0;
    break;
  case ZIEGLER_NICHOLS_PID:
    Kp = 0.6   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.125 * Pu;
    break;
  // source for Pessen Integral, Some Overshoot,
  // and No Overshoot rules:
  // "Rule-Based Autotuning Based on Frequency Domain Identification" 
  // by Anthony S. McCormack and Keith R. Godfrey
  // IEEE Transactions on Control Systems Technology, vol 6 no 1, January 1998.
  // as reported on http://www.mstarlabs.com/control/znrule.html
  case PESSEN_INTEGRAL_PID:
    Kp = 0.7   * Ku;
    Ti = 0.4   * Pu;
    Td = 0.15  * Pu;
    break;
  case SOME_OVERSHOOT_PID:
    Kp = 0.33  * Ku;
    Ti = 0.5   * Pu;
    Td = 0.33  * Pu;
    break;
  case NO_OVERSHOOT_PID:
    Kp = 0.2   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.33  * Pu;
    break;
  // source of Tyreus-Luyben and Ciancone-Marlin rules:
  // "Autotuning of PID Controllers: A Relay Feedback Approach",
  //  by Cheng-Ching Yu, 2nd Edition, p.18
  case TYREUS_LUYBEN_PI: // for lag dominated processes
    Kp = Ku / 3.2;
    Ti = Pu / 0.45;
    Td = 0.0;  
    break;
  case TYREUS_LUYBEN_PID: // for lag dominated processes
    Kp = Ku / 2.2;
    Ti = Pu / 0.45;
    Td = Pu / 6.3;  
    break;
  case CIANCONE_MARLIN_PID: // for delay dominated processes
    Kp = Ku / 3.3;
    Ti = Pu / 4.4;
    Td = Pu / 8.1;  
    break;
  case CIANCONE_MARLIN_PI: // for delay dominated processes
    Kp = Ku / 3.3;
    Ti = Pu / 4.0;
    Td = 0.0;  
    break;
  case ZIEGLER_NICHOLS_PI:
  default:
    Kp = 0.4   * Ku;
    Ti = 0.8   * Pu;
    Td = 0.0;
  }

  // converged
  return true;
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
  if (value < 5) 
  {
    value = 5;
  }
  if (value < 25)
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
