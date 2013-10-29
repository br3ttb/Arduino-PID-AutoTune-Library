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
  double iMax = lastInputs[0];
  double iMin = lastInputs[0];
  for (int i = inputCount; i >= 0; i--)
  {
    if (iMax < lastInputs[i])
    {
      iMax = lastInputs[i];
    }
    if (iMin > lastInputs[i])
    {
      iMin = lastInputs[i];
    }
  } 
  /*
  Serial.println(iMax);
  Serial.println(iMin);
  Serial.println((iMax - iMin) <= 2 * noiseBand);
  */
#ifdef DITHER
  return ((iMax - iMin) <= 2 * noiseBand + 2 * dither);
#else
  return ((iMax - iMin) <= 2 * noiseBand);
#endif
}

bool PID_ATune::Runtime()
{
  // check ready for new input
  unsigned long now = millis();

  if (state == AUTOTUNER_OFF)
  { 
    // initialize working variables the first time around
    peakType = NOT_A_PEAK;
    inputCount = 0;
    peakCount = 0;
    stepCount = 0;
    setpoint = *input;
    outputStart = *output;
    originalNoiseBand = noiseBand;
    relayBias = 0;
    peakTime[0] = now;
    stepTime[0] = now;
    sumInputSinceLastStep[0] = 0;
    newNoiseBand = noiseBand;
    
    // move to new state
    if (controlType == AMIGOF_PI)
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
  if ((state == RELAY_STEP_UP) && (refVal > setpoint + noiseBand))
  {
    state = RELAY_STEP_DOWN;
    justChanged = true;
    noiseBand = newNoiseBand;
  }
  else if ((state == RELAY_STEP_DOWN) && (refVal < setpoint - noiseBand))
  {
    state = RELAY_STEP_UP;
    justChanged = true;
    noiseBand = newNoiseBand;
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
  if (((byte) state & (STEADY_STATE_AFTER_STEP_UP | RELAY_STEP_UP)) > 0)
  {
    *output = outputStart + oStep + relayBias;
  }
  else if (state == RELAY_STEP_DOWN)
  {
    *output = outputStart - oStep + relayBias;
  }
  
  /*
  Serial.print("refVal ");
  Serial.println(refVal);
  Serial.print("setpoint ");
  Serial.println(setpoint);
  Serial.print("output ");
  Serial.println(*output);
  Serial.print("state ");
  Serial.println(state);
  */

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
  for (int i = inputCount - 1; i >= 0; i--)
  {
    double val = lastInputs[i];
    if (isMax)
    {
      isMax = (refVal >= val);
    }
    if (isMin) 
    {
      isMin = (refVal <= val);
    }
    lastInputs[i + 1] = val;
  }
  lastInputs[0] = refVal; 

  // for AMIGOf tuning rule, perform an initial
  // step change to calculate process gain K_process
  // this may be very slow for lag-dominated processes
  // and may never terminate for integrating processes 
  if (((byte) state & (STEADY_STATE_AT_BASELINE | STEADY_STATE_AFTER_STEP_UP)) > 0)
  {
    // check that all the recent inputs are 
    // equal give or take expected noise
    /*
    Serial.print(F("state "));
    Serial.println(state);
    */
    if (CheckStable())
    {
      stepTime[0] = now;
      double avgInput = 0.0;
      for (byte i = 0; i < inputCount + 1; i++)
      {
        avgInput += lastInputs[i];
      }
      avgInput /= inputCount + 1;
      /*
      Serial.println(avgInput); 
      */
      if (state == STEADY_STATE_AT_BASELINE)
      {
        state = STEADY_STATE_AFTER_STEP_UP;
        peaks[0] = avgInput;  
        inputCount = 0;
        return false;
      }
      // else state == STEADY_STATE_AFTER_STEP_UP
      // calculate process gain
      K_process = (avgInput - peaks[0]) / oStep;
      /*
      Serial.print(F("Kp "));
      Serial.println(K_process);
      */
      if (K_process < 1e-10) // zero
      {
        state = AUTOTUNER_OFF;
        return false;
      }
      state = RELAY_STEP_DOWN;
      sumInputSinceLastStep[0] = 0;
      return false;
    }
    else
    {
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
    /*
    Serial.println();
    Serial.println(peakCount);
    Serial.println(refVal);
    Serial.print(F("peak type "));
    Serial.println(peakType);
    Serial.println(isMin);
    Serial.println(isMax);
    Serial.println();
    for (int i = inputCount; i >= 0; i--)
      Serial.println(lastInputs[i]);
    Serial.println();
    */   
  }
  if (justChanged)
  {
    for (byte i = (peakCount > 2 ? 2 : peakCount); i > 0; i--)
    {
      peakTime[i] = PeakTime[i - 1];
    }
    /*
    Serial.println(F("peaks"));
    for (byte i = 0; i < (peakCount > 2 ? 2 : peakCount); i++)
      Serial.println(peaks[i]);
    */
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
    for (i = peakCount - 1; i > peakCount - 4; i--)
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
    /*
    Serial.print(F("amplitude "));
    Serial.println(inducedAmplitude);
    Serial.println(absMin);
    Serial.println(absMax);
    Serial.println((0.5 * (absMax - absMin) - inducedAmplitude) / inducedAmplitude);
    */

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
    if (controlType == AMIGOF_PI)
    {
      // calculate phase lag
      // NB hysteresis = 2 * noiseBand;
      phaseLag = CONST_PI - asin(2.0 * noiseBand / inducedAmplitude); 
      /*
      Serial.print(F("phase lag "));
      Serial.println(phaseLag / CONST_PI * 180.0);
      */
    }

    // check that phase lag is within acceptable bounds, ideally between 120° and 140°
    // but 115° to 145° is OK
    else if ((controlType == AMIGOF_PI) && (abs(phaseLag / CONST_PI * 180.0 - 130.0) > 15.0))
    {
      // phase lag outside the desired range
      // set noiseBand to new estimate
      // NB noiseBand = 0.5 * hysteresis
      newNoiseBand = 0.5 * (inducedAmplitude * CONST_SQRT2_DIV_2);
      /*
      Serial.print(F("newNoiseBand "));
      Serial.println(newNoiseBand);   
      */ 
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
  /*
  Serial.print(F("ultimate gain "));
  Serial.println(1 / Ku);
  */

  // calculate ultimate period in seconds
  double Pu = (double) (peakTime[0] - peakTime[2]) / 1000.0; 
  
  // calculate gain ratio
  double kappa_phi = (1 / Ku) / K_process;
  /*
  Serial.print(F("gain ratio kappa "));
  Serial.println(kappa_phi);
  */
  
  phaseLag = CONST_PI - asin(2.0 * noiseBand / inducedAmplitude); 
  if (2.0 * noiseBand > inducedAmplitude)
  {
    phaseLag = CONST_PI / 2;
  }
  /*
  Serial.print(F("phase lag "));
  Serial.println(phaseLag / CONST_PI * 180.0);
  */
  noiseBand = originalNoiseBand;

  // calculate gain parameters
  switch(controlType)
  {
  // AMIGOf is slow to tune, especially for lag-dominated processes, because it
  // requires an estimate of the process gain which is implemented in this
  // routine by steady state change in process variable after step change in set point
  // It is intended to give robust tunings for both lag- and delay- dominated processes
  case AMIGOF_PI:
    Kp = (( 2.50 - 0.92 * phaseLag) / (1.0 + (10.75 - 4.01 * phaseLag) * kappa_phi)      ) * Ku;
    Ti = ((-3.05 + 1.72 * phaseLag) / pow(1.0 + (-6.10 + 3.44 * phaseLag) * kappa_phi, 2)) * Pu;
    Td = 0.0;
    break;
  // source for Pessen Integral, Some Overshoot, and No Overshoot rules:
  // "Rule-Based Autotuning Based on Frequency Domain Identification" 
  // by Anthony S. McCormack and Keith R. Godfrey
  // IEEE Transactions on Control Systems Technology, vol 6 no 1, January 1998.
  // as reported on http://www.mstarlabs.com/control/znrule.html
  // These are modifications of Ziegler-Nichols
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
  // Tyreus-Luyben is more conservative than Ziegler-Nichols
  // and is preferred for lag dominated processes
  case TYREUS_LUYBEN_PI: 
    Kp = Ku / 3.2;
    Ti = Pu / 0.45;
    Td = 0.0;  
    break;
  case TYREUS_LUYBEN_PID: 
    Kp = Ku / 2.2;
    Ti = Pu / 0.45;
    Td = Pu / 6.3;  
    break;
  // Ciancone-Marlin is preferred for delay dominated processes
  case CIANCONE_MARLIN_PID: 
    Kp = Ku / 3.3;
    Ti = Pu / 4.4;
    Td = Pu / 8.1;  
    break;
  case CIANCONE_MARLIN_PI: 
    Kp = Ku / 3.3;
    Ti = Pu / 4.0;
    Td = 0.0;  
    break;
  // Ziegler-Nichols is intended for best disturbance rejection
  // can lack robustness especially for lag dominated processes
  case ZIEGLER_NICHOLS_PID: 
    Kp = 0.6   * Ku;
    Ti = 0.5   * Pu;
    Td = 0.125 * Pu;
    break;
  case ZIEGLER_NICHOLS_PI: 
  default:
    Kp = 0.45  * Ku;
    Ti = Pu / 1.2;
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

#ifdef DITHER
void PID_ATune::SetDither(double newDither)
{
  if (newDither >= 0.0)
  {
    dither = newDither;
  }
}
#endif
