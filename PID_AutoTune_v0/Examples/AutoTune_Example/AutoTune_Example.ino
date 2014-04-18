#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

// set simulation model
// define one, undef the others
#undef SIMULATION_MODEL_LAG_DOMINATED
#undef SIMULATION_MODEL_DELAY_DOMINATED
#define SIMULATION_MODEL_BALANCED_LAG_AND_DELAY
#undef SIMULATION_MODEL_LOAD_DISTURBANCE

// undefine to connect to the real world
#define USE_SIMULATION

// simulation model parameters
double kpmodel, taup, theta[50];
byte   deadTime;

// optional white noise variance for simulation
// NB variance of U(0,1) = 1/12
#define SQRT12_DIV_100 0.03464102
double noiseVariance;

// initial state
double input = 80, output=50, setpoint=180;

// default PID tuning
double kp=2, ki=0.5, kd=2;

// autotuner defaults
double outputStart = 5;
double aTuneStep = 30;
double aTuneNoise = 0.5; 
unsigned int aTuneLookBack = 5;
boolean tuning = false;

byte ATuneModeRemember;
unsigned long  modelTime, serialTime;
int modelStep, tuningStep;

PID myPID(&input, &output, &setpoint, kp, ki, kd, PID::DIRECT);
PID_ATune aTune(&input, &output);

extern Tuning tuningRule[];

void setup()
{

#if !defined (USE_SIMULATION)
  for(byte i = 0; i < 50; i++)
  {
    theta[i] = outputStart;
  }
#endif

  modelTime = 0;
  modelStep = 0;
  tuningStep = 0;
  
  // Setup the PID 
  myPID.SetMode(PID::AUTOMATIC);
  
  // set control type for autotuner
  // general considerations:
  // ZIEGLER_NICHOLS_PI is the default and is intended for good 
  //   noise rejection rather than servo control
  //   it is less robust than other rules, often
  //   with higher proportional gain and lower integral gain and often gives oscillatory results for lag-dominated processes
  // TYREUS_LUYBEN_PI is more conservative and considered better for lag-dominated processes
  // CIANCONE_MARLIN_PI is suggested for delay-dominated processes
  // AMIGOF_PI gives acceptable conservative tunings for most processes
  // PI tunings perform well for delay dominated processes
  // for lag-dominated processes PID tunings are better because
  // the derivative term allows higher integral gain
  //aTune.SetControlType(PID_ATune::AMIGOF_PI);
  //aTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PI);
  //aTune.SetControlType(PID_ATune::TYREUS_LUYBEN_PI);
  aTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
  //aTune.SetControlType(PID_ATune::NO_OVERSHOOT_PID);

  // set up simulation parameters
#if defined (SIMULATION_MODEL_LAG_DOMINATED)
  kpmodel = 1.0;
  taup = 100;
  deadTime = 5;
  noiseVariance = 0.1;
#endif

#if defined (SIMULATION_MODEL_BALANCED_LAG_AND_DELAY) || defined (SIMULATION_MODEL_LOAD_DISTURBANCE)
  kpmodel = 1.0;
  taup = 50;
  deadTime = 30;
  noiseVariance = 0.1;
#endif

#if defined (SIMULATION_MODEL_DELAY_DOMINATED)
  kpmodel = 1.0;
  taup = 20;
  deadTime = 50;
  noiseVariance = 0.1;
#endif

  if (tuning)
  {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }
  serialTime = 0;
  Serial.begin(9600);
  randomSeed(0);
}

void loop()
{
  unsigned long now = millis();

#if !defined (USE_SIMULATION)
    // pull the input in from the real world
    input = analogRead(0);
#endif
  
  if (tuning)
  {
    byte val = (aTune.Runtime());
    if (val != 0)
    {
      tuning = false;
    }
    if (!tuning)
    { 
      // we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
      
      // change set point
      setpoint = 130;
      
    }
  }
  else 
  {
    tuningStep = 0;
    myPID.Compute();
  }
  
#if defined (USE_SIMULATION)
  theta[deadTime] = output;
  if (now >= modelTime)
  {
    modelTime += 50; 
    modelStep++;
    tuningStep++;
    DoModel();
    
#if defined (SIMULATION_MODEL_LOAD_DISTURBANCE)
    if (tuningStep == 50)
    {
      Serial.println("process disturbed");
      kpmodel * 0.9;
    }
#endif  

  } 
  
#else // !defined USE_SIMULATION
   analogWrite(0, output); 
#endif
  
  //send-receive with processing if it's time
  if (millis() > serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
}

void changeAutoTune()
{
 if (!tuning)
  {
    // Set the output to the desired starting frequency.
    //output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { 
    // cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if (tuning)
  {
    Serial.println("tuning mode");
  } 
  else 
  {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if (Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   // press '1' to toggle autotuner on and off
   if (((b == '1') && !tuning) || ((b != '1') && tuning)) 
     changeAutoTune();
  }
}

void DoModel()
{
  // cycle the dead time
  for (byte i = 0; i < deadTime; i++)
  {
    theta[i] = theta[i + 1];
  }
  // compute the input
  input = (kpmodel / taup) * (theta[0] - outputStart) + input * (1 -  1 / taup) + 
          ((float)random(-50, 50)) * noiseVariance * SQRT12_DIV_100;
}
