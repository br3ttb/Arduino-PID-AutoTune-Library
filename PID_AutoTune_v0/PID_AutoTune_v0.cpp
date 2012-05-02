#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <PID_AutoTune_v0.h>

/* Adapted by Brett Beauregard
 * from the MATLAB autotunerPID library
 * by William Spinelli
 *
 * additional code by Tom Price
 */

PID_ATune::PID_ATune(double* Input, double* Output)
{
	input = Input;
	output = Output;
	controlType = PI;
	noiseBand = 0.5; // relay hysteresis parameter
	running = false;
	oStep = 30; // relay amplitude parameter
	SetLookbackSec( 10 );
	lastTime = millis();
	dither = 0;
	randomSeed( 0 );
}

void PID_ATune::Cancel()
{
	running = false;
} 
 
int PID_ATune::Runtime()
{
	justevaled = false;
	if( peakCount > 9 && running )
	{
		running = false;
		FinishUp();
		return 1;
	}
	unsigned long now = millis();
	
	if( ( now - lastTime ) < sampleTime ) return false;
	lastTime = now;
	double refVal = *input;
	justevaled = true;
	
	// dither input value to smooth quantization error
	if ( dither > 0.0 )
	{
	  // generate random number from triangular probability density function 
	  // centred on 0 and with range (-dither, dither)
	  randomNumber = ( ( random( 1, 65535 ) + random( 1, 65535 ) ) / 65536.0 - 1.0 ) * dither;
	  refVal += randomNumber;
	}
	
	if( !running )
	{ //initialize working variables the first time around
		peakType = 0;
		peakCount = 0;
		justchanged = false;
		absMax = refVal;
		absMin = refVal;
		setpoint = refVal;
		running = true;
		initCount = 0;
		outputStart = *output;
		*output = outputStart + oStep;
	}
	else
	{
		if( refVal > absMax ) absMax = refVal;
		if( refVal < absMin ) absMin = refVal;
	}
	
	//oscillate the output base on the input's relation to the setpoint
	
	if( refVal > setpoint + noiseBand ) *output = outputStart - oStep;
	else if ( refVal < setpoint - noiseBand ) *output = outputStart + oStep;
	
	//bool isMax=true, isMin=true;
	isMax = true; isMin = true;
	//id peaks 
	for( int i = nLookBack - 1; i >= 0; i-- )
	{
		double val = lastInputs[ i ];
		if( isMax ) isMax = ( refVal > val );
		if( isMin ) isMin = ( refVal < val );
		lastInputs[ i + 1 ] = lastInputs[ i ];
	}
	lastInputs[ 0 ] = refVal;  
	if( nLookBack < 9 )
	{  //we don't want to trust the maxes or mins until the inputs array has been filled
		initCount++;
		return 0;
	}
  
	if( isMax )
	{
		if( peakType == 0 ) peakType = 1;
		if( peakType == -1 )
		{
		  peakType = 1;
		  justchanged = true;
		  peak2 = peak1;
		}
		peak1 = now;
		peaks[ peakCount ] = refVal;
	}
	else if( isMin )
	{
		if( peakType == 0 ) peakType = -1;
		if( peakType == 1 )
		{
			peakType = -1;
			peakCount++;
			justchanged = true;
		}    
		if( peakCount < 10 ) peaks[ peakCount ] = refVal;
	}

	if( justchanged && peakCount > 2 )
	{ //we've transitioned.  check if we can autotune based on the last peaks
		double avgSeparation = ( abs( peaks[ peakCount - 1 ] - peaks[ peakCount - 2 ] ) + abs( peaks[ peakCount - 2 ] - peaks[ peakCount - 3 ] ) ) / 2;
		if( avgSeparation < 0.05 * ( absMax - absMin ) )
		{
			FinishUp();
			running = false;
			return 1;	 
		}
	}
	justchanged = false;
	return 0;
}

void PID_ATune::FinishUp()
{
	*output = outputStart;
	
	// generate tuning parameters
	// using Second Ziegler-Nichols method (closed loop)
	Ku = 4 * oStep / ( ( absMax - absMin ) * 3.14159265358979 ); // ultimate gain
	Pu = (double) ( peak1 - peak2 ) / 1000; // ultimate period
	
	// calculate gain parameters
	if ( controlType == PID )
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
	else // controlType == PI
	{
		Kp = 0.4   * Ku;
		Ti = 0.45  * Pu;
		Td = 0;
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
	return Kd = Kp * Td; 
}

void PID_ATune::SetOutputStep(double Step)
{
	oStep = Step;
}

double PID_ATune::GetOutputStep()
{
	return oStep;
}

void PID_ATune::SetControlType(int Type)
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
    if( value < 1 ) value = 1;
	
	if( value < 25 )
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

void PID_ATune::SetDither(int value)
{
	dither = value;
}

double PID_ATune::GetDither()
{
	return dither;
}
