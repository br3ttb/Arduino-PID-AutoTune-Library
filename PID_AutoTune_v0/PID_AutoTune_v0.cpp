/* Adapted by Brett Beauregard
 * from the MATLAB autotunerpid library
 * by William Spinelli
 *
 * Ported to C with additions by Tom Price
 */
 
#include <PID_AutoTune_v0.h>

#define MIN( x, y ) ( ( (x) < (y) ) ? (x) : (y) )
#define MAX( x, y ) ( ( (x) > (y) ) ? (x) : (y) )
#define BOUND( n, min, max ) MIN( MAX( (n), (min) ), (max) ) // NAN -> min


 // Constructor.  Links the autotune to a given pid
atune_t* atune_new( double* input, double* output )
{
	atune_t* p = malloc( sizeof( atune_t ) );
	p->input = input;
	p->output = output;
	p->Control_type = PI;
	p->Relay_hyst = 0.5; // relay relay_hysteresis parameter
	p->Running = FALSE;
	p->Relay_amp = 30; // relay amplitude parameter
	atune_set_lookback_sec( p, 10 );
	p->Last_time = millis;
	p->Dither = 0.0;
	//srandom( 0 ); // seed for random dither
	return p;
}

// Stop auto tune	
void atune_cancel( atune_t* p )
{
	p->Running = FALSE;
} 
 
// Calculate gain parameters
static void Atune_finish_up( atune_t* p )
{
	*p->output = p->output_start;
	
	// generate tuning parameters
	// using Second Ziegler-Nichols method (closed loop)
	p->Ku = 4 * p->Relay_amp / ( ( p->Abs_max - p->Abs_min ) * 3.14159265358979 ); // ultimate gain
	p->Pu = (double) ( p->Peak1 - p->Peak2 ) / 1000; // ultimate period
	
	// calculate gain parameters
	if ( p->Control_type == CLASSIC_PID )
	{
		p->Kp = 0.6   * p->Ku;
		p->Ti = 0.5   * p->Pu;
		p->Td = 0.125 * p->Pu;
	}
	else if ( p->Control_type == PESSEN )
	{
		p->Kp = 0.7   * p->Ku;
		p->Ti = 0.4   * p->Pu;
		p->Td = 0.15  * p->Pu;
	}
	else if ( p->Control_type == SOME_OVERSHOOT )
	{
		p->Kp = 0.33  * p->Ku;
		p->Ti = 0.5   * p->Pu;
		p->Td = 0.33  * p->Pu;
	}
	else if ( p->Control_type == NO_OVERSHOOT )
	{
		p->Kp = 0.2   * p->Ku;
		p->Ti = 0.5   * p->Pu;
		p->Td = 0.33  * p->Pu;
	}
	else // control_type == PI
	{
		p->Kp = 0.4   * p->Ku;
		p->Ti = 0.45  * p->Pu;
		p->Td = 0;
	}
}

// Similar to the pid compute function, returns TRUE when done
enum boolean atune_runtime( atune_t* p )
{
	p->Just_evaled = FALSE;
	if ( p->Peak_count > 9 && p->Running )
	{
		p->Running = FALSE;
		Atune_finish_up( p );
		return 1;
	}
	uint32_t now = millis;
	
	if ( ( now - p->Last_time ) < p->Sample_time ) 
	{
		return FALSE;
	}
	p->Last_time = now;
	double refval = *p->input;
	p->Just_evaled = TRUE;
	
	// dither input value to smooth quantization error
	if ( p->Dither > 0.0 )
	{
	  // generate random number from triangular probability density function 
	  // centred on 0 and with range (-dither, dither)
	  double n = ( ( random() + random() ) / RANDOM_MAX - 1.0 ) * p->Dither;
	  refval += n;
	}
	
	if ( !p->Running )
	{ 
		//initialize working variables the first time around
		p->Peak_type = NON_PEAK;
		p->Peak_count = 0;
		p->Just_changed = FALSE;
		p->Abs_max = refval;
		p->Abs_min = refval;
		p->Set_point = refval;
		p->Running = TRUE;
		p->Init_count = 0;
		p->output_start = *p->output;
		*p->output = p->output_start + p->Relay_amp;
	}
	else
	{
		p->Abs_max = MAX( p->Abs_max, refval );
		p->Abs_min = MIN( p->Abs_min, refval );
	}
	
	// oscillate the output base on the input's relation to the set point	
	if ( refval > p->Set_point + p->Relay_hyst ) 
	{
		*p->output = p->output_start - p->Relay_amp;
	} else if ( refval < p->Set_point - p->Relay_hyst ) 
	{
		*p->output = p->output_start + p->Relay_amp;
	}
	
	// identify peaks 
	p->Is_max = TRUE; 
	p->Is_min = TRUE;
	uint8_t i;
	for( i = p->Nlookback - 1; i >= 0; i-- )
	{
		double val = p->Last_inputs[ i ];
		if ( p->Is_max ) 
		{
			p->Is_max = ( refval > val );
		}
		if ( p->Is_min ) 
		{
			p->Is_min = ( refval < val );
		}
		p->Last_inputs[ i + 1 ] = p->Last_inputs[ i ];
	}
	p->Last_inputs[ 0 ] = refval;  
	if ( p->Nlookback < 9 )
	{  
		//we don't want to trust the maxes or mins until the inputs array has been filled
		p->Init_count++;
		return FALSE;
	}
  
	if ( p->Is_max )
	{
		if ( p->Peak_type == NON_PEAK ) 
		{
			p->Peak_type = MAXIMUM;
		}
		if ( p->Peak_type == MINIMUM )
		{
			p->Peak_type = MINIMUM;
			p->Just_changed = TRUE;
			p->Peak2 = p->Peak1;
		}
		p->Peak1 = now;
		p->Peaks[ p->Peak_count ] = refval;
	}
	else if ( p->Is_min )
	{
		if ( p->Peak_type == NON_PEAK ) 
		{
			p->Peak_type = MINIMUM;
		}
		if ( p->Peak_type == MAXIMUM )
		{
			p->Peak_type = MINIMUM;
			p->Peak_count++;
			p->Just_changed = TRUE;
		}    
		if ( p->Peak_count < 10 ) 
		{
			p->Peaks[ p->Peak_count ] = refval;
		}
	}

	if ( p->Just_changed && p->Peak_count > 2 )
	{ 
		// we've transitioned. check if we can autotune based on the last peaks
		double avgSeparation = 
			( 
				abs( 
					p->Peaks[ p->Peak_count - 1 ] - 
					p->Peaks[ p->Peak_count - 2 ] 
				) + 
				abs( 
					p->Peaks[ p->Peak_count - 2 ] - 
					p->Peaks[ p->Peak_count - 3 ] 
				) 
			) / 2;
		if ( avgSeparation < 0.05 * ( p->Abs_max - p->Abs_min ) )
		{
			Atune_finish_up( p );
			p->Running = FALSE;
			return TRUE;	 
		}
	}
	p->Just_changed = FALSE;
	return FALSE;
}

// get computed proportional gain
double atune_get_Kp( atune_t* p )
{
	return p->Kp;
}

// get computed integral gain
double atune_get_Ki( atune_t* p )
{
	return p->Kp / p->Ti; 
}

// get computed derivative gain
double atune_get_Kd( atune_t* p )
{
	return p->Kp * p->Td; 
}

// set relay amplitude
void atune_set_relay_amp( atune_t* p, double step )
{
	p->Relay_amp = step;
}

// get relay amplitude
double atune_get_relay_amp( atune_t* p )
{
	return p->Relay_amp;
}

// set formula for calculating the tuning parameters
void atune_set_control_type( atune_t* p, enum control type )
{
	p->Control_type = type;
}

// get formula for calculating the tuning parameters
enum control atune_get_control_type( atune_t* p )
{
	return p->Control_type;
}
	
// set relay relay_hysteresis
// The auto tune will ignore chatter above and below this range
// This parameter needs to be set_ accurately
void atune_set_relay_hyst( atune_t* p, double band )
{
	p->Relay_hyst = band;
}

// get relay relay_hysteresis
double atune_get_relay_hyst( atune_t* p )
{
	return p->Relay_hyst;
}

// set look back distance to identify peaks
void atune_set_lookback_sec( atune_t* p, uint16_t value )
{
    if ( value < 1 ) 
	{
		value = 1;
	}	
	if ( value < 25 )
	{
		p->Nlookback = value * 4;
		p->Sample_time = 250;
	}
	else
	{
		p->Nlookback = 100;
		p->Sample_time = value * 10;
	}
}

// get look back distance to identify peaks
uint16_t atune_get_lookback_sec( atune_t* p )
{
	return p->Nlookback * p->Sample_time / 1000;
}

// set range of noise added to input to smooth quantization errors
// set to smallest step value in input range
void atune_set_dither( atune_t* p, double new_dither )
{
	if ( new_dither >= 0 )
	{
		p->Dither = new_dither;
	}
}

// get dither range
double atune_get_dither( atune_t* p )
{
	return p->Dither;
}