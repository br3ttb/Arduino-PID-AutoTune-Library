#ifndef pid_autotune_v0
#define pid_autotune_v0
#define pid_autotune_LIBRARY_VERSION	0.0.0

#include <avr/io.h>
#include <PID_v1.h>

enum control
{
	PI = 0,	
	CLASSIC_PID = 1,
	PESSEN = 2,
	SOME_OVERSHOOT = 3,
	NO_OVERSHOOT = 4
};

enum peak
{
	MINIMUM = -1,
	NON_PEAK = 0,
	MAXIMUM = 1
};

typedef struct
{
// public
	double* input;
	double* output;
	
// private
	double Set_point;
	enum control Control_type;
	double Dither;
	enum boolean Running;
	enum boolean Is_max;
	enum boolean Is_min;
	uint32_t Peak1;
	uint32_t Peak2;
	uint32_t Last_time;
	uint32_t Sample_time;
	uint8_t Nlookback;
	enum peak Peak_type;
	double Last_inputs[100];
	double Peaks[10];
	uint8_t Peak_count;
	enum boolean Just_changed;
	enum boolean Just_evaled;
	uint8_t Init_count;
	double Abs_max;
	double Abs_min;
	double Relay_amp;
	double Relay_hyst;
	double Output_start;
	double Ku, Pu, Kp, Ti, Td;
} atune_t;

// public:
	
//commonly used functions **************************************************************************
	atune_t* atune_new( double*, double* );			// * Constructor.  Links the autotune to a given pid
	enum boolean atune_runtime( atune_t* );			// * Similar to the pid compute function, returns TRUE when done
	void atune_cancel( atune_t* );				// * Stops the autotune	

	void atune_set_relay_amp( atune_t*, double );		// * set relay amplitude	
	double atune_get_relay_amp( atune_t* );			// * get relay amplitude

	void atune_set_control_type( atune_t*, uint8_t );	// * set formula for calculating the tuning parameters
	enum control atune_get_control_type( atune_t* );	// * get formula for calculating the tuning parameters			

	void atune_set_lookback_sec( atune_t*, uint16_t );	// * set look back distance to identify peaks
	uint16_t atune_get_lookback_sec( atune_t* );		// * get look back distance to identify peak

	void atune_set_relay_hyst( atune_t*, double );		// * set relay relay_hysteresis
	double atune_get_relay_hyst( atune_t* );		// * get relay relay_hysteresis

	void atune_set_dither( atune_t*, double );		// * noise added to input to smooth quantization errors
	double atune_get_dither( atune_t* );			//   set to smallest step value in input range

	double atune_get_Kp( atune_t* );			// * once autotune is complete, these functions contain the
	double atune_get_Ki( atune_t* );			//   computed tuning parameters.  
	double atune_get_Kd( atune_t* );			//

/*	
private:
	void Atune_finish_up(atune_t*);				// * calculate gain parameters
*/

#endif