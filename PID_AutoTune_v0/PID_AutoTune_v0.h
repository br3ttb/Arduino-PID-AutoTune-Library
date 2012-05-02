#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION	0.0.0

#define PI 0
#define PID 1
#define PESSEN 2
#define SOME_OVERSHOOT 3
#define NO_OVERSHOOT 4

class PID_ATune
{
	public:
	//commonly used functions **************************************************************************
		PID_ATune(double*, double*);                       // * Constructor.  links the Autotune to a given PID
		int Runtime();						   			   	// * Similar to the PID Compute function, returns non 0 when done
		void Cancel();									   	// * Stops the AutoTune	

		void SetOutputStep(double);						   	// * how far above and below the starting value will the output step?	
		double GetOutputStep();							   	// 

		void SetControlType(int); 						   	// * Determies if the tuning parameters returned will be PI (D=0)
		int GetControlType();							   	//   or PID.  (0=PI, 1=PID, 2=Pessen rule, 3="some overshoot" rule, 4="no overshoot" rule)			

		void SetLookbackSec(int);							// * how far back are we looking to identify peaks
		int GetLookbackSec();								//

		void SetNoiseBand(double);							// * the autotune will ignore signal chatter smaller than this value
		double GetNoiseBand();								//   this should be accurately set

		void SetDither(double);								// * noise added to input to smooth quantization errors
		double GetDither();									//   set to smallest step value in input range

		double GetKp();										// * once autotune is complete, these functions contain the
		double GetKi();										//   computed tuning parameters.  
		double GetKd();										//

	private:
		void FinishUp();
		bool isMax, isMin;
		double *input, *output;
		double setpoint;
		double noiseBand;
		int controlType;
		double dither, randomNumber;
		bool running;
		unsigned long peak1, peak2, lastTime;
		int sampleTime;
		int nLookBack;
		int peakType;
		double lastInputs[100];
		double peaks[10];
		int peakCount;
		bool justchanged;
		bool justevaled;
		int initCount;
		double absMax, absMin;
		double oStep;
		double outputStart;
		double Ku, Pu, Kp, Ti, Td;
};
#endif

