/*
	This class combines two libs written by Brett Beauregard contact: br3ttb@gmail.com
  "Arduino PID Library" and "Arduino PID Autotune Library"
  all this libraries is licensed under a GPLv3 License
  
  
  Modifications (c) 2018-2019 dUkk
*/

#ifndef PIDLIB_h
#define PIDLIB_h

struct autotune_T
{
  bool isMax, isMin;
  double *input, *output;
  double setpoint;
  double noiseBand;
  int controlType;
  bool running;
  unsigned long peak1, peak2;
  int nLookBack;
  int peakType;
  double lastInputs[101];
  double peaks[10];
  int peakCount;
  bool justchanged;
  double absMax, absMin;
  double oStep;
  double outputStart;
  double Ku, Pu;
};

class PID
{


  public:

    //Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

    //commonly used functions **************************************************************************
    void Initialize(double*, double*, double, double, double, int);

    PID();

    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute(double *mySetpoint);                       // * performs the PID calculation.  it should be
    //   called every time loop() cycles. ON/OFF and
    //   calculation frequency can be set using SetMode
    //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
    //   it's likely the user will want to change this depending on
    //   the application



    //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the
                    double);         	    //   constructor, this function gives the user the option
    //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, int);

    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
    //   the PID calculation is performed.  default is 100



    //Display functions ****************************************************************
    double GetKp();						  // These functions query the pid for interal values.
    double GetKi();						  //  they were created mainly for the pid front-end,
    double GetKd();						  // where it's important to know what is actually
    int GetMode();						  //  inside the PID.

    void ATInitialize(double* Input, double* Output);
    int ATCompute();						   			   	// * Similar to the PID Compue function, returns non 0 when done
    void ATCancel();									   	// * Stops the AutoTune
    void ATSetLookbackSec(int);							// * how far back are we looking to identify peaks
    void ATSetNoiseBand(double);
    void ATSetControlType(int);
    void ATSetOutputStep(double);
    double ATGetKp();
    double ATGetKi();
    double ATGetKd();
    void Reset();

  private:

    void ATFinishUp();
    autotune_T AutoTuneData;

    double dispKp;				// * we'll hold on to the tuning parameters in user-entered
    double dispKi;				//   format for display purposes
    double dispKd;				//

    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    int pOn;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the
                                  //   PID, freeing the user from having to constantly tell us
    //   what these values are.  with pointers we'll just know.

    double outputSum, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto, pOnE;
};
#endif
