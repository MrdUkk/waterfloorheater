/*
  This class combines two libs written by Brett Beauregard contact: br3ttb@gmail.com
  "Arduino PID Library" and "Arduino PID Autotune Library"
  all this libraries is licensed under a GPLv3 License
  
  
  Modifications (c) 2018-2019 dUkk
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PIDLib.h"

/*Constructor (...)*********************************************************
      The parameters specified here are those for for which we can't set up
      reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID()
{
}

/* Compute() **********************************************************************
       This, as they say, is where the magic happens.  this function should be called
     every time "void loop()" executes.  the function will decide for itself whether a new
     pid Output needs to be computed.  returns true when the output is computed,
     false when nothing has been done.
 **********************************************************************************/
bool PID::Compute(double *mySetpoint)
{
  if (!inAuto) return false;
  /*Compute all the working error variables*/
  double input = *myInput;
  double error = *mySetpoint - input;
  double dInput = (input - lastInput);
  outputSum += (ki * error);

  /*Add Proportional on Measurement, if P_ON_M is specified*/
  if (!pOnE) outputSum -= kp * dInput;

  if (outputSum > outMax) outputSum = outMax;
  else if (outputSum < outMin) outputSum = outMin;

  /*Add Proportional on Error, if P_ON_E is specified*/
  double output;
  if (pOnE) output = kp * error;
  else output = 0;

  /*Compute Rest of PID Output*/
  output += outputSum - kd * dInput;

  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;
  *myOutput = output;

  /*Remember some variables for next time*/
  lastInput = input;
  return true;
}

/* SetTunings(...)*************************************************************
   This function allows the controller's dynamic performance to be adjusted.
   it's called automatically from the constructor, but tunings can also
   be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  pOn = POn;
  pOnE = POn == P_ON_E;

  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

/* SetTunings(...)*************************************************************
   Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd) {
  SetTunings(Kp, Ki, Kd, pOn);
}

/* SetSampleTime(...) *********************************************************
   sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio  = (double)NewSampleTime
                    / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

/* SetOutputLimits(...)****************************************************
       This function will be used far more often than SetInputLimits.  while
    the input to the controller will generally be in the 0-1023 range (which is
    the default already,)  the output will be a little different.  maybe they'll
    be doing a time window and will need 0-8000 or something.  or maybe they'll
    want to clamp it from 0-125.  who knows.  at any rate, that can all be done
    here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (inAuto)
  {
    if (*myOutput > outMax) *myOutput = outMax;
    else if (*myOutput < outMin) *myOutput = outMin;

    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
  }
}

/* SetMode(...)****************************************************************
   Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
   when the transition from manual to auto occurs, the controller is
   automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto)
  { /*we just went from manual to auto*/
    Reset();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
 	does all the things that need to happen to ensure a bumpless transfer
    from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize(double* Input, double* Output, double Kp, double Ki, double Kd, int POn)
{
  myOutput = Output;
  myInput = Input;
  inAuto = false;

  PID::SetOutputLimits(0, 255);  //default output limit corresponds to the arduino pwm limits
  SampleTime = 100;              //default Controller Sample Time is 0.1 seconds

  PID::SetTunings(Kp, Ki, Kd, POn);

  Reset();
}

void PID::Reset()
{
  outputSum = *myOutput;
  lastInput = *myInput;
  if (outputSum > outMax) outputSum = outMax;
  else if (outputSum < outMin) outputSum = outMin;
}

/* Status Funcions*************************************************************
   Just because you set the Kp=-1 doesn't mean it actually happened.  these
   functions query the internal state of the PID.  they're here for display
   purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
int PID::GetMode() {
  return  inAuto ? AUTOMATIC : MANUAL;
}

void PID::ATInitialize(double* Input, double* Output)
{
  AutoTuneData.input = Input;
  AutoTuneData.output = Output;
  AutoTuneData.controlType = 0 ; //default to PI
  AutoTuneData.noiseBand = 0.5;
  AutoTuneData.running = false;
  AutoTuneData.oStep = 30;
  ATSetLookbackSec(10);
}

void PID::ATCancel()
{
  AutoTuneData.running = false;
}

int PID::ATCompute()
{
  if (AutoTuneData.peakCount > 9 && AutoTuneData.running)
  {
    AutoTuneData.running = false;
    ATFinishUp();
    return true;
  }

  double refVal = *AutoTuneData.input;
  if (!AutoTuneData.running)
  { //initialize working variables the first time around
    AutoTuneData.peakType = 0;
    AutoTuneData.peakCount = 0;
    AutoTuneData.justchanged = false;
    AutoTuneData.absMax = refVal;
    AutoTuneData.absMin = refVal;
    AutoTuneData.setpoint = refVal;
    AutoTuneData.running = true;
    AutoTuneData.outputStart = *AutoTuneData.output;
    *AutoTuneData.output = AutoTuneData.outputStart + AutoTuneData.oStep;
  }
  else
  {
    if (refVal > AutoTuneData.absMax) AutoTuneData.absMax = refVal;
    if (refVal < AutoTuneData.absMin) AutoTuneData.absMin = refVal;
  }

  //oscillate the output base on the input's relation to the setpoint
  if (refVal > AutoTuneData.setpoint + AutoTuneData.noiseBand) *AutoTuneData.output = AutoTuneData.outputStart - AutoTuneData.oStep;
  else if (refVal < AutoTuneData.setpoint - AutoTuneData.noiseBand) *AutoTuneData.output = AutoTuneData.outputStart + AutoTuneData.oStep;

  //bool isMax=true, isMin=true;
  AutoTuneData.isMax = true; AutoTuneData.isMin = true;
  //id peaks
  for (int i = AutoTuneData.nLookBack - 1; i >= 0; i--)
  {
    double val = AutoTuneData.lastInputs[i];
    if (AutoTuneData.isMax) AutoTuneData.isMax = refVal > val;
    if (AutoTuneData.isMin) AutoTuneData.isMin = refVal < val;
    AutoTuneData.lastInputs[i + 1] = AutoTuneData.lastInputs[i];
  }
  AutoTuneData.lastInputs[0] = refVal;
  if (AutoTuneData.nLookBack < 9)
  { //we don't want to trust the maxes or mins until the inputs array has been filled
    return false;
  }

  if (AutoTuneData.isMax)
  {
    if (AutoTuneData.peakType == 0) AutoTuneData.peakType = 1;
    if (AutoTuneData.peakType == -1)
    {
      AutoTuneData.peakType = 1;
      AutoTuneData.justchanged = true;
      AutoTuneData.peak2 = AutoTuneData.peak1;
    }
    AutoTuneData.peak1 = millis();
    AutoTuneData.peaks[AutoTuneData.peakCount] = refVal;

  }
  else if (AutoTuneData.isMin)
  {
    if (AutoTuneData.peakType == 0) AutoTuneData.peakType = -1;
    if (AutoTuneData.peakType == 1)
    {
      AutoTuneData.peakType = -1;
      AutoTuneData.peakCount++;
      AutoTuneData.justchanged = true;
    }

    if (AutoTuneData.peakCount < 10) AutoTuneData.peaks[AutoTuneData.peakCount] = refVal;
  }

  if (AutoTuneData.justchanged && AutoTuneData.peakCount > 2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    double avgSeparation = (abs(AutoTuneData.peaks[AutoTuneData.peakCount - 1] - AutoTuneData.peaks[AutoTuneData.peakCount - 2]) + abs(AutoTuneData.peaks[AutoTuneData.peakCount - 2] - AutoTuneData.peaks[AutoTuneData.peakCount - 3])) / 2;
    if ( avgSeparation < 0.05 * (AutoTuneData.absMax - AutoTuneData.absMin))
    {
      ATFinishUp();
      AutoTuneData.running = false;
      return true;

    }
  }
  AutoTuneData.justchanged = false;
  return false;
}

void PID::ATFinishUp()
{
  *AutoTuneData.output = AutoTuneData.outputStart;
  //we can generate tuning parameters!
  AutoTuneData.Ku = 4 * (2 * AutoTuneData.oStep) / ((AutoTuneData.absMax - AutoTuneData.absMin) * 3.14159);
  AutoTuneData.Pu = (double)(AutoTuneData.peak1 - AutoTuneData.peak2) / 1000;
}

void PID::ATSetLookbackSec(int value)
{
  if (value < 1) value = 1;

  if (value < 25)
  {
    AutoTuneData.nLookBack = value * 4;
    //AutoTuneData.sampleTime = 250;
  }
  else
  {
    AutoTuneData.nLookBack = 100;
    //AutoTuneData.sampleTime = value * 10;
  }
}
void PID::ATSetOutputStep(double Step)
{
  AutoTuneData.oStep = Step;
}

void PID::ATSetControlType(int Type) //0=PI, 1=PID
{
  AutoTuneData.controlType = Type;
}

void PID::ATSetNoiseBand(double Band)
{
  AutoTuneData.noiseBand = Band;
}

double PID::ATGetKp()
{
  return AutoTuneData.controlType == 1 ? 0.6 * AutoTuneData.Ku : 0.4 * AutoTuneData.Ku;
}

double PID::ATGetKi()
{
  return AutoTuneData.controlType == 1 ? 1.2 * AutoTuneData.Ku / AutoTuneData.Pu : 0.48 * AutoTuneData.Ku / AutoTuneData.Pu; // Ki = Kc/Ti
}

double PID::ATGetKd()
{
  return AutoTuneData.controlType == 1 ? 0.075 * AutoTuneData.Ku * AutoTuneData.Pu : 0; //Kd = Kc * Td
}
