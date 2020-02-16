#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1


// #define PID_CONTROLLER_DOUBLE_PRECISION

#ifdef PID_CONTROLLER_DOUBLE_PRECISION
#define PID_CONTROLLER_VARIABLES_TYPE double
#else 
#define PID_CONTROLLER_VARIABLES_TYPE float
#endif

typedef PID_CONTROLLER_VARIABLES_TYPE real;

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
    PID(real*, real*, real*,        // * constructor.  links the PID to the Input, Output, and 
        real, real, real, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(real*, real*, real*,        // * constructor.  links the PID to the Input, Output, and 
        real, real, real, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(real, real); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(real, real,       // * While most users will set the tunings once in the 
                    real);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(real, real,       // * overload for specifying proportional mode
                    real, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	real GetKp();						  // These functions query the pid for interal values.
	real GetKi();						  //  they were created mainly for the pid front-end,
	real GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	real dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	real dispKi;				//   format for display purposes
	real dispKd;				//
    
	real kp;                  // * (P)roportional Tuning Parameter
    real ki;                  // * (I)ntegral Tuning Parameter
    real kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

    real *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    real *myOutput;             //   This creates a hard link between the variables and the 
    real *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	real outputSum, lastInput;

	unsigned long SampleTime;
	real outMin, outMax;
	bool inAuto, pOnE;
};
#endif

