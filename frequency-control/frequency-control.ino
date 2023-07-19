
/*
      Frequency Control

      Controlling and driving stepper motor to create magnetic field
      ossicllation as induced by magnet array assembly.

      Original code written
      by Dejan Nedelkovski, www.HowToMechatronics.com
      
      Edited and modified by Sayam Patel (sdpate11@ncsu.edu)
      12/1/2018
*/

//#include <avr/sleep.h>

//Electronics
/** defines pins numbers */
const int stepPin = 3;
const int dirPin = 4;

//Sanity constants
/** error code */
const int ERROR_VAL = -1;
/** start assuming not all things are fine */
boolean everythingIsFine = false;
/** 3 decimal digits shows on all float number prints */
const int PRECISION = 3;
/** micro seconds in second */
const unsigned long MICRO_IN_SECONDS = 1000000;
/** milli seconds in second */
const unsigned long MILLI_IN_SECONDS = 1000;

//Motion constants (do not edit)
/** min freq (Hz) */
const float MIN_FREQ = 0.5;
/** max freq (Hz) */
const float MAX_FREQ = 3.0;
/** min time (seconds) */
const int MIN_TIME = 1;
/** max time (s)*/
const int MAX_TIME = 5400; //1.5 hours
/** minimum cycles */
const int MIN_CYCLES = 1;
/** Traversal in one direction (FULL steps)*/
int const STEPS = 92; // 92 for peak-to-peak  
/** max step delay (~ delayMicroseconds() max value) */
unsigned long MAX_DELAY = 16000;
/** direction switch delay (micro seconds) */
const unsigned int DIR_DELAY = 10000;
/** direction switch delay (micro seconds) */
const unsigned int TRI_DIR_DELAY = 0; //cannot be over 16000
/** constant for freq error within 2%*/ 
const float ERR_CONST = (1 - 1/1.02);

//time variables (do not edit)
/** Operation time (seconds) */
int opTime = MIN_TIME;
/** number of cycles*/
int cycles = MIN_CYCLES;
/** frequency (Hz) */
float freq = -1.0;
/** max allowed error at a given frequency (seconds)*/
float allowedError = ERR_CONST / freq;

/** step delay  variable default */
unsigned long step_delay = ((MICRO_IN_SECONDS / MIN_FREQ) - DIR_DELAY) / (4 * STEPS);
/** list of step delays. delays[i] will be delay on the ith step of movement */
int triStepDelays[STEPS-1];
/** uses triangular approximation if true */
boolean useApprox = false;
/** start time var for timing */
unsigned long startTime = 0;
/** start time var for timing */
unsigned long endTime = 0;
/** boolean for checking time */
boolean timingStarted = false;

//triangle step_delay calculation constants
/** amplitude for wave (won't be exact bc of series approx)*/
double const A = 12.7;
/** distance per step. based on 20tooth GT2-3M timing pulley */
float const DIST_PER_STEP = .3003; //mm
/** offset parameter so that block doesn't ossiccllate past starting pos*/
float const OFFSET_DIST = 10; //mm, won't need it
/** starting at a magnetic peak and moving right */
float const OFFSET_TIME = .75; //s

// constants calculated from constants
double omega = TWO_PI * freq;
float periodOffset = omega * OFFSET_TIME / freq;
float time_step = 1 / (2 * freq * STEPS);
float Amp = A * omega;    

void setup() {
    // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600);
} //setup

void loop() {
  startInput:
  everythingIsFine = false;
  //getting user input
  float uIFreq = 0;
  int uIOpTime = 0;
  //String useApproxInput = "";
  boolean allValid = false;
  Serial.println();
  Serial.println("Enter frequency(Hz), operation time(s)");
  Serial.println("Example: 1.0 60 for 1 Hz for 60s");
  while (!everythingIsFine) {
    if (Serial.available() > 1){
      uIFreq = Serial.parseFloat();
      uIOpTime = Serial.parseInt();
      //useApproxInput = Serial.readString();
      //checking input
//      if (
//      if (useApproxInput == "y") {
//        useApprox = true;
//      } else if (useApprox == "no") {
//        useApprox = false;
//      } 
//      allValid = setCycles(uIOpTime, uIFreq) && setDelay(uIFreq)
      
      if (setCycles(uIOpTime, uIFreq) && setDelay(uIFreq)) {
        everythingIsFine = true;
        //recalulating parameters
        allowedError = ERR_CONST / freq;
        //triangle constants
        if (useApprox) {
          calculate_delays();   
        } else {
          setDelay(freq);
        }
      } else {
        Serial.println();
        Serial.println("Input invalid.");
        Serial.println("Enter frequency (Hz) and operation time. (ex: 1.0 60)");
      }
      }
    }
  //running the motor if all is ok
  if (everythingIsFine) {
      int input = 0;
      boolean started = false;
      boolean startAgain = false;
      Serial.println("Type 1 to begin osscillation, 2 to start again.");
      while (!started) {
        if (Serial.available() > 1){
          input = Serial.parseInt();
          if (input == 1) {
            started = true;
            input = 0;
          } else if (input == 2) {
            input = 0;
            startAgain = true;
            break;
          } else {
            Serial.println("Input invalid. Enter 1 to begin ossicllation.");
          }
        } 
      }
      if (!startAgain){
        if (useApprox){
          Serial.println("Variable step ossicllation starting.");
          tri_ossicllate(triStepDelays);
        } else {
          Serial.println("Oscillating using constant steps starting.");
          ossicllate(); //for a sharp triangle      
        }
        Serial.println("Ossicllation complete. Ending program");  
      } else {
        goto startInput; //will start the loop again?  
      }
      
      
  } else {
    Serial.println("Ending program.");
    
  }
  
} //loop

/**
    Calculates and checks input variables for number of cycles.
    Sets the cycles value if inputs are valid.
    freq: Frequency of the ossicllation in Hz
    opTime: Time of operation for the magnet assembly (in seconds)
    returns: true if valid inputs, false if invalid
*/
boolean setCycles (int inputOpTime, float inputFreq) {
  if (inputFreq < MIN_FREQ || inputFreq > MAX_FREQ) {
    Serial.println("Freqency is out of range.");
    return false;
  }
  if (inputOpTime < MIN_TIME || inputOpTime > MAX_TIME) {
    Serial.println("Operation time is out of range.");
    return false;
  }
  freq = inputFreq; //setting the global freq
  opTime = inputOpTime; //setting the global opTime
  cycles = freq * opTime; 
  if (cycles < MIN_CYCLES) {
    Serial.println("Less than one cycle calculated. Check frequency and operation time values.");
    Serial.println("Program will abort.");
    return false;
  }
  float minutes = opTime / 60.0;
  Serial.println("Magnet Block will run for: ");
  Serial.print(cycles);
  Serial.print(" cycles at ");
  Serial.print(freq);
  Serial.print(" Hz for ");
  Serial.print(opTime);
  Serial.print(" seconds ");
  Serial.print("("); 
  Serial.print(minutes);
  Serial.println(" min). ");  
  return true;

}

/**
    Sets step_delay in microseconds for the stepper motor steps.
    This is the amount of time that needs to be beween a step pulse
    for moving in a direction. See ossicllate for more details.
    Example: at 1 Hz, move right and left in 1 sec => .5 sec for right
    then .5 sec for left. So time for movement is 1/2f. It takes x steps
    to move from peak to peak. So the pulse will have occur 100 times in
    1/2f seconds. So each pulse has to be delayed by 1/(2f*x). Each pulse
    has two delays so our final delay is 1/4fx. (see ossicllate for stepper
    motor mechanism.
    Note: DIR_DELAY is subracted from 1/f because there is an additional
    delay to be made when switching directions. It happens once per cycle.

    freq: frequency desired (NOT CHECKED HERE, CHECKED IN get_cycles()
    returns: true if set corrent, false if invalid inputs
*/
boolean setDelay (float freq) {
  unsigned long period = (MICRO_IN_SECONDS / freq) - (2*DIR_DELAY);
  step_delay = period / (4 * STEPS);
  if (step_delay > MAX_DELAY) {
    Serial.println("Delay outside valid range. Check frequency input.");
    Serial.println("Program will abort.");
    step_delay = ERROR_VAL;
    return false;
  }
  return true;
}

/**
   Ossicllate moves the motor back and forth by pulsing HIGH and LOW
   states to the stepPin. Direction is set by dirPin.
   How it works:
   Runs for n cycles.
    each Cycle: right y steps + dir switch + left y steps
      each Step: y step pulses.
        each Pulse: Step HIGH + delay + Step LOW + delay.
*/
void ossicllate () {
  //boolean timed = false;
  for (int i = 0; i < cycles; i++) { //total cycles to perform
    //clockwise (looking at the front of the motor)
    digitalWrite(dirPin, HIGH);
    
    //currTime = millis(); //timing start
    for (int y = 0; y < STEPS; y++) {
    
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(step_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(step_delay);
    }
    //delay for switching directions
    delayMicroseconds(DIR_DELAY);
    //delay(DIR_DELAY);
    //counter-clockwise (looking at the front of the motor)
    digitalWrite(dirPin, LOW);

    for (int y = 0; y < STEPS; y++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(step_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(step_delay);
    }

    //delay for setting directions
    delayMicroseconds(DIR_DELAY);

  }
}
