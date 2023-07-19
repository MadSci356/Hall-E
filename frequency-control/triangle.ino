/**
   Functions defined to approximate triangular wave step delays
   for stepper motor.

   Calculating step delays using a triangle wave approximation.
   To first order correction, it's v(t) = A*w (cos(wt + o)) - cos(3wt)/3)
   We wan't delay for ith step. 0 <= i < STEPS
   Since v(t) = dy/dt; we fix dy = distance per step (call it dist).
   so dt = dist / v(t).
   where t is 0 <= t < Period/T * time_step where time_step is 1/(f*2*STEPS).
   Written by Sayam Patel (sdpate11@ncsu.edu) (12/10/2018)
*/


// constants caluculated form constants
//double omega = TWO_PI * freq;
//float periodOffset = omega * OFFSET_TIME / freq;
//float time_step = 1 / (2 * freq * STEPS);
//float Amp = A * omega;

//variables
/** time var for calculating v(t)*/
float t = 0;
/** list of step delays. delays[i] will be delay on the ith step of movement */
//float delays[STEPS];
boolean printOut = 0;

void calculate_delays() {
  omega = TWO_PI * freq;
  periodOffset = omega * OFFSET_TIME / freq;
  time_step = 1 / (2 * freq * STEPS);
  Amp = A * omega;
  if (printOut) {
    Serial.print("Omega: ");
    Serial.println(omega, 6);
    Serial.print("Period offset: ");
    Serial.println(periodOffset, 6);
    Serial.print("Amp: ");
    Serial.println(Amp, 6);
    Serial.print("time_step: ");
    Serial.println(time_step, 6);
  }
  //calculate delay of a step.
  //calculating for times from from 1*time_step to (STEPS-1)*time_step, that is
  //the list elements indices are from 0 to STEPS-2 -> STEPS-1 total elements

  for (int i = 0; i < STEPS - 1; i++) {
    triStepDelays[i] = delay_step(i + 1);
    if (printOut && i % 1 == 0) { //change to true for step delay values
      Serial.print("Step: ");
      Serial.print(i + 1);
      Serial.print(", ");
      Serial.print(t, PRECISION);
      Serial.print(" s, ");
      Serial.print("Vel: ");
      Serial.print(velocity(t), PRECISION);
      Serial.print(", ");
      Serial.print(triStepDelays[i]);
      Serial.println(" milliseconds.");
    }
  }
      //summing cycle time (crude estimate)
      float sum = 0;
      startProcessTime();
      sum += 2.0 * TRI_DIR_DELAY / 1000.0;  //2x tridelay in milli seconds
      for (int i = 0; i < STEPS-1; i++) {
          sum += 4.0 * triStepDelays[i]; //2x for 2 delays/pulse, 2x for 2nd half of cycle
      }
      sum += endProcessTime();
      printTimeStats(sum, "Calculated for variable delays (triangle approximation):");


}
/**
   Trangle wave approximation to first order
   v(t) = A*w ((cos(wt + o)) - cos(3wt)/3))
*/
float velocity (float time_t) {
  float rad = (omega * time_t) + periodOffset;
  float rad1 = 3 * rad;
  float value = Amp * (cos(rad) - (cos(rad1) / 3 ));
  return value;
}

/**
   Calculates delay of nth step.
   delay = (in milliseconds) = 1000 * (dist_step) * vel(t))
   where t = n * time_step;
*/
int delay_step (int s) {
  t = s  * time_step;
  //Serial.println(t,8);
  float vel = velocity(t);
  float  delayt = floor((DIST_PER_STEP / vel) * 1000);
  int delayint = delayt/2;
  return delayint;
}





/**
   Ossicllate moves the motor back and forth by pulsing HIGH and LOW
   states to the stepPin. Direction is set by dirPin.
   ***** Uses Triangle wave approximation to first order*****

   How it works:
   Runs for n cycles.
    each Cycle: right y steps + dir switch + left y steps
      each Step: y step pulses.
        each Pulse: Step HIGH + delay + Step LOW + delay.
*/
void tri_ossicllate (int triStepDelays[]) {
  boolean cycleTimed = false;
  startProcessTime();
  for (int i = 0; i < cycles; i++) { //total cycles to perform  
    //clockwise (looking at the front of the motor)
    digitalWrite(dirPin, HIGH);

    //currTime = millis(); //timing start
    for (int y = 0; y < STEPS - 1; y++) {
      digitalWrite(stepPin, HIGH);
      delay(triStepDelays[y]);
      digitalWrite(stepPin, LOW);
      delay(triStepDelays[y]);
    }

    //delay for switching directions
    delayMicroseconds(TRI_DIR_DELAY);

    //counter-clockwise (looking at the front of the motor)
    digitalWrite(dirPin, LOW);

    for (int y = 0; y < STEPS - 1; y++) {
      digitalWrite(stepPin, HIGH);
      delay(triStepDelays[y]);
      digitalWrite(stepPin, LOW);
      delay(triStepDelays[y]);
    }

    //delay for switching directions
    delayMicroseconds(TRI_DIR_DELAY);

    //printing executed cycle times
    if (!cycleTimed) {
      printTimeStats(endProcessTime(), "Time from running approx tri osscillate:");
      cycleTimed = true;
    }
    
  }
}
/** Prints time stats given the "actual" processing in ms
 *  processTime: time in milliseconds of your process
 *  Output: None. Prints to Serial time stats based on the global
 *  frequency value.
 */
void printTimeStats(float processTime, String msg) {
    float cycleTime = processTime/MILLI_IN_SECONDS; //now in seconds
    float motionFreq = 1 / cycleTime; //"actual freq"
    float percentDiffFrequency = 100.0 * abs(freq - motionFreq)/freq;
    
    Serial.println();
    Serial.println(msg);  
    Serial.print("for 1 cycle: ");
    Serial.print(cycleTime, PRECISION);
    Serial.print(" s, ");
    Serial.print(motionFreq, PRECISION);
    Serial.println(" Hz");
    Serial.print("Percent diff: ");
    Serial.print(percentDiffFrequency, PRECISION);
    Serial.println("%\n");
}

void startProcessTime() {
  if (timingStarted) {
    Serial.println("Cannot use startProcessTime() once it is started.");
    Serial.println("Make sure endProcessTime() is excecuted before starting timer again.");
  } else {
    startTime = millis();
    timingStarted = true;  
  }
}
/** 
 *  Returns time in ms since startProcessTime
 */
float endProcessTime() {
  if (timingStarted) {
    endTime = millis();
    timingStarted = false;
    return (float) (endTime - startTime);
  } else {
    Serial.println("Cannot use endProcessTime() before startProcessTime()");
    Serial.println("Make sure startProcessTime() is excecuted before ending timer.");
  }
}
