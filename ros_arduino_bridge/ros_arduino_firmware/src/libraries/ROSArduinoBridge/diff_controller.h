/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 2;
int Kd = 0;
int Ki = 0;
int Ko = 50;

int left_Kp=3;
int left_Kd=43;
int left_Ki=1;
int left_Ko=50;

int right_Kp=3;
int right_Kd=38;
int right_Ki=0;
int right_Ko=50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
//void doPID(SetPointInfo * p) {
//  long Perror;
//  long output;
//  int input;
//
//  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
//  input = p->Encoder - p->PrevEnc;
//  Perror = p->TargetTicksPerFrame - input;
//  /*
//  * Avoid derivative kick and allow tuning changes,
//  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
//  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
//  */
//  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
//  // p->PrevErr = Perror;
//  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
//  p->PrevEnc = p->Encoder;
//
//  output += p->output;
//  // Accumulate Integral error *or* Limit output.
//  // Stop accumulating when output saturates
//  if (output >= MAX_PWM)
//    output = MAX_PWM;
//  else if (output <= -MAX_PWM)
//    output = -MAX_PWM;
//  else
//  /*
//  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
//  */
//    p->ITerm += Ki * Perror;
//
//  p->output = output;
//  p->PrevInput = input;
//}

/* PID routine to compute the next motor commands */
void dorightPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;
  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

      //Serial.println(input);

  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (right_Kp * Perror - right_Kd * (input - p->PrevInput) + p->ITerm) / right_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* PID routine to compute the next motor commands --------------------------------------------------------------*/
void doleftPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror =p->TargetTicksPerFrame - input;
      //Serial.println(input);

  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (left_Kp * Perror - left_Kd * (input - p->PrevInput) + p->ITerm) / left_Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;
  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine -----------------------------------------------------------*/
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  dorightPID(&rightPID);//执行右马达PID
  doleftPID(&leftPID);//执行左马达PID

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
