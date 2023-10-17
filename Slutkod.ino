// User-defined variables:
// OBS: men mounting servo during robot assembly, use SensorCenterAngle = 90;


int SensorCenterAngle  = 90;    //Angle when "distance sensor" points straight ahead. OBS: between 80-100 !
int SensorTurningAngle = 80;    //Angle the "distance sensor" turns relative to center default 70
int MiddleTurningAngle = (SensorTurningAngle*0.5);


int AlarmDistanceClose = 25;    // Very close! //defalut 25
int AlarmDistanceFar   = 35;    // Close but no cigar! //default 40 ((distance)/5.8/10))= centimeter
int Insiderange = 40; 
int CheckRouteTime     = 3000;  //How often the Robot should stop and check if any obstacle has arrived from right or left // default 2000


int ForwardWheelSpeed  = 100;   // min-max = 0-255 Choose an even number = possible to divide by 2
int TurnWheelSpeed     = ForwardWheelSpeed-0; // min-max = 0-255 Choose an even number = possible to divide by 2
int BackwardWheelSpeed = ForwardWheelSpeed-0; // min-max = 0-255 Choose an even number = possible to divide by 2
int TurnTime           = 4;     // Time for turning => turning angle of vehicle. Use ~0-10
int turnSpeedCompensator = 0; //If vehicle turns to the left - increase this variable. OBS between ~-30 and 30 to start with...


// -----------------------------------------------------------------------------------
// Here starts the main code:
#include <Servo.h>
int pinLB=6;          // define pin6 as left back connect with IN1
int pinLF=7;          // define pin9 as left forward connect with IN2
int pinRB=8;          // define pin10 as right back connect with IN3
int pinRF=9;          // define pin11 as right back connect with IN4
int pinRSpeed = 3;    //ENA - Right motor - Black cord
int pinLSpeed = 11;   //ENB - Left  motor - Green cord
int inputPin = A0;    // define ultrasonic receive pin (Echo)
int outputPin =A1;    // define ultrasonic send pin(Trig)
int Fspeedd = 0;      // forward distance
int Rspeedd = 0;      // right distance
int Lspeedd = 0;      // left distance
int LMspeedd = 0;     // left middle distance
int RMspeedd = 0;     // right middle distance
int directionn = 0;   //

int lightInput = A5;
int ljusLevel = 1000;
bool ljusOff = false;
int ljuscheck = 0;


#define SERVOS 2      // Define the number of servos
//#define STATES 7    // Define the number of states
Servo myservo[SERVOS];// new myservo
int servo_pins[SERVOS] = {5,10}; // means that two servos are controlled on port 5 and 10
                                 // myservo[0] will be on port 5, and myservo[1] will be on port 10,
                                 // (and so on if more servos will be attached...)
int delay_time = 250; // set stable time
int Fgo = 8;          // forward
int Rgo = 6;          // turn right
int Lgo = 4;          // turn left
int Bgo = 2;          // back
int RMgo = 10;
int LMgo = 12;
unsigned long DriveTime;    // Timevariables to control how often to check for directions
unsigned long DriveTimeOld; // Timevariables to control how often to check for directions
int RightSensorAngle   = SensorCenterAngle-SensorTurningAngle; //Angle the distance sensor turns to the right (90 is straight ahead)
int LeftSensorAngle    = SensorCenterAngle+SensorTurningAngle; //Angle the distance sensor turns to the left  (90 is straight ahead)
int RightmiddleAngle = SensorCenterAngle-MiddleTurningAngle; //Angle the distance sensor turns to the right (90 is straight ahead
int LeftmiddleAngle = SensorCenterAngle+MiddleTurningAngle; //Angle the distance sensor turns to the left  (90 is straight ahead)


void setup()
{
    Serial.begin(9600);
    pinMode(pinLB,OUTPUT);
    pinMode(pinLF,OUTPUT);
    pinMode(pinRB,OUTPUT);
    pinMode(pinRF,OUTPUT);
    pinMode(pinLSpeed,OUTPUT);
    pinMode(pinRSpeed,OUTPUT);
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);
    //myservo[0].attach(5); // define the servo pin(PWM) for ultrasonic sensor
    //myservo[1].attach(10); // define the servo pin(PWM) for ultrasonic sensor
        for(int i = 0; i < SERVOS; i++) {
            myservo[i].attach(servo_pins[i]); // Attach the servo to the servo object
            delay(500);
         }  
    DriveTimeOld = millis(); // initiate start time
    CheckRouteTime         = CheckRouteTime+1000; //Adding one second because it takes roughly one second to check left and right and continue to drive


    //Forward motion directly from the start:
}


void advance(int a) // forward
{
  myservo[1].write(128);
  delay(250);
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);
  digitalWrite(pinLF,HIGH);
  analogWrite(pinLSpeed,ForwardWheelSpeed);
  analogWrite(pinRSpeed,ForwardWheelSpeed-turnSpeedCompensator);
  delay(a * 15);
}


void turnL(int d) //turn right
{
  myservo[1].write(48);
  delay(250);        
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);
  digitalWrite(pinLF,HIGH);
  analogWrite(pinLSpeed,TurnWheelSpeed);
  analogWrite(pinRSpeed,TurnWheelSpeed);
  delay(d * 50);
}


void turnR(int e) //turn left
{
  myservo[1].write(188);
  delay(250);  
  digitalWrite(pinRB,LOW);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);
  digitalWrite(pinLF,HIGH);
  analogWrite(pinLSpeed,TurnWheelSpeed);
  analogWrite(pinRSpeed,TurnWheelSpeed);
  delay(e * 50);
}

void turnLB(int c)
{
  myservo[1].write(188);
  delay(250);        
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);
  analogWrite(pinLSpeed,TurnWheelSpeed);
  analogWrite(pinRSpeed,TurnWheelSpeed);
  delay(c * 50);
}

void turnRB(int z)
{
  myservo[1].write(48);
  delay(250);  
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);
  analogWrite(pinLSpeed,TurnWheelSpeed);
  analogWrite(pinRSpeed,TurnWheelSpeed);
  delay(z * 50);
}

void stopp(int f) //stop
{
  myservo[1].write(128);
  delay(250);      
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
  digitalWrite(pinLSpeed,HIGH);
  digitalWrite(pinRSpeed,HIGH);
  delay(f * 100);
}


void back(int g) //back
{
  myservo[1].write(128);
  delay(250);
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);      
  analogWrite(pinLSpeed,BackwardWheelSpeed);
  analogWrite(pinRSpeed,BackwardWheelSpeed);
  delay(g * 300  /2);
}

void checkLjus(){
  ljusLevel = analogRead(lightInput);
  Serial.print("");
  Serial.print("Light Level (0-1023) = ");
  Serial.println(ljusLevel);
  
  if(ljusLevel < 150){
    if(ljuscheck > 10){
    ljusOff = true;
    Serial.println("Too dark to move");
    }else {
      ljusOff = false;
      ljuscheck++;
    }
    
  }
}

void detection() //test the distance in different directions
{
  int delay_time = 250; // A delay for the detection of obstacles to the sides..
  delay(5);
  DriveTime = millis();
  if (DriveTime > DriveTimeOld + CheckRouteTime){      
    Serial.println("in the detection loop");
    stopp(1);
    ask_pin_L();
    delay(delay_time);
    ask_pin_R();
    delay(delay_time);
    ask_pin_MR();
    delay(delay_time);
    ask_pin_ML();
    delay(delay_time);
    ask_pin_F();
    delay(delay_time);
    NOICP();
    DriveTimeOld = DriveTime;
  }
}




void ask_pin_F() // test forward distance
{
  myservo[0].write(SensorCenterAngle);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float Fdistance = pulseIn(inputPin, HIGH);
  Fdistance= Fdistance/5.8/10;
  Serial.print(" F distance= ");
  Serial.println(Fdistance);
  Fspeedd = Fdistance;
}

void ask_pin_L() // test left distance
{
  myservo[0].write(LeftSensorAngle);
  delay(delay_time);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float Ldistance = pulseIn(inputPin, HIGH);
  Ldistance= Ldistance/5.8/10;
  Serial.print(" L distance= ");
  Serial.println(Ldistance);
  Lspeedd = Ldistance;
}

void ask_pin_R() // test right distance
{
  myservo[0].write(RightSensorAngle);
  delay(delay_time);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float Rdistance = pulseIn(inputPin, HIGH);
  Rdistance= Rdistance/5.8/10;
  Serial.print(" R distance= ");
  Serial.println(Rdistance);
  Rspeedd = Rdistance;
}

void ask_pin_MR() // test right distance
{
  myservo[0].write(RightmiddleAngle);
  delay(delay_time);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float RMdistance = pulseIn(inputPin, HIGH);
  RMdistance= RMdistance/5.8/10;
  Serial.print(" RM distance= ");
  Serial.println(RMdistance);
  RMspeedd = RMdistance;
}

void ask_pin_ML() // test left distance
{
  myservo[0].write(LeftmiddleAngle);
  delay(delay_time);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW);
  float LMdistance = pulseIn(inputPin, HIGH);
  LMdistance= LMdistance/5.8/10;
  Serial.print(" LM distance= ");
  Serial.println(LMdistance);
  LMspeedd = LMdistance;
}

void OA() { // Obstacle Avoidance
  Serial.print("OA - - - - - -");
  if(LMspeedd < AlarmDistanceClose && RMspeedd > AlarmDistanceClose) //if left distance is "Too close"
    {
    turnRB(TurnTime*3);
    Serial.println(" Turning Right ");
    }
  if(RMspeedd < AlarmDistanceClose && LMspeedd > AlarmDistanceClose)//if left distance is "Too close"
    {
    turnLB(TurnTime*3);
    Serial.println(" Turning Left ");
    }
  if (LMspeedd < AlarmDistanceClose && RMspeedd < AlarmDistanceClose)//if left distance and right distance both less than 1
    {
    back(3);
    Serial.println(" Reverse ");
    }
  else
    {
    advance(1); // forward go
    }
}




void NOICP() { // No obstacle in close proximity  
  Serial.println("in NOICP");
  Serial.println(Fspeedd);
  Serial.println(Lspeedd);
  Serial.println(Rspeedd);
  Serial.println(RMspeedd);
  Serial.println(LMspeedd);
  
  if (Fspeedd < Insiderange)
  {
      back(3);
      Serial.println(" Reverse ");
  }
  else if (Fspeedd >= LMspeedd && Fspeedd >= RMspeedd && Fspeedd >= Rspeedd && Fspeedd >= Lspeedd)
      {
        advance(1);
        Serial.println("F");
      }
  else if (Lspeedd > Rspeedd && Lspeedd > Fspeedd && Lspeedd > LMspeedd && Lspeedd > RMspeedd)
      {
          turnL(TurnTime*2);
          Serial.println(" Turning Left NOICP ");
      }
  else if (Rspeedd > Lspeedd && Rspeedd > Fspeedd && Rspeedd > LMspeedd && Rspeedd > RMspeedd)
      {
          turnR(TurnTime*2);
          Serial.println(" Turning Right NOICP");
      }
  else if (LMspeedd > Lspeedd && LMspeedd > Rspeedd && LMspeedd > Fspeedd && LMspeedd > RMspeedd)
      {
          turnL(TurnTime);
          Serial.println(" Turning Left NOICP");
      }
  else if (RMspeedd > Lspeedd && RMspeedd > Rspeedd && RMspeedd > Fspeedd && RMspeedd > LMspeedd)
      {
          turnR(TurnTime);
          Serial.println(" Turning Right NOICP");
      } 
      
  else {
    Serial.println("Inget");
  }
}




void loop()  // Main loop from where the program is run.
{  
checkLjus();

  myservo[0].write(SensorCenterAngle);

    if(ljusOff == true){
      stopp(10);
        
  } 
  else {
    myservo[0].write(SensorCenterAngle);
    detection();
  }
}

