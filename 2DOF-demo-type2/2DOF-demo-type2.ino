/*   
 Arduino code for dynamic playseat 2DOF
 Created 24 May 2011 by Jim Lindblom   SparkFun Electronics https://www.sparkfun.com/products/10182 "Example Code"
 Created 24 Apr 2012 by Jean David SEDRUE Version betatest26 - 24042012 http://www.gamoover.net/Forums/index.php?topic=25907
 Updated 20 May 2013 by RacingMat in english http://www.x-sim.de/forum/posting.php?mode=edit&f=37&t=943&p=8481  in french : http://www.gamoover.net/Forums/index.php?topic=27617
 Updated 30 April 2014 by RacingMat (bug for value below 16 corrected)
 */

#define BRAKEVCC 0
#define RV  2 //beware it's depending on your hardware wiring
#define FW  1 //beware it's depending on your hardware wiring
#define STOP 0
#define BRAKEGND 3

////////////////////////////////////////////////////////////////////////////////
#define pwmMax 255 // or less, if you want to lower the maximum motor's speed

// defining the range of potentiometer's rotation
const int potMini=208;
const int potMaxi=815;

////////////////////////////////////////////////////////////////////////////////
#define motLeft 0
#define motRight 1
#define potL A0
#define potR A1

////////////////////////////////////////////////////////////////////////////////
//  DECLARATIONS
////////////////////////////////////////////////////////////////////////////////
/*  VNH2SP30 pin definitions*/
int inApin[2] = {
  7, 4};  // INA: Clockwise input
int inBpin[2] = {
  8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {
  5, 6}; // PWM input
int cspin[2] = {
  2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {
  0, 1}; // EN: Status of switches output (Analog pin)
int statpin = 13;  //not explained by Sparkfun
/* init position value*/
int DataValueL=512; //middle position 0-1024
int DataValueR=512; //middle position 0-1024

////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // serial initialization
  Serial.begin(115200);

  // initialization of Arduino's pins
  pinMode(statpin, OUTPUT); //not explained by Sparkfun
  digitalWrite(statpin, LOW);

  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked for motor
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Main Loop ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  int sensorL,sensorR;

  readSerialData();   // DataValueR & L contain the last order received
  // (if there is no newer received, the last is kept)
  // the previous order will still be used by the PID regulation MotorMotion
  // Function

  sensorR = analogRead(potR);  // range 0-1024
  sensorL = analogRead(potL);  // range 0-1024

  motorMotion(motRight,sensorR,DataValueR);
  motorMotion(motLeft,sensorL,DataValueL);
}
////////////////////////////////////////////////////////////////////////////////
// Procedure: wait for complete trame
////////////////////////////////////////////////////////////////////////////////
void readSerialData()
{
  byte Data[3]={
    '0','0','0'          };
  // keep this function short, because the loop has to be short to keep the control over the motors

  if (Serial.available()>2){
    //parse the buffer : test if the byte is the first of the order "R"
    Data[0]=Serial.read();
    if (Data[0]=='L'){
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      //  call the function that converts the hexa in decimal and that maps the range
      DataValueR=NormalizeData(Data);
    }
    if (Data[0]=='R'){
      Data[1]=Serial.read();
      Data[2]=Serial.read();
      //  call the function that converts the hexa in decimal and maps the range
      DataValueL=NormalizeData(Data);

    }
  }
  if (Serial.available()>16) Serial.flush();
}
////////////////////////////////////////////////////////
void motorMotion(int numMot,int actualPos,int targetPos)
////////////////////////////////////////////////////////
{
  int Tol=20; // no order to move will be sent to the motor if the target
  // is close to the actual position
  // this prevents short jittering moves
  //could be a parameter read from a pot on an analogic pin
  // the highest value, the calmest the simulator would be (less moves)

  int gap;
  int pwm;
  int brakingDistance=30;

  // security concern : targetPos has to be within the mechanically authorized range
  targetPos=constrain(targetPos,potMini+brakingDistance,potMaxi-brakingDistance);

  gap=abs(targetPos-actualPos);

  if (gap<= Tol) {
    motorOff(numMot); //too near to move     
  }
  else {
    // PID : calculates speed according to distance
    pwm=195;
    if (gap>50)   pwm=215;
    if (gap>75)   pwm=235;   
    if (gap>100)  pwm=255;
    pwm=map(pwm, 0, 255, 0, pwmMax);  //adjust the value according to pwmMax for mechanical debugging purpose !

    // if motor is outside from the range, send motor back to the limit !
    // go forward (up)
    if ((actualPos<potMini) || (actualPos<targetPos)) motorGo(numMot, FW, pwm);
    // go reverse (down)   
    if ((actualPos>potMaxi) || (actualPos>targetPos)) motorGo(numMot, RV, pwm);

  }
}



////////////////////////////////////////////////////////////////////////////////
void motorOff(int motor){ //Brake Ground : free wheel actually
  ////////////////////////////////////////////////////////////////////////////////
  digitalWrite(inApin[motor], LOW);
  digitalWrite(inBpin[motor], LOW);
  analogWrite(pwmpin[motor], 0);
}
////////////////////////////////////////////////////////////////////////////////
void motorOffBraked(int motor){ // "brake VCC" : short-circuit inducing electromagnetic brake
  ////////////////////////////////////////////////////////////////////////////////
  digitalWrite(inApin[motor], HIGH);
  digitalWrite(inBpin[motor], HIGH);
  analogWrite(pwmpin[motor], 0);
}

////////////////////////////////////////////////////////////////////////////////
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
////////////////////////////////////////////////////////////////////////////////
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);

    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void motorDrive(uint8_t motor, uint8_t direct, uint8_t pwm)
////////////////////////////////////////////////////////////////////////////////
{
  // more readable function than Jim's (for educational purpose)
  // but 50 octets heavier ->  unused
  if (motor <= 1 && direct <=4)
  {
    switch (direct) {
    case 0: //electromagnetic brake : brake VCC
      digitalWrite(inApin[motor], HIGH);
      digitalWrite(inBpin[motor], HIGH);
      break;
    case 3: //Brake Ground (free wheel)
      digitalWrite(inApin[motor], LOW);
      digitalWrite(inBpin[motor], LOW);
      break;
    case 1: // forward : beware it's depending on your hardware wiring
      digitalWrite(inApin[motor], HIGH);
      digitalWrite(inBpin[motor], LOW);
      break;
    case 2: // Reverse : beware it's depending on your hardware wiring
      digitalWrite(inApin[motor], LOW);
      digitalWrite(inBpin[motor], HIGH);
      break;
    }
    analogWrite(pwmpin[motor], pwm);
  }
}
////////////////////////////////////////////////////////////////////////////////
// testPot
////////////////////////////////////////////////////////////////////////////////
void testPot(){

  Serial.print(analogRead(potL));
  Serial.print(";");
  Serial.println(analogRead(potR));
  delay(250);

}
////////////////////////////////////////////////////////////////////////////////
void testpulse(){
  int pw=120;
  while (true){

    motorGo(motLeft, FW, pw);
    delay(250);       
    motorOff(motLeft);
    delay(250);       
    motorGo(motLeft, RV, pw);
    delay(250);       
    motorOff(motLeft);     

    delay(500);       

    motorGo(motRight, FW, pw);
    delay(250);       
    motorOff(motRight);
    delay(250);       
    motorGo(motRight, RV, pw);
    delay(250);       
    motorOff(motRight);     
    Serial.println("testpulse pwm:80");     
    delay(500);

  }
}
////////////////////////////////////////////////////////////////////////////////
// Function: convert Hex to Dec
////////////////////////////////////////////////////////////////////////////////
int NormalizeData(byte x[3])
////////////////////////////////////////////////////////////////////////////////
{
  int result;

  if ((x[2]==13) || (x[2]=='R') || (x[2]=='L'))  //only a LSB and Carrier Return or 'L' or 'R' in case of value below 16 (ie one CHAR and not 2)
  {
    x[2]=x[1];  //move MSB to LSB
    x[1]='0';     //clear MSB
  }
  for (int i=1; i<3; i++)
   {
    if (x[i]>47 && x[i]<58 ){//for x0 to x9
      x[i]=x[i]-48;
    }                       
      if (x[i]>64 && x[i]<71 ){//for xA to xF
      x[i]=(x[i]-65)+10;       
    }
  }
  // map the range from Xsim (0 <-> 255) to the mechanically authorized range (potMini <-> potMaxi)
  result=map((x[1]*16+x[2]),0,255,potMini,potMaxi);
  return result;
}
 
