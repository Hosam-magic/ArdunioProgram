/*  Arduino Leonardo  *
 Arduino code for dynamic playseat 2DOF
 Created 24 May 2011 by Jim Lindblom   SparkFun Electronics https://www.sparkfun.com/products/10182 "Example Code"
 Created 24 Apr 2012 by Jean David SEDRUE Version betatest26 - 24042012 http://www.gamoover.net/Forums/index.php?topic=25907
 Updated 20 May 2013 by RacingMat in english http://www.x-sim.de/forum/posting.php?mode=edit&f=37&t=943&p=8481  in french : http://www.gamoover.net/Forums/index.php?topic=27617
 Updated 30 April 2014 by RacingMat (bug for value below 16 corrected)
 */


////////////////////////////////////////////////////////////////////////////////
//  定義參數
////////////////////////////////////////////////////////////////////////////////
#define BRAKEVCC 0
#define RV  2 //beware it's depending on your hardware wiring
#define FW  1 //beware it's depending on your hardware wiring
#define STOP 0
#define BRAKEGND 3

////////////////////////////////////////////////////////////////////////////////
#define pwmMax 30 // or less, if you want to lower the maximum motor's speed

// defining the range of potentiometer's rotation
const int potMini=208;
const int potMaxi=815;

////////////////////////////////////////////////////////////////////////////////
#define motLeft 0
#define motRight 1
#define potL A0
#define potR A1

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
int sensorL,sensorR;
/*----------------------------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////
// 初始化
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // serial initialization
  SerialUSB.begin(12000000);
  SerialUSB.setTimeout(000.1);

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

  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  pinMode(2, OUTPUT);
}
/*----------------------------------------------------------------------------*/




////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// 主迴圈 ////////////////////////////////////////
void loop()
{
  String S;
  S = "*recive: " + readSerialData();   // DataValueR & L contain the last order received
  // (if there is no newer received, the last is kept)
  // the previous order will still be used by the PID regulation MotorMotion
  // Function

  sensorR = analogRead(potR);  // range 0-1024
  sensorL = analogRead(potL);  // range 0-1024

  //SerialUSB.print("\tDataValueR: ");
  //SerialUSB.print(DataValueR);
  //SerialUSB.print("\tDataValueL: ");
  //SerialUSB.print(DataValueL);

  motorMotion(motRight,sensorR,DataValueR);
  motorMotion(motLeft,sensorL,DataValueL);

  int val = digitalRead(13);
  int val2 = digitalRead(12);
  int val3 = digitalRead(11);
  int val4 = digitalRead(2);

  int val5 = analogRead(A3);
  int val6 = analogRead(A2);
  int val7 = analogRead(A0);
  int val8 = analogRead(A1);
 
  S += "\tDataValueR: " + String(DataValueR) + "\tDataValueL: " + String(DataValueL) + "\tM1-1: " + String(val2) + " M1-2: " + String(val3) +" M2-1: " + String(val4) + " M2-2: " + String(val) + " M1-PWM: " + String(val5) + "\tM2-PWM: " + String(val6) + "\tM1-pos: " + String(val7) + "\tM2-pos: " + String(val8);
  SerialUSB.println(S);
}
/*----------------------------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////
// Procedure: wait for complete trame
////////////////////////////////////////////////////////////////////////////////
String readSerialData()
{
  String recive = "";
  byte Data[5]={
    '0','0','0','0','0'          };
  // keep this function short, because the loop has to be short to keep the control over the motors   

  if (SerialUSB.available()){
    recive = SerialUSB.readString();    
    if(recive == "RESET\r\n")
    {
      digitalWrite(2, LOW);
    }
    else 
    {
      for(int i = 0; i < recive.length() - 2; i++)
      {
        Data[0]=recive[i];
        if (Data[0]=='L')
        {
          Data[1]=recive[i+1];
          Data[2]=recive[i+2];
          Data[3]=recive[i+3];
          if (Data[1] == 45) Data[4]=recive[i+4];
          //  call the function that converts the hexa in decimal and that maps the range
          DataValueR=NormalizeData(Data);
        } 
        else if (Data[0]=='R')
        {
          Data[1]=recive[i+1];
          Data[2]=recive[i+2];
          Data[3]=recive[i+3];
          if (Data[1] == 45) Data[4]=recive[i+4];
          //  call the function that converts the hexa in decimal and maps the range
          DataValueL=NormalizeData(Data);
        }
      }
    }
  }
  if (SerialUSB.available()>16) SerialUSB.flush();  
  return recive;
}
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////
void motorMotion(int numMot,int actualPos,int targetPos)
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
    pwm=70;
    if (gap>50)   pwm=127;
    if (gap>75)   pwm=180;   
    if (gap>100)  pwm=255;
    pwm=map(pwm, 0, 30, 0, pwmMax);  //adjust the value according to pwmMax for mechanical debugging purpose !

    // if motor is outside from the range, send motor back to the limit !
    // go forward (up)
    if ((actualPos<potMini) || (actualPos<targetPos)) motorGo(numMot, FW, pwm);
    // go reverse (down)   
    if ((actualPos>potMaxi) || (actualPos>targetPos)) motorGo(numMot, RV, pwm);

  }
}
//----------------------------------------------------//


////////////////////////////////////////////////////////////////////////////////
// Brake Ground : free wheel actually  
////////////////////////////////////////////////////////////////////////////////
void motorOff(int motor)
{ 
  digitalWrite(inApin[motor], LOW);
  digitalWrite(inBpin[motor], LOW);
  analogWrite(pwmpin[motor], 0);
}
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
// "brake VCC" : short-circuit inducing electromagnetic brake
////////////////////////////////////////////////////////////////////////////////
void motorOffBraked(int motor) 
{  
  digitalWrite(inApin[motor], HIGH);
  digitalWrite(inBpin[motor], HIGH);
  analogWrite(pwmpin[motor], 0);
}
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
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
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////
void motorDrive(uint8_t motor, uint8_t direct, uint8_t pwm)
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
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
// Function: convert Hex to Dec
////////////////////////////////////////////////////////////////////////////////
int NormalizeData(byte x[5])
{
  int result;
  int sign = 4;
  int a = sign - 3;
  int m = 0;

  if (x[1] == 45)
  {
    sign = 5;
  }

  
  for (a = sign - 3; a < sign; a++) //MLSB
  {
    if ((x[a]==10) || (x[a]==32) || (x[a]==13) || (x[a]=='R') || (x[a]=='L'))
    {
      if (a == (sign - 3))
      {
        x[a] = '0';
        x[a+1] = '0';
        x[a+2] = '0';
      } else
      {
        for(int b = 0; b < a - (sign - 3); b++)
        {
          int c = a - b;
          x[c]=x[c-1];  //move MSB to LSB
          x[c-1]='0';                  
        }        
      }
    }
  }
  
  for (a = sign - 3; a < sign; a++)
  {
    if (x[a]>47 && x[a]<58 )//for x0 to x9
    {
      x[a]=x[a]-48;
    }
  }
  a = sign - 3;
  m = (x[a]*10*10+x[a+1]*10+x[a+2]);
  if(m > 100)  m = 100;
  result=map(m,0,100,((potMaxi+potMini)/2),potMaxi);
  if(sign == 5) result = ((potMaxi+potMini)/2) - (result-((potMaxi+potMini)/2)-1);
  return result;
}
//----------------------------------------------------------------------------//
