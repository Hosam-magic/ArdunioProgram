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
const float potMini=-85; //208
const float potMaxi=85; //815

////////////////////////////////////////////////////////////////////////////////
//左:0 右:1
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
float TargetHightL=0; //中間位置0 目標高度 -R~R
float TargetHightR=0; //中間位置0 目標高度 -R~R
int sensorL,sensorR;

int pwm = 0; //PWM
float R = 10; //轉動連桿半徑(CM)
float RmaxD = 300; //電阻極限角度
float gap = 0;  //容許定位公差
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
  String S; //實驗性檢測******
  S = "*recive: " +  //實驗性檢測******
  readSerialData(); //讀取&解析USB通訊 

  // DataValueR & L contain the last order received
  // (if there is no newer received, the last is kept)
  // the previous order will still be used by the PID regulation MotorMotion
  // Function

  sensorR = analogRead(potR);  // range 0-1024
  sensorL = analogRead(potL);  // range 0-1024

  motorMotion(motLeft, ResisToDegree(sensorL) - 150, compensator_Target(TargetHightL));
  int pwmL = pwm; //實驗性檢測******
  motorMotion(motRight, ResisToDegree(sensorR) - 150, compensator_Target(TargetHightR));
  int pwmR = pwm; //實驗性檢測******
  

//實驗性檢測******
  int val = digitalRead(13);
  int val2 = digitalRead(12);
  int val3 = digitalRead(11);
  int val4 = digitalRead(2);

  int val5 = analogRead(A3);
  int val6 = analogRead(A2);
  int val7 = analogRead(A0);
  int val8 = analogRead(A1);

  float degreeR = ResisToDegree(sensorR) - 150;  
  float degreeL = ResisToDegree(sensorL) - 150;
  float targetR = compensator_Target(TargetHightR);
  float targetL = compensator_Target(TargetHightL);

 
  S +=  "\t高度(左): " + String(TargetHightL) +
        "\t角度(左): " + String(targetL) + "°" + 
        "\t感測器(左): " + String(degreeL) + "°" + 
        "\t gap: " + String(gap) + "°" + 
        "\t馬達(左)-A: " + String(val) + 
        "\t馬達(左)-B: " + String(val2) + 
        "\t馬達(左)-PWM: " + String(pwmL); 
  SerialUSB.println(S);
//
}
/*----------------------------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////
// 分析USB指令碼
////////////////////////////////////////////////////////////////////////////////
String readSerialData()
{
  String recive = "";
  String RightValue = "";
  String LeftValue = "";
  
  if (SerialUSB.available())
  {
    recive = SerialUSB.readString();    
    if(recive == "RESET\r\n")
    {
      digitalWrite(2, LOW);
    }
    else 
    {
      if(recive.indexOf("Right") != -1) //有沒有Right參數
      {
        int Rword = recive.indexOf("Right");
        if(recive[Rword+5] == '=') //檢查關鍵字後面是否有 '='
        {
          RightValue = recive.substring(Rword+6, recive.indexOf("&", Rword));

          if(StrIsNumber(RightValue))
          {
            TargetHightR=RightValue.toFloat(); //改寫目標高度(右)
          }

        }
      }
      
      if(recive.indexOf("Left") != -1) //有沒有Left參數
      {
        int Lword = recive.indexOf("Left");
        if(recive[Lword+4] == '=') //檢查關鍵字後面是否有 '='
        {
          LeftValue = recive.substring(Lword+5, recive.indexOf("&", Lword));
          
          if(StrIsNumber(LeftValue))
          {
            TargetHightL=LeftValue.toFloat(); //改寫目標高度(左)
          }    

        }
      }   
    } 
  }  
  if (SerialUSB.available()>16) SerialUSB.flush(); //清USB暫存
  return recive;
}
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////
// 確定馬達方向&速度
// >>已轉為角度計算!
////////////////////////////////////////////////////////
void motorMotion(int numMot,float actualPos,float targetPos)
{
  float Tol = 6; 
  //float gap;  
  float brakingDistance = 8;

  // 分析目標位置是否超出極限，若超出則限制
  targetPos = constrain(targetPos, potMini + brakingDistance, potMaxi - brakingDistance);

  if (numMot == 0) gap=abs(targetPos-actualPos);

  if (gap<= Tol) {
    motorOff(numMot); //too near to move
    pwm = 0; 
  }
  else {
    // PID : calculates speed according to distance
    pwm=20;
    if (gap>15)   pwm=127;
    if (gap>22)   pwm=180;   
    if (gap>30)   pwm=255;
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
// 啟動馬達
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
/*
float NormalizeData(byte x[5])
{
  float result;
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
*/
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
// 2DOF圓周補償(高度轉角度)
////////////////////////////////////////////////////////////////////////////////
float compensator_Target(float target)
{
  float ans = 0;
  //target -= R; //判定中間值為零點
  if (R >= abs(target)) ans = asin(target / R) * 180 / PI;

  return ans;
}
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
// 電阻訊號轉電阻角度
////////////////////////////////////////////////////////////////////////////////
float ResisToDegree (int Vol)
{
  float ans = 0;  
  ans = float(Vol) * RmaxD / 1024;
  return ans;
}

////////////////////////////////////////////////////////////////////////////////
// 電阻角度轉電阻訊號
////////////////////////////////////////////////////////////////////////////////
int DegreeToResis (float Deg)
{
  float ans = 0;  
  ans = float(Deg) * 1024 / RmaxD;
  return int(ans);
}

////////////////////////////////////////////////////////////////////////////////
// 檢查字串是否為數字(含小數可)
////////////////////////////////////////////////////////////////////////////////
boolean StrIsNumber(String str) 
{
  if(str.length()) //內容判斷
  {
    int dot = 0;
    int nev = 0;

    for(char i = 0; i < str.length(); i++) //掃描全部字元
    {      
      if ( !(isDigit(str.charAt(i)) || str.charAt(i) == '.' || str.charAt(i) == '-' )) //判定是否為數字
      {
        return false;
      }

      if(str.charAt(i) == '.') //檢查是否重複小數點
      {
        dot++;
        if(dot > 1) return false;
      }

      if(str.charAt(i) == '-') //檢查是否重複小數點
      {
        nev++;
        if(nev > 1) return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}