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
const float potMini=-75; //208
const float potMaxi=75; //815

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

int pwm = 0; //PWM訊號值0~255
int startMotorVoltage = 50;//馬達啟動電壓(PWM)
float R = 10; //轉動連桿半徑(CM)
float RmaxD = 300; //電阻極限角度
float gotoTargetGap = 0;  //容許定位公差
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
        "\t gap: " + String(gotoTargetGap) + "°" + 
        "\t馬達(左)-A: " + String(val) + 
        "\t馬達(左)-B: " + String(val2) + 
        "\t馬達(左)-PWM: " + String(pwmL);
  SerialUSB.println(S);
//

  motorMotion(motRight, ResisToDegree(sensorR) - 150, compensator_Target(TargetHightR));
  int pwmR = pwm; //實驗性檢測******
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
  float Tol = 4;
  float percentVelocity = 0; //速度百分比
  float brakingDistance = 5;

  // 分析目標位置是否超出極限，若超出則限制
  targetPos = constrain(targetPos, potMini + brakingDistance, potMaxi - brakingDistance);

  actualPos = constrain(actualPos, potMini, potMaxi);  //限制角度值
  targetPos = constrain(targetPos, potMini, potMaxi);  //限制角度值

  gotoTargetGap = abs(targetPos-actualPos);   //計算距離值

  if (gotoTargetGap<= Tol) {
    motorOff(numMot); //太近，必須停下
    percentVelocity = 0;
    pwm = 0;
  }
  else {
    ///////////////
    // PID加減速
    ///////////////
    percentVelocity=1;
    if (gotoTargetGap>6)   percentVelocity=25;
    if (gotoTargetGap>8)   percentVelocity=50;
    if (gotoTargetGap>9)   percentVelocity=75;
    if (gotoTargetGap>10)   percentVelocity=100;

    ///////////////
    //速度補償
    ///////////////
    if (actualPos != 0) 
      percentVelocity = percentVelocity / abs(1/sin((actualPos*PI/180))); 
    else  //0度時不為0
      percentVelocity = percentVelocity / 100;

    ///////////////
    //限制PWM輸入值
    ///////////////
    pwm = map((percentVelocity*1000), 0, 100000, startMotorVoltage-1, 263);  //pwm比例縮放補足馬達啟動電壓 & 去除75度後之值(263是由試誤法得出)

    if(pwm != 0)
      pwm = constrain(pwm, startMotorVoltage-1, 255);  
    else
      pwm = constrain(pwm, 0, 255);  //限制PWM輸入值

    ///////////////
    //馬達執行動作
    ///////////////
    //馬達超出極限範圍，將會反向調控!
    // 正轉(上)
    if ((actualPos<potMini) || (actualPos<targetPos)) motorGo(numMot, FW, pwm);
    // 反轉(下) 
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
// 控制馬達IO
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