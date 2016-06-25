/*
speed calibration for RC car
speed measured at two rear wheels every 500 ms, in cm/s
input format: abbbbcccc
              a: car mode
              b: speed PWM
              c: steering PWM  (1300 for straight)

*/



//servo parameters
#include<Servo.h>
Servo str;//steering servo

//user input
String serial_read; //read user input
int SpeedPWM; //input PWM for motor
int mode;//car mode: stop to ground; stop to VCC; backward; forward


//speed calculation
double speeL;
double speeR;
//time sliding window
 volatile double timeL[2];
 volatile double timeR[2];
 double currentSp;


//motor parameters
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin = 7;  // INA: Clockwise input
int inBpin = 8; // INB: Counter-clockwise input
int pwmpin = 5; // PWM input
int cspin = 2; // CS: Current sense ANALOG input
int enpin = 0; // EN: Status of switches output (Analog pin)
int statpin = 13;


 //PID parameters
 int desireSp;//in mm/s
 double previousErr;
 double integral;
double derivative;
  int outPWM;
  int oldPWM;
  int tempTL;
  int i; //counter
double error = 0;
double tempSp = 0;
void setup()
{
  Serial.begin(115200);//baud rate for bluetooth control
  str.attach(6); //steering control PWM is output at digital pin 6
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  
    pinMode(inApin, OUTPUT);
    pinMode(inBpin, OUTPUT);
    pinMode(pwmpin, OUTPUT);
    
   //initial mode for the car: braked but has forward potential , straight steering
    SpeedPWM = 0;
    mode = CCW;
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);
    str.writeMicroseconds(1500);
   Serial.println("RC control test");
   Serial.println("mode: forward");
    
    //speed detection  
   attachInterrupt(1, magnet_detect_1, RISING);//pin 3
   attachInterrupt(0,magnet_detect_2,RISING);//Initialize the intterrupt pin (Arduino digital pin 2)
   
   speeL = 0.00; speeR = 0.00;
   tempTL = 0;
   
   
   
    //PID parameters
  desireSp = 0;//input speed
  previousErr = 0;
  outPWM = 0;  
  oldPWM = 0;
  integral = 0.00;
  derivative = 0.00;
  currentSp = 0.00;
  tempTL = 0;
  i = 0;
}

void loop()
{
  //user input for speed PWM, steering PWM, car mode
  input_speed();
  
    
  //speed meausurement
    speed_detect();
       
 //pid controller
  pid_controller(); 
  
    
  
  if ((analogRead(cspin < CS_THRESHOLD)))
  { 
    digitalWrite(statpin, HIGH);
  }
 
 
}

void motorOff(int motor)
{
  // Initialize braked
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);
   analogWrite(pwmpin, 0);
}

void motorGo(int motor, int direct, int pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin, HIGH);
      else
        digitalWrite(inApin, LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin, HIGH);
      else
        digitalWrite(inBpin, LOW);

      analogWrite(pwmpin, pwm);
    }
  }
}

//---------------------------user input steering degree and motor speed------------------------------
void input_speed()//input format: steering[4]; car speed[2];  mode
{
   int rev = 0;

      delay(5);
      if (Serial.available()>0)
      {
      serial_read = Serial.readStringUntil('#');
      Serial.println(serial_read);
      delay(10);
       while( Serial.read() != -1);
      }

   if (serial_read.length()>=7)
  { 
            rev = '$';
       // Serial.write(rev);
       // Serial.print("\n");
        //---------------------mode input--------------------------------
        String ModeStr;
        ModeStr = serial_read.substring(0,1);
        char ModeArray[2];
        ModeStr.toCharArray(ModeArray,2);
        char forward = 'f';
        char backward = 'b';
         
         if (ModeArray[0] == forward) 
        { 
          mode = CCW;//GO FORWARD  
        }
        else if (ModeArray[0] == backward) 
       { 
         mode = CW; //GO BACKWARD
       }
        
         else // 0 for stop to ground; 1 for stop to VCC
        {
            int mode1 = atoi(ModeArray);     
           if (mode1 == 0)
         {
           mode = BRAKEGND;
         }
           else if (mode1 == 1) 
          {
            mode = BRAKEVCC;
            timeL[0] = 0.0;
            timeL[1] = 0.0;
            timeR[0] = 0.0;
            timeR[1] = 0.0;           
          }
        }
        
        //----------------------speed input----------------------------------
        String SpeedStr;
        SpeedStr = serial_read.substring(1,5);
        char SpeedArray[5];
        SpeedStr.toCharArray(SpeedArray,5);
        desireSp = atoi(SpeedArray);
        Serial.println("desireSp = ");    
        Serial.println(desireSp);
        integral = 0;           
        
        //-------------------steering input------------------------------------
        String SteeringStr;
        SteeringStr = serial_read.substring(5,9);
        char SteeringArray[5];
        SteeringStr.toCharArray(SteeringArray,5);
        int deg = atoi(SteeringArray);
        str.writeMicroseconds(deg);      
  }
    serial_read = "";
  
}

//---------------------------speed measurement---------------------------------

void speed_detect()//speed calculation using sliding window, measured in cm/s
{
  double rpsL = 0.0;
  double rpsR = 0.0;
  
  if (timeL[0] )
  {
      rpsL =  1*1000/(timeL[1]-timeL[0]);
      speeL = 38.0*rpsL;
      timeL[0] = 0;
  }
  if (timeR[0])
  {
        rpsR =  1*1000/(timeR[1]-timeR[0]);
      speeR = 38.0*rpsR;      
      timeR[0] = 0;
   }
    
     
     currentSp = (speeL + speeR)/2;
       
}
//**********************************************************
void print_speeds(float spee1, float spee2)
{
    Serial.print(spee1);
    Serial.print("\t");
    Serial.print(",  ");
    Serial.print(spee2);
    Serial.print(",  ");
    Serial.print("\n");   
}

//-------------------------------sliding window speed measurement-------------------------------------------
 void magnet_detect_1()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
       timeL[0] = timeL[1];
       timeL[1] = millis(); 
 }
 
void magnet_detect_2()//This function is called whenever a magnet/interrupt is detected by the arduino
 {
       timeR[0] = timeR[1];
       timeR[1] = millis();
       if (timeR[0] != tempTL)//print 
  {
    Serial.print(outPWM);
    Serial.print("\t");
   
    Serial.print(currentSp);
    Serial.print("\t");
   
    Serial.print(error);
    Serial.print("\n");
    tempTL = timeR[0];
  }
 }
 
 
//----------------------------PID controller-------------------------------------------
 void pid_controller()
{
  /*calibration curve:
  Linear model Poly3:
     f(x) = p1*x^3 + p2*x^2 + p3*x + p4
       where x is normalized by mean 207.6 and std 87.01
Coefficients (with 95% confidence bounds):
       p1 =    -0.03369  (-0.2169, 0.1495)
       p2 =      0.1804  (-0.1752, 0.5361)
       p3 =       4.524  (4.204, 4.844)
       p4 =        19.3  (19.13, 19.48)

Goodness of fit:
  SSE: 0.08964
  R-square: 0.9992
  Adjusted R-square: 0.9989
  RMSE: 0.1132


       */
  int timeChange = 0;
 double derivative = 0;
 error = 0;
 double P;
 double I;
 double D;
 
 //no load calibration curve
// double x = (desireSp - 207.6)/87.1;
// double Uref = -0.03369*pow(x,3) + 0.1804*pow(x,2) + 4.524*x + 19.3;//no load

// nominal load
 double x = desireSp;
 double Uref = 3* pow(10,-6)*pow(x,3) - 0.0004*pow(x,2) + 0.0489* x +32.116;

 /* PID controller gains */
 //no load:
 //P = 0.5;
 //I = 0.00000001;
 //D = 7.355;//7.355;
// 
 
 
//nominal load 
/P = 0.5; 
I=0.0000000;
D= 0;

if(desireSp != 0)
{
  timeChange = timeR[1] - timeR[0];
   if(currentSp != 0)
   {
     /* error */
       error =  desireSp - currentSp;
       integral += error * timeChange;
       derivative = (error - previousErr)/timeChange;
       outPWM =  Uref +(P * error) + (D * (error - previousErr));// timeChange + (I * integral);
       outPWM = min(outPWM, 100);
       outPWM = max(outPWM, 30);//30 for nominal, 0 for no load
       previousErr = error;
   }
   else
   {
//     outPWM = i*0.05;
      outPWM = Uref;
      i++;
   }
   SpeedPWM = outPWM;
   
}
else
{
  outPWM = 0;
}


if (outPWM != oldPWM)
 { 
  oldPWM = outPWM;
   motorGo(0, mode, outPWM);
  //SpeedPWM = outPWM;
 }
 
 
}



