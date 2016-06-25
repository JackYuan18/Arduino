//optical sensors speed measuring

#include <Servo.h>

Servo str;
Servo thr;
String serial_read;
String strcmd;
String thrcmd;

//hall effect sensor test


int timeL[2];
int timeR[2];
int timeOld;

 double speeL;
 double speeR;

int L = 0;
int R = 0;
double tempL = 0.00;
double tempR = 0.00;

double previousErr = 0;
int oldPWM;
double currentSp;
int desireSp=0;
double error=0;
int outPWM;


void setup() {
  // put your setup code here, to run once:
  str.attach(7);
  thr.attach(9);
  Serial.begin(115200);
  str.writeMicroseconds(1550);//1340,1760
  thr.writeMicroseconds(1500);
 //Serial.println("RC control test");
  
  //hall effect testing
  attachInterrupt(0, magnet_detect_R, RISING);//pin 2
  attachInterrupt(1,magnet_detect_L,RISING);//Initialize the intterrupt pin (Arduino digital pin 3)
  
  

 timeL[0] = 0;
 timeL[1] = 0;
 timeR[0] = 0;
 timeR[1] = 0;
}

void loop() 
{
  //speed control
  input_speed();
  
  //hall effect testing
  
 pid_controller();
 
 
// if (L != 0 && R !=0)
// {
//  // while (serial_read != "$");
//   print_speeds();
//   L = 0;
//   R = 0;
//   
// }
 
 

}


 void magnet_detect_R()//This function is called whenever a magnet/interrupt is detected by the arduino
 { 
   timeR[0] = timeR[1];
   timeR[1] = millis();
   speeR = 21.2/6*1000/(timeR[1] - timeR[0]);
   R = 1;
   currentSp = 0.5*(speeR + speeL);
//    Serial.print(speeR);
//    Serial.print("\t");
//    Serial.print(timeR[1] - timeR[0]);
//    Serial.print("\n");
;
 }
 
void magnet_detect_L()//This function is called whenever a magnet/interrupt is detected by the arduino
 {

   timeL[0] = timeL[1];
   timeL[1] = millis();
   speeL = 21.2/6*1000/(timeL[1] - timeL[0]);
   L = 1;
   
 }
 
void input_speed()
{
  
      if(Serial.available() > 0) {
         serial_read = Serial.readStringUntil('#');
         //Serial.println(serial_read);
       // serial_read += c;
      }
    
  if(serial_read.length() > 8 ) {
    
   // Serial.println(serial_read);
    thrcmd = serial_read.substring(1,5);
    strcmd = serial_read.substring(5,9);
    
    char carray1[5]; 
    thrcmd.toCharArray(carray1, 5);
    char carray2[5]; 
    strcmd.toCharArray(carray2, 5);
   
//    Serial.println("value received are:");
//    Serial.println(carray1);
//    Serial.println(carray2);
    
    int deg = atoi(carray2);
    int rev = atoi(carray1);
    
//    Serial.println(deg);
//    Serial.println(rev);
    
    str.write(deg);
    desireSp = rev;
    currentSp = 0;
   //thr.write(rev);
    
  }
    serial_read = "";
}


void print_speeds()
{
     double spee;
     spee = (speeL +speeR)*0.5;
     Serial.print(speeL);
     Serial.print(",");
     Serial.print(speeR);
     Serial.print("\t");
     Serial.print(currentSp);
     Serial.print("\t");
     Serial.print(outPWM);
     Serial.print("\t");
     Serial.print(error);
     Serial.print("\n");
}

void pid_controller()
{
      /*calibration curve:
      Linear model Poly3:
     f(x) = p1*x^3 + p2*x^2 + p3*x + p4
       where x is normalized by mean 267.2 and std 98.52
Coefficients (with 95% confidence bounds):
       p1 =     -0.1013  (-2.583, 2.381)
       p2 =      -1.911  (-6.647, 2.826)
       p3 =      -17.89  (-24.76, -11.02)
       p4 =        1463  (1461, 1466)

Goodness of fit:
  SSE: 46.62
  R-square: 0.9802
  Adjusted R-square: 0.9717
  RMSE: 2.581
    
           */
      
      int timeChange = 0;
     double derivative = 0;

     double integral;
     double P;
     double I;
     double D;
     
     
     //no load calibration curve
    // double x = (desireSp - 207.6)/87.1;
    // double Uref = -0.03369*pow(x,3) + 0.1804*pow(x,2) + 4.524*x + 19.3;//no load
    
    // nominal load
     double x = (desireSp - 267.2)/98.52;
     double Uref = -0.1013* pow(x,3) -1.911*pow(x,2)-17.89 * x +1463;
    
     /* PID controller gains */
     //no load:
     //P = 0.5;
     //I = 0.00000001;
     //D = 7.355;//7.355;
    // 
     
     
    //nominal load 
    P = -0.0385; 
    I=-0.1;
    D= -15;
    
    if (currentSp != 0){
      if (desireSp == 0){
        outPWM = 1500;
        thr.write(Uref);
      }
      else{
            timeChange = timeR[1] - timeR[0];
  //           if(currentSp != 0){
               /* error */
                 error =  desireSp - currentSp;
                 integral += error * timeChange;
                 derivative = (error - previousErr)/timeChange;
                 outPWM = (P * error) + (D * (error - previousErr))/ timeChange + (I * integral)+1500;
                 outPWM = min(outPWM, 1480);
                 outPWM = max(outPWM, 1400);
                 previousErr = error;
                 thr.write(outPWM);
           }
     }    
     else
         thr.write(Uref);
      //Serial.println(outPWM);
      
       
    
    
    
    
//    if (outPWM != oldPWM){
//      oldPWM = outPWM;
//      thr.write(outPWM);
   //   Serial.println(outPWM);
      //SpeedPWM = outPWM;
//    }
 
 
}

    
