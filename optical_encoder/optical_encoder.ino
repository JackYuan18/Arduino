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
  
 
 if (L != 0 && R !=0)
 {
  // while (serial_read != "$");
   print_speeds();
   L = 0;
   R = 0;
   
 }

}


 void magnet_detect_R()//This function is called whenever a magnet/interrupt is detected by the arduino
 { 
   timeR[0] = timeR[1];
   timeR[1] = millis();
   speeR = 0.212/6*1000/(timeR[1] - timeR[0]);
   R = 1;
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
   speeL = 0.212/6*1000/(timeL[1] - timeL[0]);
   L = 1;
   
 }
 
void input_speed()
{
  while(Serial.available()) {
      delay(5);
      if(Serial.available() > 0) {
         serial_read = Serial.readStringUntil('#');
         //Serial.println(serial_read);
       // serial_read += c;
      }
    }
  if(serial_read.length() > 0 ) {
    
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
    thr.write(rev);
    
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
     //Serial.print("\t");
    // Serial.print(spee);
     Serial.print("\n");
}
  
