/*********************

LineFollowTimer.ino
Author: Steve Ghertner
Date: 02/15/2014

The hardware is a the adafruit RGBLCD I2C display and uses adafruit RGBLCD library
The select button is pushed to "reset" the timing sequence. (not completely working yet)
A proximity sensor is used to determine if the 
linefollower has passed the start line at which point a timer starts counting
the time and displaying the elapsed time on the LCD.  Once the LF is detected
by the Proximity sensor, then the final elapsed time is displayed on the LCD until the 
select button is pressed to start the sequence over again.

Display currently shows time with the last digit being 1/10's of second.

Revision Date: 02/20/2014
Added code and hardware for plotclock by joo on thingiverse
https://www.thingiverse.com/thing:248009 to linefollower timer to 
plot finished time.  Currently, the colon is used instead of a decimal when
plotting.  

Revision Date: 03/01/2014
initialized elapse_time=0 and  set elapse_time=0 and start_time=0
after plotting finished time to try to fix elapsed_time not being reset
when reset or select button pushed after timing finished.

**********************/

// include the library code:
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define NUM_READINGS  5 //set the moving average window to 5

#define IR_THRESHOLD  175 //Threshold to determine if see robot at start line

#define  SONIC_TRIG_PIN  3  //srf04 sonic trigger pin
#define  SONIC_ECHO_PIN  4  //srf04 sonic echo pin
#define  SONIC_THRESHOLD  5 //# inches to be in zone to sense robot

#define  DEBUG  0    // 0 for test using left PB to simulate sensing robot
#define  SHARP_IR  1 //1 if using sharp ir for sensing robot
#define  SONIC  2 //2 if using ping like sonic sensor to sense robot

//Global Variables
int start_time=0;
boolean timing_started=false;
boolean timing_finished=false;
boolean timing=false;
boolean robot_cleared=true;
long  elapse_time=0; //store elapsed time
int input_select=SONIC;  //select robot sensing input device

//long sum=0;  //variable to hold sum of values
//int a0=0,a1=0,a2=0,a3=0,a4=0; //variables to hold values

//i/o
int IR_pin=A0; //set analog input for Sharp IR sensor to detect robot

//plotter variables
#define SERVOFAKTOR 650

// length of arms
#define L1 35
#define L2 55.1
#define L3 13.2


// origin points of left and right servo 
#define O1X 22
#define O1Y -25
#define O2X 47
#define O2Y -25

// Zero-position of left and right servo
#define SERVOLEFTNULL 1890
#define SERVORIGHTNULL 960

// lift positions of lifting servo
#define LIFT0 1080// on drawing surface
#define LIFT1 925// between numbers
#define LIFT2 725
// going towards sweeper

// speed of liftimg arm, higher is slower
#define LIFTSPEED 1500

// servo width:

// Servo1: Write: 1080ms; Lift Sweep: 724; Lift Number: 924 
// Servo2: 0.65-1.98
// Servo3: 1.1-1.8ms 


int servoLift = 1500;

Servo servo1;  // 
Servo servo2;  // 
Servo servo3;  // 

int val;    // variable to read the value from the analog pin 

volatile double lastX = 75;
volatile double lastY = 47.5;

int last_min = 0;



int ComputeMoveAvg(int value){
  long sum;  //variable to hold sum of values
  static int a0,a1,a2,a3,a4; //variables to hold values
  
  //update the sum used for averaging by subtracting the oldest value
  //and adding in newest value
  //sum-=a0;
  //sum+=value;
  //compute new moving average
  //int avg=int(sum/=NUM_READINGS); //moving average window has 5 values
  
  //shift all the values and store in variables
  a0=a1;
  a1=a2;
  a2=a3;
  a3=a4;
  a4=value;
  
  sum=a0+a1+a2+a3+a4;
  
  //compute new moving average
  int avg=int(sum/NUM_READINGS); //moving average window has 5 values
  
  return avg;  //return new moving average
}

int  ReadSonic(void){
  long duration;
  
  digitalWrite(SONIC_TRIG_PIN, LOW);                   // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(SONIC_TRIG_PIN, HIGH);                  // Send a 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG_PIN, LOW);                   // Send pin low again
  duration = pulseIn(SONIC_ECHO_PIN, HIGH);
  delay(5);
  return int(duration/148);
}

void InitDisplay(void){
  lcd.clear();
  lcd.print("MTRAS Timer");
  //time = millis() - time;
  //Serial.print("Took "); Serial.print(time); Serial.println(" ms");
  lcd.setCursor(0,1);
  lcd.print("Waiting Robot...");
  lcd.setBacklight(WHITE);
}

boolean RobotSeen(void){
  /*Inputs: None
    Outputs: booean if robot detected
    determine if the robot has been detected to start/stop timing
    three inputs can be used to detect robot:
    1. select pb used for debugging
    2. sharp IR sensor- which gave false inputs when sumo robots with
          forward facing IR transmitters were used as line followers
    3. SR04 type ultrasonic sensor
  */
  boolean see_robot;
  uint8_t buttons = lcd.readButtons();
  unsigned int ir_avg, sonic_avg;
  
  //used to debug code to simulate robot seen
  switch(input_select){
  
    case DEBUG:
      if (buttons & BUTTON_LEFT)
        see_robot=true;
       else
         see_robot=false;
       break;
     case SHARP_IR:
      //increment the moving average
      ir_avg=ComputeMoveAvg(analogRead(IR_pin));
      //Serial.print(analogRead(IR_pin));
      //Serial.print("|");
      //Serial.println(ir_avg);
      if (ir_avg>IR_THRESHOLD)
         see_robot=true;
       else
         see_robot=false;
       break; 
     
     case SONIC:
      int value=ReadSonic();
      sonic_avg=ComputeMoveAvg(value);
      //Serial.print(value);
      //Serial.print("|");
      //Serial.println(sonic_avg);
      if (sonic_avg<=SONIC_THRESHOLD)
         see_robot=true;
       else
         see_robot=false;
      break;
  } 
   
   
   return see_robot;

}

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {

  switch (num) {

  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(0);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift(1);
    break;
  case 1:

    drawTo(bx + 3 * scale, by + 15 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(1);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawTo(bx + 1 * scale, by + 0 * scale);
    drawTo(bx + 12 * scale, by + 0 * scale);
    lift(1);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    lift(1);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 6 * scale);
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(1);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawTo(bx + 5 * scale, by + 20 * scale);
    drawTo(bx + 12 * scale, by + 20 * scale);
    lift(1);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    lift(1);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    lift(0);
    drawTo(bx + 12 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 0);
    lift(1);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift(1);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    lift(1);
    break;

  case 111:
    //input 111 commands erase board motion
    //lift(2);
    //drawTo(75, 47.5);
    lift(0);
    //speed = 0;
    drawTo(75, 43);
    drawTo(65, 43);

    drawTo(65, 49);
    drawTo(5, 49);
    drawTo(5, 45);
    drawTo(65, 45);
    drawTo(65, 40);

    drawTo(5, 40);
    drawTo(5, 35);
    drawTo(65, 35);
    drawTo(65, 30);

    drawTo(5, 30);
    drawTo(5, 25);
    drawTo(65, 25);
    drawTo(65, 20);

    drawTo(5, 20);
    drawTo(60, 44);
    drawTo(77, 44);
    drawTo(78, 44);
    lift(2);
    //speed = 3;

    break;

  case 11:
    //command 11 moves pen to print colon
    drawTo(bx + 5 * scale, by + 15 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    drawTo(bx + 5 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    break;

  }
}



void lift(char lift) {
  switch (lift) {
    // OPTIMIEREN  !

  case 0: //850

      if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0) 
      {
        servoLift--;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT0) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }

    }

    break;

  case 1: //150

    if (servoLift >= LIFT1) {
      while (servoLift >= LIFT1) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }
    } 
    else {
      while (servoLift <= LIFT1) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }

    }

    break;

  case 2:

    if (servoLift >= LIFT2) {
      while (servoLift >= LIFT2) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT2) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  //lcd.setCursor(0, 0);
  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTOR) + SERVOLEFTNULL));
  //lcd.print(floor(((a2 + a1 - M_PI) * 750) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.637) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.637) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  //lcd.setCursor(0, 1);
  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTOR) + SERVORIGHTNULL));
  //  lcd.print(floor(((a1 - a2) * 750) + SERVORIGHTNULL));
}

void  PlotTime(int num){
  //plot number
  //input: number to plot--- assume maximum 4 digits
  //output: N/A
  int digit[10]; //init array to hold digits
  int n=num;  //store number to plot in new local variable
  int i=0; //init counter
  //erases
  //lift(0);
  //number(3,3,111,1);
  lift(1);
  //break off each digit and store in digit array
  while(n){
    digit[i++]=n%10; //get least significant digit
    n=int(n/10); //shift integer to right to get next significant digit
  }
  
  //plot the digits at fixed locations for each digit
  if(i>=3) number(5,25,digit[3],0.9);
  if (i>=2) number(19,25,digit[2],0.9);
  
  if (i>=1) number(28,25,digit[1],0.9); //print colon
  if (i>=0) {
      number(34,25,11,0.9);
      number(48,25,digit[0],0.9);
   }
  //***************DEBUG 
  Serial.println(digit[4]);
  Serial.println(digit[3]);
  Serial.println(digit[2]);
  Serial.println(digit[1]);
  Serial.println(digit[0]);
  //move marker back to "home" position
  lift(2);
  drawTo(75,47.5);
  lift(1);
}
void setup() {
  int j;
  // Debugging output
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  
  drawTo(75, 44);
  lift(0);
  servo1.attach(5);  //  lifting servo orig 2
  servo2.attach(6);  //  left servo  orig 3
  servo3.attach(7);  //  right servo orig 4
  delay(1000);
  
  //initialize ultrasonic sensor pins
  pinMode(SONIC_ECHO_PIN,INPUT); //set digital input sonic echo pin  
  pinMode(SONIC_TRIG_PIN,OUTPUT); //set digital output sonic trigger pin
  digitalWrite(SONIC_TRIG_PIN,LOW);//initialize sonic trigger pin low
  
  //initialize moving average of IR Readings and ultrasonic
  for (int i = 1;i<=9;i++){
    if(input_select==SHARP_IR)
      j=ComputeMoveAvg(analogRead(IR_pin));
    if (input_select==SONIC)
      j=ComputeMoveAvg(ReadSonic());
      //Serial.println(j);
  }
  Serial.println(" to main loop");
  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  int time = millis();
  InitDisplay();
}

uint8_t i=0;
void loop() {
  uint8_t buttons = lcd.readButtons();
  
  //debug run RobotSeen() and print out values
  boolean test=RobotSeen();
  
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  
  //ready to time, wait for robot to be sensed at start line
  while (!timing_started && !timing){
    //Serial.println(ReadSonic());
    if (RobotSeen()){
      //Serial.print("robot seen");
      //Serial.println(ComputeMoveAvg(ReadSonic()));
      timing_started=true;
      start_time=millis(); //wait for select button
    }
  }
  //initialize display for timing and reset cleared flag
  if(timing_started && !timing){
    robot_cleared=false; //wait until robot cleared before checking if robot cross finish line
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Elapsed Time..");
    timing=true; //set timing flag to true
  }
  //elapsed time timing and display
  lcd.setCursor(0, 1);
  // print the elapsed time seconds:
  elapse_time=(millis()-start_time)/100;
  lcd.print(elapse_time);
  
  //check that robot cleared start line before checking to see if robot
  //crossed finish line, use 5 second buffer in case parts of
  //signal goes in/out as robot passes the IR proximity sensor
  if (timing && !robot_cleared && millis()-start_time>3000){
    if(!RobotSeen()){
        robot_cleared=true;
        //Serial.println("robot cleared");
    }    
}
  
  if (robot_cleared && timing&& RobotSeen() && !timing_finished){
     lcd.setCursor(0,0);
     lcd.print("Finish Time:   ");
     timing=false;
     timing_finished=false;
     robot_cleared=false;
     timing_started=false;
     PlotTime(elapse_time);
     start_time=0;
     elapse_time=0;
     while(!(buttons & BUTTON_SELECT)){
       buttons=lcd.readButtons();
       //Serial.println("waiting for button push"); //wait for select to start over
     }
     InitDisplay();
     
  }
  
//  uint8_t buttons = lcd.readButtons();
//
//  if (buttons) {
//    lcd.clear();
//    lcd.setCursor(0,0);
//    if (buttons & BUTTON_UP) {
//      lcd.print("UP ");
//      lcd.setBacklight(RED);
//    }
//    if (buttons & BUTTON_DOWN) {
//      lcd.print("DOWN ");
//      lcd.setBacklight(YELLOW);
//    }
//    if (buttons & BUTTON_LEFT) {
//      lcd.print("LEFT ");
//      lcd.setBacklight(GREEN);
//    }
//    if (buttons & BUTTON_RIGHT) {
//      lcd.print("RIGHT ");
//      lcd.setBacklight(TEAL);
//    }
//    if (buttons & BUTTON_SELECT) {
//      lcd.print("SELECT ");
//      lcd.setBacklight(VIOLET);
//    }
//  }
}
