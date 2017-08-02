//MBot driver layer
//This hardware manages all the hardware ie: wheelspeed, sensor interafce etc

//Interface HB25 Motor Control from Parallax 

//BOARD: Arduino MEGA
//SHIELDS:
//http://www.dfrobot.com/wiki/index.php/LCD_KeyPad_Shield_For_Arduino_SKU:_DFR0009



//STARTUP PROCEDURE:
//Turn the Mega on BEFORE turning on the HB25 motor controllers

//SHUTDOWN PROCEDURE
//Turn the HB25 motor controllers off BEFORE turning the Mega off

#include <Servo.h>
#include <LiquidCrystal.h>

//DO NOT CHANGE THESE PIN ASSIGNMENTS
Servo leftWheelServo;
Servo rightWheelServo;
const int leftWheel = 13;   //Servo control output for HB25
const int rightWheel = 12;  //Servo control output for HB25

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//The following pins are used for the encoder inputs
const int chn_l_a = 2;  //Left wheel encoder channel A - Interupt 0
const int chn_l_b = 3;  //Left wheel encoder channel B - Interupt 1
const int chn_r_a = 20;  //Right wheel encoder channel A - Interupt 3
const int chn_r_b = 21;  //Right wheel encoder channel B - Interupt 2

//DO NOT CHANGE THESE ASSIGNMENTS
const boolean fwd = 1;
const boolean rev = 0;
const int encoderArray[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};    //Array used to increment encoder values
const float pi = 3.142;
float v = 0.0;      //Velocity of center point of robot measured in mm/second
float w = 0.0;    //Vehicle's rotational speed measured in millie radials per seconds
unsigned long int old_time = 0;
unsigned long int time = 0;
int delta_t = 99;    //time used by PID controller to update control data in milli-seconds
const float PID_dc = 1000 / (delta_t + 1);    //Calcualte how many times this loop fires per second ie converts delta_t to Hz

//### Time variables used to timewhen new data must be sent serially
unsigned long int oldTimeTX = 0;
int deltaTX = 299;

const int lf = 10;    //Declares a line feed character


//Robot STATE variables
float s_l = 0.0;    //Linear distance in milli-meters traveled by left wheel PER UPDATE CYCLE OF PID LOOP
float s_r = 0.0;    //Linear distance in milli-meters traveled by right wheel PER UPDATE CYCLE OF PID LOOP
float s = 0.0;      //Linear distance traveled by CENTER POINT of robot in milli-meters PER UPDATE CYCLE OF PID LOOP
float phi = 0.0;    //Instantaneous heading change
float delta_x = 0.0;  //Instantaneous change in x pos
float delta_y = 0.0;  //Instantaneous change in y pos
float delta_phi = 0.0;//Instantaneous change in phi

//DO NOT CHANGE THE FOLLOWING VARIABLES
//These variable are used in the operation of the speed controller
int currLeft = 0;    //Used to calculate change delta ticks on left wheel
int prevLeft = 0;    //Used to calculate change delta ticks on left wheel
int currRight = 0;    //Used to calculate change delta ticks on right wheel
int prevRight = 0;    //Used to calculate change delta ticks on right wheel
int encCntLeft = 0;    //Actual measured amount of ticks on left wheel
int prevEncCntLeft = 0;
int encCntRight = 0;   //Actual measured amount of ticks on right wheel
int prevEncCntRight = 0;
int encCntDiv = 0;
int encIn = 0;        //Variable used to hold the combination of prev and current encoder values
float error_l = 0.0;
float error_r = 0.0;
float e_old = 0.0;
float e_old2 = 0.0;
float e_old_r = 0.0;
float e_old2_r = 0.0;
float ticks_l = 0.0;
float ticks_r = 0.0;
float ticks_desired_l = 0.0;
float ticks_desired_r = 0.0;
float l_mot = 0.0;    //left motor reference speed
float l_control = 1500.0;
float r_mot = 0.0;    //right motor reference speed
float r_control = 1500.0;
float error_dist = 0.0;
float error_dist_old = 0.0;
float error_mot;

//ONLY CHANGE THIS UNDER SPECIAL CIRCUMSTANCES
//Constants for the robot's physical parameters
const int ticks_per_rev = 144; //36; //1632;  //Ticks per revolution
const float wheelbase = 392.74;      //Wheelbase of chassis in milli-meters
const float wheel_radius = 76.58;  //Wheel radii in milli-meters
const float wheel_circ = 2*pi*wheel_radius;
const float seg_lin_distance = wheel_circ / ticks_per_rev;
//Gain parameters for the speed controller PID controller
float Kp = 0.5;      // Proportional gain for PID controller
float Ki = 0.5;
float Kd = 0.1;
float Kp_speed = 0.0;
float Ki_speed = 0.0;

//USER DEFINED VARIABLES
boolean debug = false;
String serialData = "";
char Str1[10];
int index = 0;
boolean done = false;
float robotState[] = {0.0,0.0,0.0};

float Kp_w = 0.1;      //Proportional gain parameter for turn rate
float Ki_w = 0.1;      //Integral gain parameter for turn rate;
float e_angle = 0.0;
float e_angle_old = 0.0;
float w_old = 0.0;
float phi_desired = 0.0;

const int maxV = 400;
const int maxW = 2;

int txCntr = 0;     //CNTR used to show the amount of transactions already received

void setup()
{
  setupMotorControl();

  //### Comm port for comms to PC
  Serial.begin(115200);
  Serial.println("PC Comms Started:"); 

   lcd.begin(16,2);
   lcd.setCursor(0,0);
   lcd.print("Started"); 
   delay(1000); 
   lcd.clear();
   lcd.print("Running");
  
    v = 0;                  
    w = 0;    
    phi_desired = 0;    
}

void loop()
{  
    //###Lees seriedata tot 'n EOL karakter gekry word
    if (Serial.available() >0)
    {
      char c = Serial.read();
      
      switch(c)
      {
        case '<':      //new command line starts
        index = 0;
        break;
        
        case '\r':
          done = true;
          //serialData += '\0';      //Add EOL character for string
          break;
          
        default:
          serialData += c;        //Add serial data input char to string
          break;
      }      
    }   
    
    
    //Parse die seriedata
    if (done == true)
    {
      txCntr ++;      
      lcd.setCursor(0,1);
      lcd.print(txCntr,DEC);
      lcd.print(":");
      lcd.print(serialData);
      lcd.print(" - ");
    
      done = false;
      
      char testChar = serialData.charAt(0);
      
      switch (testChar)
      {
        case 'v':
        {
          int len = serialData.length();
          String value = "";
          for (int i = 1; i <= len; i++) 
            value += serialData.charAt(i);
            
          v = value.toInt();

          
          lcd.print(v);
          
          if (debug)
          {
            Serial.println ("Velocity");          
            Serial.println (v);
          }          
          break;
        }
        
        case 'w':
        {
          int len = serialData.length();          
          String value = "";
          
          for (int i = 1; i<=len; i++) 
            value += serialData.charAt(i);

            
          w = value.toInt();   

          //w is a float multiplied by 1000 to convert it to integer, now it must
          //  be divided by 1000 to convert it back to float
          w = w / 1000;   
          
          lcd.print("                ");
          lcd.print(w);
    
          if (debug)
          {
            Serial.println ("Turning rate");          
            Serial.println (w);          
          }          
          break;
        }
        
        case 'r':
        {
          robotState[0] = 0.0;
          robotState[1] = 0.0;
          robotState[2] = 0.0;
          phi_desired = 0.0;
          break;
        }
        
        case 'p':
        {
          //Serial.println("phi_desired");
          //Serial.println(serialData);
          int p_index = serialData.indexOf('p');
          int len = serialData.length();          
          String value = "";
          for (int i = 1; i <= len; i++) value += serialData.charAt(i);          
          int delta_phi = 0;
          delta_phi = value.toInt();
          float delta_phi_float = delta_phi/1000.0;
          phi_desired = delta_phi_float;
          //Serial.println(delta_phi_float);
          //Serial.println(phi_desired);         
          break;
        }

        case'.':    //Set v=0 and w=0 effectievly stopping the robot
        {
          v=0;
          w=0;
          break;
        }
        case 'd':
        {
          debug = !debug;      //Flip boolean value between TRUE and FALSE
          break;
        }
        
        case '?':
        {              
          
          break;
        }
      }
      serialData = "";
    }

    time = millis();
  

  //### This routine is executed every delta_t and is used to update the motor control values  
  int interval = time-old_time; 
  if (interval > delta_t)    //Enter IF after delay of delta_t seconds
  {
    //Serial.print(" PID ");    
    if (debug)
    {      
      Serial.print(robotState[0]);
      Serial.print(":");
      Serial.print(robotState[1]);
      Serial.print(":");
      Serial.print(robotState[2]);
      Serial.print('\t');
      Serial.print(s_l);
      Serial.print('\t');
      Serial.print(s_r);        
      Serial.print('\t');
      Serial.print(e_angle);
      Serial.print('\t');
      Serial.print(w);    
      Serial.println();
    }
    
    s_l = 2*pi*wheel_radius * ticks_l / PID_dc / ticks_per_rev;   //Must devide by PID_dc in order to get real delta value
    s_r = 2*pi*wheel_radius * ticks_r / PID_dc / ticks_per_rev;    
    s = (s_l + s_r) / 2;
    delta_phi = (s_r - s_l) / wheelbase;    //+w = counter clockwise
    
    delta_x = s*cos(robotState[2]);
    delta_y = s*sin(robotState[2]);
    robotState[0] += delta_x;
    robotState[1] += delta_y;    
    robotState[2] += delta_phi;  

    if (robotState[2] < -PI) robotState[2] += 2*PI;
    if (robotState[2] > PI) robotState[2] -= 2*PI;

    
    
    //Angle PID control
    /*
    phi_desired = w;
    e_angle = 0.1 * (phi_desired - robotState[2]);
    /*    
    w = w_old + Kp_w * (e_angle - e_angle_old) + Ki_w * (e_angle + e_angle_old)/2;
    w = min(w, 1);
    w = max(w, -1);
    w_old = w;
    e_angle_old = e_angle;    
    */       

//    phi_desired = w;
//    float errorAngle = phi_desired - robotState[2];
//
//    if (errorAngle < -PI) errorAngle += (2*PI);
//    if (errorAngle > PI) errorAngle -= (2*PI);  
//
//    w = 3*min(errorAngle,150);      
    
    v = min (v, maxV);
    v = max (v, -maxV);
    velocityControl(v,w);      //Sends new v and w values to the motor controller 

    old_time = time;    
  }  //end of PID Interval

  //### This routine sends serial data to PC every deltaTX time    
  int newTimeInterval = time - oldTimeTX;
  if (newTimeInterval > deltaTX)
  {
    //Serial.println(" TX Data ");
//    sendPos();
//    sendSensor();   

    oldTimeTX = time;
    delay(10);
    //Serial.println("Exit: send data");
  }  
} //end of Loop()

//////////////////////////////////////////////////////////////////////////////////////
// Funtions and Procedures start here

//Send robot position on serial port
void sendPos()
{
  Serial.print("?");
  Serial.print(robotState[0]);
  Serial.print(",");
  Serial.print(robotState[1]);        
  Serial.print(",");
  Serial.print(robotState[2]);         
  Serial.println(); 
}

//###
void setupMotorControl()
{  
  //Attaches the encoder inputs to interrupts in order to catch state changes
  //Ints attached to pins 2,3 and 20,21
  //If int fires, ISR readEncoder will be called and executed
  attachInterrupt(0,readencoder, CHANGE);
  attachInterrupt(1,readencoder, CHANGE);  
  attachInterrupt(2,readencoder, CHANGE);
  attachInterrupt(3,readencoder, CHANGE);  
  
  pinMode(chn_l_a,INPUT);
  digitalWrite(chn_l_a,HIGH);
  pinMode(chn_l_b,INPUT);
  digitalWrite(chn_l_b,HIGH);  
  pinMode(chn_r_a,INPUT);
  digitalWrite(chn_r_a,HIGH);  
  pinMode(chn_r_b,INPUT);  
  digitalWrite(chn_r_b,HIGH);

  leftWheelServo.attach(leftWheel);
  rightWheelServo.attach(rightWheel);
}

//Implementation of the vw (vOmega) interface from the book - Embedded Robotics, Chp 5.5, p98

//The following steps are followed
  //Calculate current ticks, for each wheel, based on encCntLeft and encCntRight values
  //Calculate wanted ticks for each wheel based on v and w values
  //Calculate the error value in the ticks
  //Apply values into PID loop for ech wheel and for distance the wheels travelled - Chp 5.4, p98
  //Use PID return values as new motor control inputs

void velocityControl(float v1, float w1)
{
  //PID_dc --> It is the duty cycle for the updates send to the PID controller.
  //Ticks_l and ticks_r must be multiplied with this value to calculate the total ticks per second
  ticks_l = (encCntLeft - prevEncCntLeft) * PID_dc;    //Calculates the ticks per second for left wheel
  ticks_r = (encCntRight - prevEncCntRight) * PID_dc;  //Calcualtes the ticks per second for the right wheel
    
  //wheel_circ = 1/(2*pi*radius-of-weel)
  float dotOmega_L = (v1 - wheelbase/2*w1)/wheel_circ;    //Embedded Robotics, Chp 8.6, p 143, Inverse Kinematics
  ticks_desired_l = dotOmega_L * ticks_per_rev;
  float dotOmega_R = (v1 + wheelbase/2*w1)/wheel_circ;
  ticks_desired_r = dotOmega_R * ticks_per_rev;
  
  /* OLD FORMULAS
  ticks_desired_l = (v1*PID_dc - (wheelbase*w1)/2) * (ticks_per_rev / (2*pi*wheel_radius));
  ticks_desired_r = (v1*PID_dc + (wheelbase*w1)/2) * (ticks_per_rev / (2*pi*wheel_radius));
  */
    
  error_l = (ticks_desired_l - ticks_l);
  error_r = (ticks_desired_r - ticks_r);
    
  //Left motor PID Controller  
  l_mot = Kp * (error_l - e_old) + Ki*(error_l + e_old)/2 + Kd*(error_l-2*e_old+e_old2); 
  //Right motor PID Controller
  r_mot = Kp * (error_r - e_old_r) + Ki*(error_r + e_old_r)/2 + Kd*(error_r-2*e_old_r+e_old2_r);    
  //Distance travelled between wheels PID controller
  error_dist = ticks_l - ticks_r + w1;
  error_mot = Kp_speed * (error_dist - error_dist_old) + Ki_speed * (error_dist + error_dist_old)/2;
  //Control signals for left and right motor
  l_control += l_mot - error_mot;   
  r_control += r_mot + error_mot; 

  
    
  //Clamps control signals to accepted values
  if (l_control > 2000) l_control = 2000;
  if (l_control < 1000) l_control = 1000;
  
  if (r_control > 2000) r_control = 2000;
  if (r_control < 1000) r_control = 1000;   
   
  leftWheelServo.writeMicroseconds(l_control);   
  rightWheelServo.writeMicroseconds(r_control);    
  

  prevEncCntLeft = encCntLeft;
  e_old2 = e_old;
  e_old = error_l;
    
  prevEncCntRight = encCntRight;
  e_old2_r = e_old_r;
  e_old_r = error_r; 
 
  error_dist_old = error_dist;       
}

//Interupt Service Routine
//Run whenever interupt 0,1,2,3 fires as attached in setupMotorControl() to the different wheel encoder inputs
//Implemented the quad interface from the following website: http://mkesc.co.uk/ise.pdf
void readencoder()
{   
  currLeft = B00110000 & PINE;    //Pin2 is mapped to PE4, Pin3 is mapped to PE5
  currRight = B00000011 & PIND;   //Pin21 is mappeed to PD0, Pin20 is mapped to PD1
  currLeft = currLeft >> 4;       //Shift currLeft variable to Bit0 and Bit1
  prevLeft = prevLeft << 2;       //Shift prevLeft value to Bit2 and Bit3
  encIn = currLeft | prevLeft;    //Combine currLeft and prevLeft into one variable used to determine
  encCntLeft += encoderArray[encIn];    //what value must be added to encCntLeft
  prevLeft = currLeft;
  
  prevRight = prevRight << 2;
  encIn = currRight | prevRight;
  encCntRight += encoderArray[encIn];
  prevRight = currRight;
}
