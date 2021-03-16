#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      3                       // Quadrature encoder B pin
#define M1              11                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10
#define M3              13                       // PWM outputs to L298N H-Bridge motor driver module
#define M4              12
#define LOOPTIME        100
#define trig1 4
#define trig2 5
#define trig3 6
#define echo1 7
#define echo2 8
#define echo3 9

double kpL = 0 , kiL = 200000 , kdL = 0;             // modify for optimal performance
double inputL = 0, outputL = 0, setpointL = 0;
double kpR = 0, kiR = 200000, kdR = 0;             // modify for optimal performance
double inputR = 0, outputR = 0, setpointR = 0;
long temp;
volatile long encoderPosL = 0, encoderPosR = 0;
double v_L, v_R;
unsigned long lastMilli = 0;
PID myPIDL(&inputL, &outputL, &setpointL, kpL, kiL, kdL, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPIDR(&inputR, &outputR, &setpointR, kpR, kiR, kdR, DIRECT);
int distancefr=0, distanceleft=0, distanceright=0, durationfr=0, durationleft=0, durationright=0;

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;

const float diameter = 0.065;                   //Wheel radius, in m
const float wheelbase = 0.20;               //Wheelbase, in m

float speed_req = 0;                         //Desired linear speed for the robot, in m/s
float angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

float speed_req_left = 0;                    //Desired speed for left wheel in m/s                //Actual speed for left wheel in m/s
float speed_cmd_left = 0;                    //Command speed for left wheel in m/s

float speed_req_right = 0;                   //Desired speed for right wheel in m/s


ros::NodeHandle nh;
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {                                                //Reset the counter for number of main loops without communication
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
std_msgs::Float32 left_msg, right_msg ;
nav_msgs::Odometry odom;
ros::Publisher speedLeft("speed_wheel_left", &left_msg);
ros::Publisher speedRight("speed_wheel_right", &right_msg);
ros::Publisher odom_pub("odom", &odom);
   void RobotMove(int action)
{
  switch (action)
  {
  case 0:
  digitalWrite(M1,0);
  digitalWrite(M2,0);
  digitalWrite(M3,0);
  digitalWrite(M4,0);
  break;
  case 1:  //Turn right
  digitalWrite(M1,1);
  digitalWrite(M2,0);
  digitalWrite(M3,0);
  digitalWrite(M4,1);
  break;
  case 2: //Turn left
  digitalWrite(M1,1);
  digitalWrite(M2,0);
  digitalWrite(M3,1);
  digitalWrite(M4,0);
  break;
  case 3: //Go straight
  digitalWrite(M1,1);
  digitalWrite(M2,0);
  digitalWrite(M3,0);
  digitalWrite(M4,1);
  break;
  case 4: 
  digitalWrite(M2,0);
  analogWrite(M1,255/2);
  digitalWrite(M4,0);
  analogWrite(M3,255/2);
  
  break;  
}}
int frontdistance()
{
  digitalWrite(trig1,0); 
  delayMicroseconds(2);
  digitalWrite(trig1,1);    // phát xung từ chân trig
  delayMicroseconds(5);     // xung có độ dài 5 microSeconds
  digitalWrite(trig1,0);    // tắt chân trig 
  durationfr = pulseIn(echo1,HIGH);
  distancefr = int(durationfr/2/29.412);  
  return distancefr;
}
int leftdistance()
{
  digitalWrite(trig2,0); 
  delayMicroseconds(2);
  digitalWrite(trig2,1);    // phát xung từ chân trig
  delayMicroseconds(5);     // xung có độ dài 5 microSeconds
  digitalWrite(trig2,0);    // tắt chân trig 
  durationleft = pulseIn(echo2,HIGH);
  distanceleft = int(durationleft/2/29.412);
  return distanceleft;
}
int rightdistance()
{
  digitalWrite(trig3,0); 
  delayMicroseconds(2);
  digitalWrite(trig3,1);    
  delayMicroseconds(5);     
  digitalWrite(trig3,0);   
  durationright = pulseIn(echo3,HIGH);
  distanceright = int(durationright/2/29.412);
  return distanceright;
}
void Avoider()
{
  int dis_fr = frontdistance();
  int dis_left = leftdistance();
  int dis_right = rightdistance();
  int max_distance;
  if(dis_fr > 30)
  {
    RobotMove(3);
    delay(10000);
  }
    if(dis_fr < 30)
    {
      RobotMove(4);
      delay(500);
      RobotMove(0);
      delay(500);
      max_distance = max(dis_left,dis_right);
      if(max_distance == dis_left)
      {
        RobotMove(2);
        delay(500);
      }
      else
      {
        if(max_distance == dis_right)
        {
          RobotMove(1);
          delay(500);
        }
      }}}
void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);
    pinMode(M1, OUTPUT);
     pinMode(M2, OUTPUT); 
      pinMode(M3, OUTPUT); 
       pinMode(M4, OUTPUT); // quadrature encoder input A
  attachInterrupt(0, encoderL, FALLING);
  attachInterrupt(1, encoderR, FALLING); // update encoder position v
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speedLeft);
  nh.advertise(speedRight);
  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetSampleTime(1);
  myPIDL.SetOutputLimits(0, 255);
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetSampleTime(1);
  myPIDR.SetOutputLimits(0, 255); 
    pinMode(trig1,OUTPUT);
  pinMode(trig2,OUTPUT);
  pinMode(trig3,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(echo2,INPUT);
  pinMode(echo3,INPUT);
}

void loop() {

  // data from encoder
  // monitor motor position
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop
    lastMilli = millis();
  //  Avoider();
    if (speed_req_left < 0) {
      setpointL =-speed_req_left;
       inputL = (encoderPosL / 550.00) * 10 * 3.14 * diameter;
      encoderPosL = 0;
      myPIDL.Compute();
      analogWrite(M1, 255-outputL);                           // drive motor CW
      digitalWrite(M2, HIGH);
      left_msg.data = -inputL;

    }
    if (speed_req_left >= 0) {
      setpointL = (speed_req_left);
      inputL = (encoderPosL / 550.00) * 10 * 3.14 * diameter ;
       encoderPosL = 0;
      myPIDL.Compute();
      analogWrite(M1, outputL);                             // drive motor CW
      digitalWrite(M2, LOW);
      left_msg.data = inputL;
      
     
    }
    if (speed_req_right < 0) {
      setpointR =-speed_req_right;
      inputR = (encoderPosR / 550.00) * 10 * 3.14 * diameter;
       encoderPosR = 0;
      myPIDR.Compute();
       analogWrite(M3, outputR);                           // drive motor CW
      digitalWrite(M4, LOW);
      right_msg.data = -inputR;
      
    }

    if (speed_req_right >= 0) {
      setpointR = (speed_req_right);
      inputR = (encoderPosR / 550.00) * 10 * 3.14 * diameter;
       encoderPosR = 0;
      myPIDR.Compute(); 
      analogWrite(M3, 255-outputR);                             // drive motor CW
      digitalWrite(M4, HIGH);
     right_msg.data = inputR;

    }
    speedLeft.publish( &left_msg );
    speedRight.publish( &right_msg );
  }

  // drive L298N H-Bridge module
}



void encoderL()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinA1) == LOW )  encoderPosL++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
void encoderR()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB1) == LOW )  encoderPosR++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
