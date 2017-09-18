#include <SPI.h>
#include <Encoder_Buffer.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <SabertoothSimplified.h>

#define EncoderCS1 48
#define EncoderCS2 49

#define MOTOR_LEFT 2
#define MOTOR_RIGHT 1

#define MAX_SPEED 127
//Minimum speed required to turn the motors
#define MIN_SPEED 30

#define ENCODER_LEFT 0
#define ENCODER_RIGHT 1

#define MIN_READ_VALUE 1

SabertoothSimplified ST(Serial1);

int encoder[] = {0,0};

Encoder_Buffer encoders[] = {Encoder_Buffer(EncoderCS1), Encoder_Buffer(EncoderCS2)};

ros::NodeHandle  nh;
std_msgs::Int16 int_msg[] = {std_msgs::Int16(), std_msgs::Int16()};
ros::Publisher wheels[] = {ros::Publisher("lwheel", &int_msg[ENCODER_LEFT]), ros::Publisher("rwheel", &int_msg[ENCODER_RIGHT])};

int mSpeedRight = 0;
int mSpeedLeft = 0;
int mGoalLeftPower = 0;
int mGoalRightPower = 0;
int mCurrentLeftPower = 0;
int mCurrentRightPower = 0;
unsigned long mTimeElapsed = 0;
unsigned long mLastUpdate = 0;

/*
* Helper function to map Floats, based on Arduino map()
*/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void messageCb( const geometry_msgs::Twist& toggle_msg){
   if(toggle_msg.linear.x > 0.05){
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  //Generate speeds for left and right motors
  //float right = 0.5;
  float right = toggle_msg.linear.x - toggle_msg.angular.z / 2;
  float left = toggle_msg.linear.x + toggle_msg.angular.z / 2;
  //Map speeds -1.0 to 1.0 to motor speeds
  mSpeedRight = mapfloat(right, -1, 1, -MAX_SPEED, MAX_SPEED);
  mSpeedLeft = mapfloat(left, -1, 1, -MAX_SPEED, MAX_SPEED);

  mGoalLeftPower = constrain(mSpeedLeft, -MAX_SPEED, MAX_SPEED);
  mGoalRightPower = constrain(mSpeedRight, -MAX_SPEED, MAX_SPEED);

  //Make sure goal is not below the minimum
  if(mGoalLeftPower > 3 && mGoalLeftPower < MIN_SPEED){
    mGoalLeftPower = MIN_SPEED;
  }
  if(mGoalLeftPower < -3 && mGoalLeftPower > -MIN_SPEED){
    mGoalLeftPower = -MIN_SPEED;
  }
  if(mGoalRightPower > 3 && mGoalRightPower < MIN_SPEED){
    mGoalRightPower = MIN_SPEED;
  }
  if(mGoalRightPower < -3 && mGoalRightPower > -MIN_SPEED){
    mGoalRightPower = -MIN_SPEED;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb );

//Speed 127 / 2000 = 0.0635
//Speed 65 / 2000 = 0.0325
const float SPEED = 0.0635;
// = 1 / SPEED rounded up
const float updateSpeed = 100; 

int updateGoal(int goal, int current, int timeElapsed){
  float change = timeElapsed * SPEED;
  if(current + change < goal){
    return current + change;
  }
  if(current - change > goal){
    return current - change;
  }
  return current;
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //Serial.println("Starting...");
  SPI.begin();
  encoders[ENCODER_LEFT].initEncoder();
  encoders[ENCODER_RIGHT].initEncoder();
  Serial1.begin(9600);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(wheels[ENCODER_LEFT]);
  nh.advertise(wheels[ENCODER_RIGHT]);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

long wait = 0;
boolean side = false;

void loop() {
  nh.spinOnce();
  if(side && millis() > wait){
    wait = millis() + 5;
    if(readEncoder(ENCODER_LEFT)){
      //Serial.println(-encoder[ENCODER_LEFT]);
      int_msg[ENCODER_LEFT].data = encoder[ENCODER_LEFT];
      wheels[ENCODER_LEFT].publish( &int_msg[ENCODER_LEFT] );
    }
    side = !side;
  }
  nh.spinOnce();
  if(!side && millis() > wait){
    wait = millis() + 5;
    if(readEncoder(ENCODER_RIGHT)){
      //Serial.println(encoder[ENCODER_RIGHT]);
      int_msg[ENCODER_RIGHT].data = -encoder[ENCODER_RIGHT];
      wheels[ENCODER_RIGHT].publish( &int_msg[ENCODER_RIGHT] );
    }
    side = !side;
  }
  nh.spinOnce();
  //Udpate motors if needed
  mTimeElapsed = millis() - mLastUpdate;
  if(mTimeElapsed > updateSpeed){
    if(mCurrentRightPower != mGoalRightPower || mCurrentLeftPower != mGoalLeftPower){
      mCurrentLeftPower = updateGoal(mGoalLeftPower, mCurrentLeftPower, mTimeElapsed);
      mCurrentRightPower = updateGoal(mGoalRightPower, mCurrentRightPower, mTimeElapsed);     
      //Send new speed to the motors
      ST.motor(MOTOR_LEFT, constrain(mCurrentLeftPower, -127, 127));
      ST.motor(MOTOR_RIGHT, constrain(mCurrentRightPower, -127, 127));
    }
    mLastUpdate = millis();
  }
}

bool readEncoder(int pos){
  int reading = encoders[pos].readEncoder();
  if(abs(reading) > MIN_READ_VALUE){
    encoder[pos] += reading / MIN_READ_VALUE;
    encoders[pos].clearEncoderCount();
    return true;
  }
  return false;
}

