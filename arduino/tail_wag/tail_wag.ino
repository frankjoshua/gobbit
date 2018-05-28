/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <Servo.h>

Servo mServo;

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  //Wag the tail
  wag();
}

ros::Subscriber<std_msgs::Empty> sub("wag_tail", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

void wag() {
  //Connect to the servo
  mServo.attach(9);
  //Sweep back and forth a few times
  for(int i = 0; i < 4; i++){
    for (int pos = 170; pos >= 90; pos -= 1) {
      mServo.write(pos);
      delay(2);
    }
    for (int pos = 90; pos <= 170; pos += 1) {
      // in steps of 1 degree
      mServo.write(pos);
      delay(2);
    }
  }
  //Disconect
  mServo.detach();
}

