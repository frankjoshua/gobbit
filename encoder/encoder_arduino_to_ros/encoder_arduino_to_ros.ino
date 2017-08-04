#include <SPI.h>
#include <Encoder_Buffer.h>
#include <ros.h>
#include <std_msgs/Int16.h>


#define EncoderCS1 48
#define EncoderCS2 49

#define LEFT 0
#define RIGHT 1

#define MIN_READ_VALUE 10

int encoder[] = {0,0};

Encoder_Buffer encoders[] = {Encoder_Buffer(EncoderCS1), Encoder_Buffer(EncoderCS2)};

ros::NodeHandle  nh;
std_msgs::Int16 int_msg[] = {std_msgs::Int16(), std_msgs::Int16()};
ros::Publisher wheels[] = {ros::Publisher("lwheel", &int_msg[LEFT]), ros::Publisher("rwheel", &int_msg[RIGHT])};

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //Serial.println("Starting...");
  SPI.begin();
  encoders[LEFT].initEncoder();
  encoders[RIGHT].initEncoder();

  
  nh.initNode();
  nh.advertise(wheels[LEFT]);
  nh.advertise(wheels[RIGHT]);

  pinMode(LED_BUILTIN, OUTPUT);
}

long wait = 0;
boolean side = false;

void loop() {

  if(side && millis() > wait){
    digitalWrite(LED_BUILTIN, HIGH);
    wait = millis() + 5;
    if(readEncoder(LEFT)){
      int_msg[LEFT].data = -encoder[LEFT];
      wheels[LEFT].publish( &int_msg[LEFT] );
    }
    side = !side;
  }
  nh.spinOnce();
  if(!side && millis() > wait){
    digitalWrite(LED_BUILTIN, LOW);
    wait = millis() + 5;
    if(readEncoder(RIGHT)){
      int_msg[RIGHT].data = encoder[RIGHT];
      wheels[RIGHT].publish( &int_msg[RIGHT] );
    }
    side = !side;
  }
  nh.spinOnce();
  
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

