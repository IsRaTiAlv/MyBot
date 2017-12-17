#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
const int en1 = 7; //EncoderDerecho
const int en2 = 8; //EncoderDerecho
const int salida = 13;

//Pines del motor
int in1=9;
int in2=10;
int in3=11;
int in4=12;

//ENCODER DERECHO
std_msgs::Int64 en1_msgs;
ros::Publisher pub_button("rwheel", &en1_msgs);
bool last_reading;
long last_debounce_time=0;
long debounce_delay=10;
bool published = true;
int count1=0;
//ENCODER Izquierdo
std_msgs::Int64 en2_msgs;
ros::Publisher pub_button2("lwheel", &en2_msgs);
bool last_reading2;
long last_debounce_time2=0;
long debounce_delay2=10;
bool published2 = true;
int count2=0;
int adelante1=0;
int adelante2=0;

void m1( const std_msgs::Int16& p){
  if(p.data<256){
    digitalWrite(in1,LOW);
    analogWrite(in2,p.data);
    adelante1=1;
    }
  else{
    int val=p.data-255;
    digitalWrite(in2,LOW);
    analogWrite(in1,val);
    adelante1=0;
    }
}

void m2( const std_msgs::Int16& p){
  if(p.data<256){
    digitalWrite(in4,LOW);
    analogWrite(in3,p.data);
    adelante2=1;
    }
  else{
    int val=p.data-255;
    digitalWrite(in3,LOW);
    analogWrite(in4,val);
    adelante2=0;
    }
}

ros::Subscriber<std_msgs::Int16> sub("motor1", &m1 );
ros::Subscriber<std_msgs::Int16> sub2("motor2", &m2 );

void setup()
{
  
  pinMode(salida, OUTPUT);
  pinMode(en1, INPUT); 
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  nh.initNode();
  nh.advertise(pub_button);
  nh.advertise(pub_button2);
  nh.subscribe(sub);
  nh.subscribe(sub2);
   
  digitalWrite(en1, HIGH);
  last_reading = ! digitalRead(en1);
  pinMode(en2, INPUT);  
  digitalWrite(en2, HIGH);
  last_reading2 = ! digitalRead(en2);
}
void encoder1(){
  bool reading = ! digitalRead(en1); 
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  } 
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(salida, reading);
    if(adelante1==1){
          count1++;
      }
      else{
          count1--;
        }
    en1_msgs.data = count1;
    pub_button.publish(&en1_msgs);
    published = true;
  }
  last_reading = reading;
  
}
void encoder2(){
  bool reading = ! digitalRead(en2); 
  if (last_reading2!= reading){
      last_debounce_time2 = millis();
      published2 = false;
  } 
  if ( !published2 && (millis() - last_debounce_time2)  > debounce_delay2) {
    digitalWrite(salida, reading);
    if(adelante2==1){
          count2++;
      }
      else{
          count2--;
        }
    en2_msgs.data = count2;
    pub_button2.publish(&en2_msgs);
    published2 = true;
  }
  last_reading2 = reading;
  
}
void loop()
{ 
  encoder1();
  encoder2();
  nh.spinOnce();
  delay(1);
}
