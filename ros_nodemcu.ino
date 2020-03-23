#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#define LED D0 

const char* ssid = "***";
const char* password = "***";
IPAddress server(192,168,1,***);      // Set the rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411;

void setupWiFi() {                    // connect to ROS server as as a client
  if(DEBUG){
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  if(DEBUG){
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> Sub("toggle_led", &messageCb );

void messageCb( const std_msgs::Int16& toggle_msg){
  int i;
  for(i=0;i<toggle_msg.data;i++)
   {
  digitalWrite(13, HIGH-digitalRead(13));
  delay(200);
  // blink the led
   }
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
setupWiFi();
delay(2000);
nh.initNode();
nh.subscribe(Sub);
pinMode(LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

nh.spinOnce();
delay(1000);
}
