/*
 * Avans Arduin shield
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>

#define NUMBER_OF_ADC       2
#define NUMBER_OF_BUTTON    4
#define NUMBER_OF_LED       8

ros::NodeHandle nh;

std_msgs::UInt8 pushed_msg;
std_msgs::UInt8 state_msg;
ros::Publisher *pub_button_pushed;
ros::Publisher *pub_button_state;


const int button_pin[NUMBER_OF_BUTTON] = {8, 9, 10, 11};

unsigned char last_reading;
long last_debounce_time = 0;
long debounce_delay=50;
bool published = true;

const int led_pin[NUMBER_OF_LED] = {12, 13, 2, 3, 4, 5, 6, 7};


std_msgs::UInt16 adc_value_msg[2];
ros::Publisher *pub_adc_value[2];

void ledToggleCb(const std_msgs::UInt8& state_msg){

  bool pin_data;
  unsigned char data = state_msg.data;

  for(int i = 0; i < NUMBER_OF_LED; i++){
    pin_data = data & 0x01 ? true : false;
    if(pin_data){
      digitalWrite(led_pin[i], HIGH-digitalRead(led_pin[i]));   // blink the led
    }
    data = data >> 1;
  }
}

void ledStateCb(const std_msgs::UInt8& state_msg){
  bool pin_data;
  unsigned char data = state_msg.data;

  for(int i = 0; i < NUMBER_OF_LED; i++){
    pin_data = data & 0x01 ? false : true;
    digitalWrite(led_pin[i], pin_data);
    data = data >> 1;
  }
}

ros::Subscriber<std_msgs::UInt8> *led_topic_sub;
ros::Subscriber<std_msgs::UInt8> *led_state_sub;
char led_toggle_topic[32];
char led_state_topic[32];
char adc_value_topic[NUMBER_OF_ADC][32];

char button_push_topic[32];
char button_state_topic[32];

void setup()
{

  nh.initNode();
  sprintf(button_push_topic, "avans/buttons/pushed");
  pub_button_pushed = new ros::Publisher(button_push_topic, &pushed_msg);
  nh.advertise(*pub_button_pushed);

  sprintf(button_state_topic, "avans/buttons/state");
  pub_button_state = new ros::Publisher(button_state_topic, &state_msg);
  nh.advertise(*pub_button_state);

  last_reading = 0;
  unsigned char bit_value = 0x01;
  for(int i = 0; i < NUMBER_OF_BUTTON; i++){
    //pinMode(led_pin[i], OUTPUT);
    pinMode(button_pin[i], INPUT);

    //Enable the pullup resistor on the button
    digitalWrite(button_pin[i], HIGH);

    //The button is a normally button

    last_reading |= digitalRead(button_pin[i]) ? 0 : bit_value;
    bit_value = bit_value << 1;
  }


  sprintf(led_toggle_topic, "avans/leds/toggle");
  led_topic_sub = new ros::Subscriber<std_msgs::UInt8>(led_toggle_topic, ledToggleCb);
  nh.subscribe(*led_topic_sub);


  sprintf(led_state_topic, "avans/leds/state");
  led_state_sub = new ros::Subscriber<std_msgs::UInt8>(led_state_topic, ledStateCb);
  nh.subscribe(*led_state_sub);


  for(int i = 0; i < NUMBER_OF_LED; i++){
    pinMode(led_pin[i], OUTPUT);
    digitalWrite(led_pin[i], HIGH);
  }

  for(int i = 0; i < NUMBER_OF_ADC; i++){
    sprintf(adc_value_topic[i], "avans/adc%i/value", i);
    pub_adc_value[i] = new ros::Publisher(adc_value_topic[i], &adc_value_msg[i]);

    nh.advertise(*pub_adc_value[i]);
  }
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void loop()
{

  unsigned char reading = 0;

  for(int i = 0; i < NUMBER_OF_BUTTON; i++){
    reading = reading << 1;
    reading |= !digitalRead(button_pin[i]) ? 0x01 : 0x00;
  }

  if (last_reading!= reading){
    last_debounce_time = millis();
    published = false;
  }

  //if the button value has not changed for the debounce delay, we know its stable
  if (!published && (millis() - last_debounce_time)  > debounce_delay) {
    pushed_msg.data = reading;
    pub_button_pushed->publish(&pushed_msg);
    published = true;
  }

  state_msg.data = reading;
  pub_button_state->publish(&state_msg);

  last_reading = reading;

  for(int i = 0; i < NUMBER_OF_ADC; i++){
    int adc_value = averageAnalog(i);
    adc_value_msg[i].data = adc_value;
    pub_adc_value[i]->publish(&adc_value_msg[i]);
  }


  nh.spinOnce();
  //delay(100);
}
