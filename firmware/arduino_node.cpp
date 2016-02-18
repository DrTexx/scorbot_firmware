/* Author: Zubin Priyansh
   Desc:   Node that runs on arduino mega interfacing with a scorbot ERV 
*/

#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Float32.h>
#include <scorbot_firmware/scorbot_joint_states.h>
#include <scorbot_firmware/scorbot_joint_cmd.h>
#include <Encoder_Scorbot.h>
#include <arduino_node.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329252
#endif
//these magic constants are imported from the MTIS 
//scorbot controller matlab files for scorbot ER3
//modified for ERV plus model
#define enc_kb -141.8888
#define enc_kse -113.5111
#define enc_offs 120.27
#define enc_offe -25.24
#define enc_kw -27.9
#define enc_offw 63.57


ros::NodeHandle  nh;
float joint_angle_deg[5] = {0,0,0,0,0};
float joint_angle_rad[5] = {0,0,0,0,0};

float motor_commands[5] = {0,0,0,0,0};

bool new_data_ready = false;
scorbot_firmware::scorbot_joint_cmd input_cmd_msg;
scorbot_firmware::scorbot_joint_states joint_state_msg;
void joint_cmd_cb(const scorbot_firmware::scorbot_joint_cmd& msg)
{
    new_data_ready =true;
    input_cmd_msg           = msg;        
}



ros::Publisher joint_state_publisher("/scorbot_arduino_states", &joint_state_msg);


ros::Subscriber<scorbot_firmware::scorbot_joint_cmd> joint_cmd_subscriber("/scorbot_arduino_cmd", joint_cmd_cb );


//Non ROS Functions
void motorFW(const int motorNum, byte rpm)
{
  digitalWrite(M1[motorNum], HIGH);
  digitalWrite(M2[motorNum], LOW);
  analogWrite(Mpwm[motorNum], rpm);
}

void motorBW(const int motorNum, byte rpm)
{
    digitalWrite(M1[motorNum], LOW);
    digitalWrite(M2[motorNum], HIGH);
    analogWrite(Mpwm[motorNum], rpm);
}

void motorHalt(const int motorNum)
{
    digitalWrite(M1[motorNum], HIGH);
    digitalWrite(M2[motorNum], HIGH);
    //analogWrite(Mpwm[motorNum], 255);

}

void motorST(const int motorNum)
{
    digitalWrite(M1[motorNum], LOW);
    digitalWrite(M2[motorNum], LOW);
    //analogWrite(Mpwm[motorNum], 0);
}

void update_counter()
{
  for(int i=0;i<5;i++)
  {
    //we subtract the reference as we cannot directly modify the encoder 
    counter[i] = encoder_data(i) - encoder_reference[i];
  }
}

void counts_to_rad()
{
  update_counter();

  //keeping the angles in degrees as it is more intuitive while debugging
  joint_angle_deg[0] = counter[0] / enc_kb;
  joint_angle_deg[1] = counter[1] / enc_kse - enc_offs;
  joint_angle_deg[2] = enc_offs + enc_offe - (counter[1] + counter[2]) / enc_kse;
  joint_angle_deg[3] = (counter[4] - counter[3]) / (2 * enc_kw) + counter[2] / enc_kse + enc_offw - enc_offe;
  joint_angle_deg[4] = (counter[3] + counter[4])/(-2 * enc_kw);

  for(int i=0;i<5;i++)
  {
   joint_angle_rad[i] = joint_angle_deg[i] * DEG_TO_RAD;
  }
}



void setup()
{
  //ROS related
  nh.getHardware()->setBaud(115200); //for higher speed
  nh.initNode();
  nh.advertise(joint_state_publisher);
  nh.subscribe(joint_cmd_subscriber);

  //Scorbot related
  encoder_begin();  // Start the library, it basically attaches the timer
  for (int i=0;i<6;i++) {
    
      pinMode(M1[i],OUTPUT);
      pinMode(M2[i],OUTPUT);
      attach_encoder(i, pin_P0[i], pin_P1[i]);  // Attach an encoder to pins A and B
      motorST(i); //STOP command
      pinMode(Mpwm[i],OUTPUT);
      pinMode(pin_P0[i],INPUT_PULLUP);//pullup the pins for encoders
      pinMode(pin_P1[i],INPUT_PULLUP);//pullup the pins for encoders
      digitalWrite(Mpwm[i], LOW);
      pinMode(MS[i],INPUT); //microswitches are the inputs
    }
    attachInterrupt(0, resetCounter0, RISING);  //attach interrupt for the microswitches //2
    attachInterrupt(1, resetCounter1, RISING); //3  
    attachInterrupt(4, resetCounter2, RISING);  //the numbering seems funny
    attachInterrupt(2, resetCounter3, RISING);  
    attachInterrupt(3, resetCounter4, RISING);
}

void loop()
{
  if(new_data_ready)
  {
    joint_state_msg.joint0 = joint_angle_rad[0]; //* input_cmd_msg.cmd0;
    joint_state_msg.joint1 = joint_angle_rad[1]; //* input_cmd_msg.cmd1;
    joint_state_msg.joint2 = joint_angle_rad[2]; //* input_cmd_msg.cmd2;
    joint_state_msg.joint3 = joint_angle_rad[3]; //* input_cmd_msg.cmd3;
    joint_state_msg.joint4 = joint_angle_rad[4]; //* input_cmd_msg.cmd4;
  }
  else
  {
    joint_state_msg.joint0 = 0;
    joint_state_msg.joint1 = 0;
    joint_state_msg.joint2 = 0;
    joint_state_msg.joint3 = 0;
    joint_state_msg.joint4 = 0;
  }
  
  joint_state_publisher.publish(&joint_state_msg);
  nh.spinOnce();
  delay(100);
}


