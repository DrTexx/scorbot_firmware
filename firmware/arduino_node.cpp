#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Float32.h>
#include <scorbot_firmware/scorbot_joint_states.h>
#include <scorbot_firmware/scorbot_joint_cmd.h>
ros::NodeHandle  nh;

float joint_angle_deg[5] = {0,0,0,0,0};
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

// void read_joint_cb(const read_joint_states::Request & req, read_joint_states::Response & res)
// {
//   // res.joint0 = joint_angle_deg[0];
//   // res.joint1 = joint_angle_deg[1];
//   // res.joint2 = joint_angle_deg[2];
//   // res.joint3 = joint_angle_deg[3];
//   // res.joint4 = joint_angle_deg[4];

//   // Dummy test, previous commands passed through as current states
//   res.joint0 = motor_commands[0];
//   res.joint1 = motor_commands[1];
//   res.joint2 = motor_commands[2];
//   res.joint3 = motor_commands[3];
//   res.joint4 = motor_commands[4];

// }

// void write_cmd_cb(const write_joint_cmd::Request & req, write_joint_cmd::Response & res)
// { 
//   motor_commands[0] = req.cmd0;
//   motor_commands[1] = req.cmd1;
//   motor_commands[2] = req.cmd2;
//   motor_commands[3] = req.cmd3;
//   motor_commands[4] = req.cmd4;

// }


  // ros::ServiceServer<read_joint_states::Request, read_joint_states::Response> read_server("read_joint_states",&read_joint_cb);
  // ros::ServiceServer<write_joint_cmd::Request, write_joint_cmd::Response> write_server("write_joint_cmd",&write_cmd_cb);




void setup()
{
  //randomSeed(analogRead(0));
  nh.initNode();
  nh.advertise(joint_state_publisher);
  nh.subscribe(joint_cmd_subscriber);
}

void loop()
{
  if(new_data_ready)
  {
    joint_state_msg.joint0 = input_cmd_msg.cmd0;
    joint_state_msg.joint1 = input_cmd_msg.cmd1;
    joint_state_msg.joint2 = input_cmd_msg.cmd2;
    joint_state_msg.joint3 = input_cmd_msg.cmd3;
    joint_state_msg.joint4 = input_cmd_msg.cmd4;
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


