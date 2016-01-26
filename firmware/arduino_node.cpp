#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Float32.h>
#include <scorbot_firmware/read_joint_states.h>
#include <scorbot_firmware/write_joint_cmd.h>
ros::NodeHandle  nh;
using scorbot_firmware::read_joint_states;
using scorbot_firmware::write_joint_cmd;

float joint_angle_deg[5] = {0,0,0,0,0};
float motor_commands[5] = {0,0,0,0,0};


void read_joint_cb(const read_joint_states::Request & req, read_joint_states::Response & res)
{
  res.joint0 = joint_angle_deg[0];
  res.joint1 = joint_angle_deg[1];
  res.joint2 = joint_angle_deg[2];
  res.joint3 = joint_angle_deg[3];
  res.joint4 = joint_angle_deg[4];

}

void write_cmd_cb(const write_joint_cmd::Request & req, write_joint_cmd::Response & res)
{ 
  motor_commands[0] = req.cmd0;
  motor_commands[1] = req.cmd1;
  motor_commands[2] = req.cmd2;
  motor_commands[3] = req.cmd3;
  motor_commands[4] = req.cmd4;

}


ros::ServiceServer<read_joint_states::Request, read_joint_states::Response> read_server("read_joint_states",&read_joint_cb);
ros::ServiceServer<write_joint_cmd::Request, write_joint_cmd::Response> write_server("write_joint_cmd",&write_cmd_cb);


void setup()
{
  nh.initNode();
  nh.advertiseService(read_server);
  nh.advertiseService(write_server);
}

void loop()
{

  nh.spinOnce();
  delay(10);
}


