#include "ros/ros.h"
#include "xarm_ros_client.h"
#include <xarm_driver.h>

// Please run "export ROS_NAMESPACE=/xarm" first

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xarm_modbus");
	ros::NodeHandle nh;

	xarm_api::XArmROSClient client;
	client.init(nh);

	int recv_bytes = 7;
	unsigned char send_data[6] = {0x01,0x06,0x00,0x0A,0x00,0x03}, recv_data[recv_bytes]={0};
	
	int ret = 0;
	ros::Rate rate(10);

	ret = client.send_tool_modbus(send_data, 6, recv_data, recv_bytes);
	fprintf(stderr, "ret = %d, recv_data: ", ret);

	for(int i=0; i<recv_bytes; i++)
	{
		fprintf(stderr, "%x\t", recv_data[i]);
	}
	fprintf(stderr, "\n");
	rate.sleep();

	return 0;
}