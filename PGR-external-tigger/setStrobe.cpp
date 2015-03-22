
//ROS Communications
#include <ros/ros.h>
	// Messages
	#include <hast/flag.h>

	// Services
	#include <camera1394/SetCameraRegisters.h>


class setStrobe
{
private:	
	/*--------- ROS Communication Containers ------------- */
	ros::NodeHandle n;
	ros::Subscriber HastShutDown_sub;

	uint8_t register_type;
	uint64_t register_offset;
	std::vector<uint32_t> value;
	uint32_t mode;

public:
	ros::ServiceClient SetLeftRegisters_cli, SetRightRegisters_cli;
		camera1394::SetCameraRegisters SetRegister_msg;

	setStrobe()
	{
		/*--------- Initialize ROS Communication ------------- */
		HastShutDown_sub 		= n.subscribe("/hast/shutdown",  10, &setStrobe::nodeShutDown, this);
		SetLeftRegisters_cli 	= n.serviceClient<camera1394::SetCameraRegisters>("/left/camera/set_camera_registers", true);
		SetRightRegisters_cli 	= n.serviceClient<camera1394::SetCameraRegisters>("/right/camera/set_camera_registers", true);


		// value.push_back(2181038080); // This translates to the binary version of: 10000010 00000000 00000000 00000000 which can be generated via PGR's flycap program
		value.push_back(2182086656); // This translates to the binary version of: 10000010 00010000 00000000 00000000 which can be generated via PGR's flycap program
		SetRegister_msg.request.type = 0;
		SetRegister_msg.request.offset = 2096; // This corresponds to the hex address of the externaltrigger: 0x830
		SetRegister_msg.request.value = value;
		SetRegister_msg.request.mode = 0;
	}
	
	void nodeShutDown(const hast::flag::ConstPtr& ShutDown)
	{ 
		if(ShutDown->flag)
		{
			ROS_INFO("Shutting Down...");
			ros::shutdown();
		}
	}
};


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "setStrobe");
	setStrobe sS;

	for(int b = 1; b < 4; ++b)
	{		
		ROS_INFO("setStrobe: %d", b);
		sS.SetLeftRegisters_cli.call(sS.SetRegister_msg);
		sS.SetRightRegisters_cli.call(sS.SetRegister_msg);

		ros::spinOnce();
		ros::Duration(3).sleep(); // sleep for 'x' second(s).			
	}

	return 0;
}