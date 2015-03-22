
//ROS Communications
#include <ros/ros.h>
	// Services
	#include <camera1394/SetCameraRegisters.h>


class setStrobe
{
private:	
	/*--------- ROS Communication Containers ------------- */
	ros::NodeHandle n;
	ros::Subscriber HastShutDown_sub;


public:
/*------- set camera register containers ------- */
	ros::ServiceClient SetLeftRegisters_cli, SetRightRegisters_cli;
	camera1394::SetCameraRegisters SetTriggerRegister_msg, SetShutterRegister_msg;
	std::vector<uint32_t> ext_trig_value, abs_shutter_value;

	setStrobe()
	{
		SetLeftRegisters_cli 	= n.serviceClient<camera1394::SetCameraRegisters>("/left/camera/set_camera_registers", true);
		SetRightRegisters_cli 	= n.serviceClient<camera1394::SetCameraRegisters>("/right/camera/set_camera_registers", true);
	}


	void setTriggerMode()
	{
		/*--------- Initialize ROS Communication ------------- */

		// value.push_back(2181038080); // This translates to the binary version of: 10000010 00000000 00000000 00000000 which can be generated via PGR's flycap program
		ext_trig_value.push_back(2182086656); // This translates to the binary version of: 10000010 00010000 00000000 00000000 which can be generated via PGR's flycap program
		SetTriggerRegister_msg.request.type = 0; //standard control
		SetTriggerRegister_msg.request.offset = 2096; // This corresponds to the hex address of the externaltrigger: 0x830 TRIGGER_MODE
		SetTriggerRegister_msg.request.value = ext_trig_value;
		SetTriggerRegister_msg.request.mode = 0;
		ROS_INFO("trigger mode set");
	}

	void setShutterSpeed()
	{
		//in PGR, the bit setting I have been usinf is 120 after unchecking the "absolute mode"
		// This translates to the binary version of: 00111011 01110101 10101001 00000000 which can be generated via PGR's flycap program
		// Hex :: Dec :: Bin = 3b 75 A9 00 :: 997566720 :: 0011 1011 0111 0101 1010 1001 0000 0000
		abs_shutter_value.push_back(997566720); // 3ms
		// abs_shutter_value.push_back(1028177936); // 49 ms
		
		// msg.request.type = 0; //standard control
		// msg.request.type = 1; // absolute control
		SetShutterRegister_msg.request.type = 0; // advanced control
		SetShutterRegister_msg.request.offset = 2328; // This corresponds to the hex address of the externaltrigger: 0x918 ABS_VAL_SHUTTER
		SetShutterRegister_msg.request.value = abs_shutter_value;
		SetShutterRegister_msg.request.mode = 0;
		ROS_INFO("shutter speed set");
		
	}
};


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "setStrobe");
	setStrobe sS;

	sS.setTriggerMode();
	sS.setShutterSpeed();

	for(int b = 1; b < 4; ++b)
	{		
		ROS_INFO("Call Register Services: %d", b);
		
		sS.SetLeftRegisters_cli.call(sS.SetTriggerRegister_msg);
		sS.SetLeftRegisters_cli.call(sS.SetShutterRegister_msg);

		sS.SetRightRegisters_cli.call(sS.SetTriggerRegister_msg);
		sS.SetRightRegisters_cli.call(sS.SetShutterRegister_msg);
		
		ros::spinOnce();
		ros::Duration(2).sleep(); // sleep for 'x' second(s).			
	}

	// for(int b = 1; b < 4; ++b)
	// {		
	// 	ROS_INFO("setStrobe: %d", b);
	// 	sS.SetLeftRegisters_cli.call(sS.SetRegister_msg);
	// 	sS.SetRightRegisters_cli.call(sS.SetRegister_msg);

	// 	ros::spinOnce();
	// 	ros::Duration(3).sleep(); // sleep for 'x' second(s).			
	// }

	return 0;
}