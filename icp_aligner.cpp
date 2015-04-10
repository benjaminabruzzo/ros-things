// defaul libs
#include <iostream>


// Math Libs
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS libs
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

//pcl libs
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// int2string reconfigurator
namespace patch
{
template < typename T > std::string to_string( const T& n)
{
	std::ostringstream stm;
	stm << n;
	return stm.str();
}
}

// * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *
//                         Example converted to ROS-esq class
//        http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
// * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloudNoNans_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

class icp_aligner
{
private:
	// Test point cloud
	ros::NodeHandle n;
	ros::Subscriber NanoCloud_sub;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_preXform, xyzrgb_noNans;


	// Template point cloud
	std::string pcd_filename, ply_root, ply_file;

	// Aligning point clouds
	int cloudCounter, fitCounter;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;


	// Aligned point cloud
	float fitness_score;
	Eigen::Matrix4f final_transformation;

	sensor_msgs::PointCloud2 alignedcloud_rosmsg;
	ros::Publisher AlignedCloud_pub;

public:
	// string for argv[1] function call
	std::string plyfilename;

	icp_aligner()
	{
		cloudCounter = 0;
		fitCounter = 0;

		ROS_INFO("icp_aligner constructed...");
	}

	void startROS()
	{
		// Load the object templates specified in the object_templates.txt file
		ROS_INFO("argv[1] template : %s", plyfilename.c_str());
		ply_root = "/home/benjamin/r2_hydro/src/nasa_r2_vision/nasa_r2_vision_handrail/templates/";
		ply_file = ply_root + plyfilename;
		pcl::io::loadPLYFile (ply_file, *template_cloud);
		// template_cloud.loadInputPly(ply_root + plyfilename);

		// Test point cloud
		NanoCloud_sub = n.subscribe("/r2/right_leg/nano/points_xyzrgb", 1, &icp_aligner::pclBridge, this);
		AlignedCloud_pub = n.advertise<sensor_msgs::PointCloud2>("/r2/right_leg/nano/aligned_cloud", 1);
		// NanoCloud_sub = n.subscribe("/r2/" + leggedness + "_leg/nano/points_xyzrgb", 1, &icp_aligner::pclBridge, this);
	}


	void pclBridge(const sensor_msgs::PointCloud2ConstPtr& msg)
	{	// called each time a new point cloud is posted
		// ROS_INFO("pclBridge invoked.");
		fitness_score = 999;
		final_transformation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
		++cloudCounter;
		fromROSMsg(*msg, cloud_preXform);
		std::vector<int> indices;

		pcl::removeNaNFromPointCloud(cloud_preXform, xyzrgb_noNans, indices);
		pcl::PointXYZRGB tempPoint_RGB;
		pcl::PointXYZ tempPoint_XYZ;
		rgbCloudNoNans_ptr->points.clear();
		cloud_in->points.clear();


		for (int w = 0; w < xyzrgb_noNans.width; ++w)
		{
			tempPoint_RGB  = xyzrgb_noNans.at(w);
			rgbCloudNoNans_ptr->points.push_back(tempPoint_RGB);

			tempPoint_XYZ.x = tempPoint_RGB.x;
			tempPoint_XYZ.y = tempPoint_RGB.y;
			tempPoint_XYZ.z = tempPoint_RGB.z;
			if (tempPoint_XYZ.z < 1.0)
			{
				cloud_in->points.push_back(tempPoint_XYZ);
			}
		}
		// savePLY(cloud_in, "rail_end", cloudCounter);

		fitTemplate();
		visualizeICP();
	}

	void fitTemplate()
	{

		icp.setInputSource(template_cloud);
		icp.setInputTarget(cloud_in);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		fitness_score = icp.getFitnessScore();
		ROS_INFO("ICP converged with a fitness score of %6.16f ", fitness_score);
		// std::cout << "has converged:" << icp.hasConverged() << " score: " << fitness_score << "\n" << std::endl;
		// std::cout << icp.getFinalTransformation() << "\n\n\n" << std::endl;
	}

	void visualizeICP()
	{

		pcl::transformPointCloud (*template_cloud, *transformed_cloud, icp.getFinalTransformation());
		toROSMsg(*transformed_cloud, alignedcloud_rosmsg);
		alignedcloud_rosmsg.header.frame_id =  "/r2/right_leg/nano_optical_frame";
		AlignedCloud_pub.publish(alignedcloud_rosmsg);
	}

	void savePLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const char* name, int count)
	{
		std::string path = "/home/benjamin/Documents/plydata/";
		std::string filename = path + name + "_" + patch::to_string(count) + ".ply";
		pcl::io::savePLYFileASCII (filename.c_str(), *cloud);
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "handrail_icp_aligner");
	icp_aligner IA;
	IA.plyfilename = argv[1];
	IA.startROS();

	ros::spin();
	return 0;
}
