// ROS libs
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <nasa_r2_vision_handrail/HandrailEndPose.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// PCL libs
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>


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


class FeatureCloud
{
public:
  // A bit of shorthand
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

  FeatureCloud () :
    search_method_xyz_ (new SearchMethod),
    normal_radius_ (0.02f),
    feature_radius_ (0.02f)
  {}

  ~FeatureCloud () {}

  // Process the given cloud
  void setInputCloud (PointCloud::Ptr xyz)
  {
    xyz_ = xyz;
    processInput ();
  }

  // Load and process the cloud in the given PCD file
  void loadInputCloud (const std::string &pcd_file)
  {
    xyz_ = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPCDFile (pcd_file, *xyz_);
    processInput ();
  }

  void loadInputPly (const std::string &ply_file)
  {
    xyz_ = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPLYFile (ply_file, *xyz_);
    processInput ();
  }


  // Get a pointer to the cloud 3D points
  PointCloud::Ptr getPointCloud () const
  {
    return (xyz_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr getSurfaceNormals () const
  {
    return (normals_);
  }

  // Get a pointer to the cloud of feature descriptors
  LocalFeatures::Ptr getLocalFeatures () const
  {
    return (features_);
  }

protected:
  // Compute the surface normals and local features
  void
  processInput ()
  {
    computeSurfaceNormals ();
    computeLocalFeatures ();
  }

  // Compute the surface normals
  void
  computeSurfaceNormals ()
  {
    normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud (xyz_);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);
  }

  // Compute the local feature descriptors
  void
  computeLocalFeatures ()
  {
    features_ = LocalFeatures::Ptr (new LocalFeatures);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (xyz_);
    fpfh_est.setInputNormals (normals_);
    fpfh_est.setSearchMethod (search_method_xyz_);
    fpfh_est.setRadiusSearch (feature_radius_);
    fpfh_est.compute (*features_);
  }

private:
  // Point cloud data
  PointCloud::Ptr xyz_;
  SurfaceNormals::Ptr normals_;
  LocalFeatures::Ptr features_;
  SearchMethod::Ptr search_method_xyz_;

  // Parameters
  float normal_radius_;
  float feature_radius_;
};

class TemplateAlignment
{
public:

  // A struct for storing alignment results
  struct Result
  {
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment () :
    min_sample_distance_ (0.05f),
    max_correspondence_distance_ (0.01f * 0.01f),
    nr_iterations_ (500)
  {
    // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance (min_sample_distance_);
    sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
    sac_ia_.setMaximumIterations (nr_iterations_);
  }

  ~TemplateAlignment () {}

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud (FeatureCloud &target_cloud)
  {
    target_ = target_cloud;
    sac_ia_.setInputTarget (target_cloud.getPointCloud ());
    sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
  }

  // Add the given cloud to the list of template clouds
  void addTemplateCloud (FeatureCloud &template_cloud)
  {
    templates_.push_back (template_cloud);
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
  {
    sac_ia_.setInputCloud (template_cloud.getPointCloud ());
    sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align (registration_output);

    result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
    result.final_transformation = sac_ia_.getFinalTransformation ();
  }

  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
  {
    results.resize (templates_.size ());
    for (size_t i = 0; i < templates_.size (); ++i)
    {
      align (templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int findBestAlignment (TemplateAlignment::Result &result)
  {
    // Align all of the templates to the target cloud
    std::vector<Result, Eigen::aligned_allocator<Result> > results;
    alignAll (results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity ();
    int best_template = 0;
    for (size_t i = 0; i < results.size (); ++i)
    {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score)
      {
        lowest_score = r.fitness_score;
        best_template = (int) i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }

private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;
};

// * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *
//                                   Example converted to ROS-esq class
//              http://pointclouds.org/documentation/tutorials/template_alignment.php
// * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloudNoNans_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

class TemplateAligner
{
private:
  // Template pointclouds
  std::vector<FeatureCloud> object_templates; // a vector of feature clouds (can probably be replaced by a single feature cloud to match the handrail end)
  std::string pcd_filename, pcd_path;
  FeatureCloud template_cloud;

  // Test point cloud
  ros::NodeHandle n;
  ros::Subscriber NanoCloud_sub;
  float depth_limit;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_preXform, xyzrgb_noNans;
  int cloudCounter, fitCounter;

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index;
  // FeatureCloud &best_template;

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

  sensor_msgs::PointCloud2 alignedcloud_rosmsg;
  ros::Publisher AlignedCloud_pub;


public:
  TemplateAligner()
  {
    // * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *
    //                                    Template pointclouds
    // * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *
    // Load the object templates specified in the object_templates.txt file
    object_templates.resize (0);
    pcd_path = "/home/benjamin/r2_hydro/src/nasa_r2_vision/nasa_r2_vision_handrail/templates/";
    pcd_filename = "rail_end_5y.ply";
    ROS_INFO("Matching tempplate : %s", pcd_filename.c_str());
    template_cloud.loadInputPly(pcd_path+pcd_filename);
    object_templates.push_back (template_cloud);

    // * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *
    //                                    Test point cloud
    // * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *  * * *
    NanoCloud_sub = n.subscribe("/r2/right_leg/nano/points_xyzrgb", 1, &TemplateAligner::pclBridge, this);
    AlignedCloud_pub = n.advertise<sensor_msgs::PointCloud2>("/r2/right_leg/nano/aligned_cloud", 1);
    // NanoCloud_sub = n.subscribe("/r2/" + leggedness + "_leg/nano/points_xyzrgb", 1, &TemplateAligner::pclBridge, this);
    cloudCounter = 0;
    fitCounter = 0;
    ROS_INFO("handrail_template_aligner constructed...");
  }


  void pclBridge(const sensor_msgs::PointCloud2ConstPtr& msg)
  { // called each time a new point cloud is posted
    // ROS_INFO("pclBridge invoked.");
    ++cloudCounter;
    fromROSMsg(*msg, cloud_preXform);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(cloud_preXform, xyzrgb_noNans, indices);
    pcl::PointXYZRGB tempPoint_RGB;
    pcl::PointXYZ tempPoint_XYZ;
    rgbCloudNoNans_ptr->points.clear();
    cloud->points.clear();


    for (int w = 0; w < xyzrgb_noNans.width; ++w)
    {
      tempPoint_RGB  = xyzrgb_noNans.at(w);
      rgbCloudNoNans_ptr->points.push_back(tempPoint_RGB);

      tempPoint_XYZ.x = tempPoint_RGB.x;
      tempPoint_XYZ.y = tempPoint_RGB.y;
      tempPoint_XYZ.z = tempPoint_RGB.z;
      if (tempPoint_XYZ.z < 1.0)
      {
        cloud->points.push_back(tempPoint_XYZ);
      }
    }
    // savePLY(cloud, "rail_end_raw", cloudCounter);

    fitTemplates();
  }


  void fitTemplates()
  {
    // ROS_INFO("fitTemplates invoked.");
    // Assign to the target FeatureCloud
    target_cloud.setInputCloud (cloud);

    // Set the TemplateAlignment inputs
    TemplateAlignment template_align;
    for (size_t i = 0; i < object_templates.size (); ++i)
    {
      template_align.addTemplateCloud (object_templates[i]);
    }
    template_align.setTargetCloud (target_cloud);

    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment (best_alignment);
    const FeatureCloud &best_template = object_templates[best_index];

    // Print the alignment fitness score (values less than 0.00002 are good)
    ROS_INFO("Fitness[%d] score: %f", ++fitCounter, best_alignment.fitness_score);

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

    // ROS_INFO("*** ***  ***  ***  ***  ***  ***  ***  *** ");
    // ROS_INFO("    | %6.3f %6.3f %6.3f | ", rotation (0, 0), rotation (0, 1), rotation (0, 2));
    // ROS_INFO("R = | %6.3f %6.3f %6.3f | ", rotation (1, 0), rotation (1, 1), rotation (1, 2));
    // ROS_INFO("    | %6.3f %6.3f %6.3f | ", rotation (2, 0), rotation (2, 1), rotation (2, 2));
    // ROS_INFO("*** ***  ***  ***  ***  ***  ***  ***  *** ");
    // ROS_INFO("t = < %0.3f, %0.3f, %0.3f >", translation (0), translation (1), translation (2));

    // Save the aligned template for visualization
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);

    toROSMsg(transformed_cloud, alignedcloud_rosmsg);
    alignedcloud_rosmsg.header.frame_id =  "/r2/right_leg/nano_optical_frame";
    AlignedCloud_pub.publish(alignedcloud_rosmsg);

  }

  void savePLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const char* name, int num)
  {
    std::string path = "/home/benjamin/Documents/plydata/";
    std::string filename = path + name + "_" + patch::to_string(num) + ".ply";
    pcl::io::savePLYFileASCII (filename.c_str(), *cloud);
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "handrail_template_aligner");
  TemplateAligner TA;

  ros::spin();
  return 0;
}
