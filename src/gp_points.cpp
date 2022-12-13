#include <iostream>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h> // for compute3DCentroid
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <Eigen/Core>
#include <math.h>
#include <array>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vtkSmartPointer.h>


using std::placeholders::_1;
using Eigen::placeholders::all;

class GraspPoints : public rclcpp::Node
{

public:
    GraspPoints()
        : Node("gp_points")
    {

        //subscribe to the stitched point cloud of /stitchedObject topic for further processing to get grasp points 
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/stitchedObject", 
                                                                       10, 
                                                                       std::bind(&GraspPoints::topic_callback, 
                                                                       this, std::placeholders::_1));

        //stitched point cloud is published to stitched_cloud_sor topic after statistical outlier removal 
        stitched_pub_sor = this->create_publisher<sensor_msgs::msg::PointCloud2>("stitched_cloud_sor", 10);
        //grasp points for the stitched point cloud are published to stitched_grasp_points topic for visualization in Rviz 
        stitched_gp_point_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("stitched_grasp_points", 10);
        //centroid of stitched point cloud is published to StitchedCentroidPoint topic
        stitched_centroid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("StitchedCentroidPoint", 10);

    }

private:
 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stitched_pub_sor;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stitched_gp_point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stitched_centroid_pub_;

   bool isCollinear(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2, double eps = 0.1) const
  {
    // Normalize the functions before calculation dot product
    const auto& vec1_norm = vec1.normalized();
    const auto& vec2_norm = vec2.normalized();

    // derived dot product
    auto dot_product_val = vec1_norm.dot(vec2_norm);

    // making sure that dot product value is around -1 (angle as 180)
    //  since there can be numerical accuracies not giving us perfect -1
    if ((-1 - eps <= dot_product_val) && (dot_product_val <= -1 + eps))
    {
      return true;
    } 
    return false;
  }

   struct PointPairHasher
  {
    template <typename T1, typename T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

 /* Function to calculate the best grasp contact points in the stitched point cloud */
 std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> 
    getBestGraspContactPair(const Eigen::Matrix3Xf& normals, 
                            const Eigen::Matrix3Xf& contact_points, 
                            const Eigen::Vector3f& centroid) const
  {
    double min_dist_centr=100;
    int grasp_pair_index = 0;
    // Initialize the containers to store the contact pairs
    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> cp_pairs;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grasp_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr best_grasp_points (new pcl::PointCloud<pcl::PointXYZRGB>);

    // ideal best grasp angle value
    double best_grasp_angle = 0;

    // define angle threshold
    float angle_threshold_degree = 10;
    float angle_threshold = angle_threshold_degree * (M_PI / 180);

    // Matrix of Vectors between contact points and centroid
    auto CX0 = contact_points.colwise() - centroid;

    /* Check against all the contact points iteratively */
    double best_angle_clearance = 10;
    for (size_t i=0; i < normals.cols(); i++)
    {
      // 1st contact point that is considered
      const auto& C1 = contact_points(all, i);

      // 1st contact point normal
      const auto& C1N = normals(all, i);
      
      // vector between 1st contact point and centroid
      const auto& C10 = CX0(all, i);

      for (size_t j=0; j<normals.cols(); j++)
      {
        // exclude comparing between same contact points
        if (i==j)
        {
          continue;
        }

        // 2nd contact point that is considered
        const auto& C2 = contact_points(all, j);

        // vector between 2nd contact point and centroid
        const auto& C20 = CX0(all, j);

        // check if vectors between 
        //      (1st contact point and centroid) 
        //  and (2nd contact point and centroid) 
        //  are collinear
        auto is_collinear = isCollinear(C10,C20);

        // if they are collinear check if they satisfy the necessary 
        //  force vector grasp formulation
        if (1==1) //Used instead of isCollinear because we are not considering the collinearity of contact points with the centroid to calculate the best grasp 
        {

          // 1st contact point normal
          const auto& C2N = normals(all, j);

          // vector between contact points
          auto C1C2 = (C1-C2).normalized();

          // calculate angles between contact points and corresponding normals
          auto angle1 = acos(C1N.dot(C1C2));
          auto angle2 = acos(C2N.dot(C1C2));
          double grasp_angle = angle1 + angle2;
          
          if((std::abs(M_PI/2-angle1)>0.9)&&(std::abs(angle2-M_PI/2)>0.9)) { //in order to not lose information regarding grasp provided by each angle, otherwise the points with normals facing the viewer might also be selected 
            // Check if corresponding grasp angle is falling within the threshold
            if(M_PI - angle_threshold < grasp_angle && grasp_angle < M_PI + angle_threshold)
            {
              // Check if the corresponding grasp angle is better
              // than previous candidate grasp angles
              if(std::abs(grasp_angle - M_PI)<best_angle_clearance) //If the grasp_angle is the closest to 180 degrees angle, then we add it to our grasp_point_cloud
              { 
                double dist1 = sqrt((centroid(0) - C1(0))*(centroid(0) - C1(0))  +  (centroid(1) - C1(1))*(centroid(1) - C1(1))  +  (centroid(2) - C1(2))*(centroid(2) - C1(2)));
                double dist2 = sqrt((centroid(0) - C2(0))*(centroid(0) - C2(0))  +  (centroid(1) - C2(1))*(centroid(1) - C2(1))  +  (centroid(2) - C2(2))*(centroid(2) - C2(2)));
                if(dist1+dist2<min_dist_centr){ //distance to the centroid is minimized to select the best grasp points 
                  min_dist_centr = dist1+dist2;
                  grasp_point_cloud->push_back(pcl::PointXYZRGB(C1(0),C1(1),C1(2)));
                  grasp_point_cloud->push_back(pcl::PointXYZRGB(C2(0),C2(1),C2(2)));
                  cp_pairs.push_back({C1, C2});
                  best_angle_clearance = std::abs(grasp_angle - M_PI); //the best difference from 180 degrees is updated so that it will be used for the next set of contact points 
                  best_grasp_angle = grasp_angle; //best grasp angle is updated
                }
             }
            }
          }
        }
      }
    }
    
    int k = 0;
    for (k = 0; k < grasp_point_cloud->points.size (); k++){

    }
     std::cout<<"min_dist_centr "<<min_dist_centr<<std::endl;
    std::cout<<"best grasp angle in radians: "<<best_grasp_angle<<std::endl;
    std::cout<<"k= "<<grasp_point_cloud->points.size ()<<std::endl;
    //Penultimate pair of points of the grasp_point_cloud are the best grasp points as per the above code so taking them and pushing them into best_grasp_points for visualization 
    best_grasp_points->push_back((*grasp_point_cloud)[k-4]);
    best_grasp_points->push_back((*grasp_point_cloud)[k-3]);

    //Changing the color of the points, one point is red while the other is green to distinguish 
    
    (*best_grasp_points)[0].r = 255;
    (*best_grasp_points)[0].g = 0;
    (*best_grasp_points)[0].b = 0;

    (*best_grasp_points)[1].r = 0;
    (*best_grasp_points)[1].g = 255;
    (*best_grasp_points)[1].b = 0;


    auto output_grasp_points = new sensor_msgs::msg::PointCloud2;
    pcl::PCLPointCloud2::Ptr cloud_grasp_points(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*best_grasp_points,*cloud_grasp_points);
    pcl_conversions::fromPCL(*cloud_grasp_points, *output_grasp_points);
    output_grasp_points->header.frame_id = "world";
    stitched_gp_point_pub_->publish(*output_grasp_points); //publishing best_grasp_points

    return cp_pairs;
  }
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
      pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2); // container for pcl::PCLPointCloud
      pcl_conversions::toPCL(*msg, *cloudPtr); // convert to PCLPointCloud2 data type

      // 1. Downsampling 
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.008f, 0.008f, 0.008f);
      sor.filter (*cloudPtr);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloudPtr,*cloud_filtered);

      //2. Removing outliers using a StatisticalOutlierRemoval filter
      pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_1;
      sor_1.setInputCloud (cloud_filtered);
      sor_1.setMeanK (10);
      sor_1.setStddevMulThresh (1.0);
      sor_1.filter (*XYZcloudPtr);

      auto stitched_output = new sensor_msgs::msg::PointCloud2;
      pcl::PCLPointCloud2::Ptr cloud_stitched(new pcl::PCLPointCloud2); 
      pcl::toPCLPointCloud2(*XYZcloudPtr,*cloud_stitched);
      pcl_conversions::fromPCL(*cloud_stitched, *stitched_output); 
      stitched_pub_sor->publish(*stitched_output);  

      // 3. NORMAL ESTIMATION
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (XYZcloudPtr);

      // Create an empty KDTree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);
      ne.useSensorOriginAsViewPoint();

      // 4 Compute the features
      ne.compute (*cloud_normals);
      RCLCPP_INFO_STREAM(this->get_logger(), "# of normals: " << cloud_normals->size ());

      // 5. CENTROID
      // 16-bytes aligned placeholder for the XYZ centroid 
      Eigen::Vector4f xyz_centroid;
      
      
      // 6. Estimate the XYZ centroid
      pcl::compute3DCentroid (*XYZcloudPtr, xyz_centroid);
      auto centroid_point = new sensor_msgs::msg::PointCloud2;                   
      pcl::PCLPointCloud2::Ptr centroid_cloud_point(new pcl::PCLPointCloud2);  
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
      centroid_point_cloud->push_back(pcl::PointXYZRGB(xyz_centroid(0),xyz_centroid(1),xyz_centroid(2))); 
      (*centroid_point_cloud)[0].r = 40;
      (*centroid_point_cloud)[0].g = 80;
      (*centroid_point_cloud)[0].b = 250;
      pcl::toPCLPointCloud2(*centroid_point_cloud,*centroid_cloud_point);
      pcl_conversions::fromPCL(*centroid_cloud_point, *centroid_point);  
      stitched_centroid_pub_->publish(*centroid_point);
      RCLCPP_INFO_STREAM(this->get_logger(), "centroid: "<<xyz_centroid(0)<<","<<xyz_centroid(1)<<","<<xyz_centroid(2));

      pcl::PointXYZ centroidXYZ(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

      // 7. FLIPPING NORMALS ACCORIDNG TO CENTROID
      Eigen::Matrix3Xf normal_vector_matrix(3,cloud_normals->size());
      Eigen::Matrix3Xf point_cloud(3,cloud_normals->size());
      for(size_t i = 0; i < cloud_normals->size(); i++) 
      {
        Eigen::Vector3f normal = cloud_normals->at(i).getNormalVector4fMap().head(3);
        Eigen::Vector3f normal_dup = cloud_normals->at(i).getNormalVector4fMap().head(3);
        
        pcl::flipNormalTowardsViewpoint(XYZcloudPtr->at(i), xyz_centroid[0], xyz_centroid[1], xyz_centroid[2], normal);
        normal_vector_matrix(0,i) = normal[0];
        normal_vector_matrix(1,i) = normal[1];
        normal_vector_matrix(2,i) = normal[2];
        

        //const auto& pointMatrix = XYZcloud_filtered->at(i);
        point_cloud(0,i) = XYZcloudPtr->points[i].x;
        point_cloud(1,i) = XYZcloudPtr->points[i].y;
        point_cloud(2,i) = XYZcloudPtr->points[i].z;
      }

      const auto& data = getBestGraspContactPair(normal_vector_matrix, point_cloud,xyz_centroid.head(3)); //FDunction is called to get the best grasp contact points
      RCLCPP_INFO_STREAM(this->get_logger(), "Size of data: " << data.size());
      
    };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspPoints>());
  rclcpp::shutdown();
  return 0;
}