#include <iostream>
#include <optional>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include "gazebo_msgs/msg/link_states.hpp"


class Stitcher : public rclcpp::Node //Create a node for stitching 
{

public:
    Stitcher()
        : Node("stitcher")
        , toFrameRel("camera_link") //We need tf with respect to camera link 
        , fromFrameRel("camera_link_optical") //We need tf of camera_link_optical 
    {
        //Subscribe to /demo/link/states for getting the transformation of camera with respect to the world from gazebo 
        subscription_1 = this->create_subscription<gazebo_msgs::msg::LinkStates>( "/demo/link_states", 
                                                                        10, 
                                                                        std::bind(&Stitcher::topic_callback1, 
                                                                        this, std::placeholders::_1));

        //Subscribe to /objectPoints for getting Point Cloud data 
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( "/objectPoints", 
                                                                       10, 
                                                                       std::bind(&Stitcher::topic_callback, 
                                                                       this, std::placeholders::_1));

        //Publish the stitched point cloud to /stitchedObject topic for visualization in Rviz2
        stitched_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stitchedObject", 10);



        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // Creating tf2 listener
    }

private:

    geometry_msgs::msg::TransformStamped message1; 
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr subscription_1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stitched_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr old_stitched_point_cloud; //create a pointer to the current stitched point cloud 

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::optional<Eigen::Matrix4d> reference_transform; //Initial transform of camera to the world when the camera is launched in gazebo 
    std::optional<Eigen::Matrix4d> old_transform_camera; //old camera transformation to the world frame 

    std::string toFrameRel;
    std::string fromFrameRel;

    // Helper function to convert sensor_msg::PointCloud2 to PointXYZ data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        sensorMsgToXYZCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // Convert sensor_msg::PointCloud2 to pcl::PCLPointCloud2
        pcl::PCLPointCloud2::Ptr msg_cloudPtr(new pcl::PCLPointCloud2);

        pcl_conversions::toPCL(*msg, *msg_cloudPtr);

        // Convert pcl::PCLPointCloud2 to PointXYZ data type
        pcl::PointCloud<pcl::PointXYZ>::Ptr XYZcloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*msg_cloudPtr,*XYZcloudPtr);
        return XYZcloudPtr;
    }

    // Helper function to convert PointXYZ data type to sensor_msgs::msg::PointCloud2
    std::shared_ptr<sensor_msgs::msg::PointCloud2>
        XYZCloudToSensorMsg(const pcl::PointCloud<pcl::PointXYZ>& cloud) const
    {

        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::PCLPointCloud2::Ptr pcl2_cloud(new pcl::PCLPointCloud2);

        pcl::toPCLPointCloud2(cloud,*pcl2_cloud);

        pcl_conversions::fromPCL(*pcl2_cloud, *cloud_msg);

        cloud_msg->header.frame_id = "world";

        return cloud_msg;
    }

    //Get transform of camera from a /demo/link_states message 
    std::optional<Eigen::Matrix4d> get_transform_camera(geometry_msgs::msg::TransformStamped message1) const
    {
        auto eigen_transform = tf2::transformToEigen(message1);
        Eigen::Matrix4d data = eigen_transform.matrix();
        return {data}; //matrix data is extracted from the message and returned as std::optional<Eigen::Matrix4d>
    }

    std::optional<Eigen::Matrix4d> get_transform() const
    {
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between camera_link_optical frame and camera_link frame
        try 
        {
            t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) 
        {
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return {};
        }

        auto eigen_transform = tf2::transformToEigen(t);
        Eigen::Matrix4d data = eigen_transform.matrix();
        return {data}; //return the transform as a matrix as std::optional<Eigen::Matrix4d>
    }

    template<typename T>
    bool is_tolerant(const T& val, double threshold, double eps = 0.1) const //checks whether the given value is in a certain range within the threshold input value 
    {
        if (threshold - eps <= val && val <= threshold + eps)
        {
            return true;
        }

        return false;
    }

    struct TStruct //struct for axis angle representation 
    {
        Eigen::Vector3d trans;
        Eigen::Vector3d axis;
        double angle;
    };

    TStruct convertTtoAA(const Eigen::Matrix4d& T) const //Converts a transformation matrix to axis angle representation and returns the translation vector,rotation matrix and axis angle
    {

        Eigen::Matrix3d rot = T(Eigen::seqN(0,3),Eigen::seqN(0,3));
        Eigen::Vector3d trans = T(Eigen::seqN(0,3),3);

        auto axisAngle = Eigen::AngleAxisd(rot);
        const auto& axis = axisAngle.axis().normalized();
        const auto& angle = axisAngle.angle();
        
        return {trans, axis, angle};

    }

    bool is_transformation_changed( const Eigen::Matrix4d& newT, 
                                    double eps = 0.2) const     //Checks if transformation of camera is changed when the user moves it manually in gazebo. Returns true if changed
    {

        if (!reference_transform) //if reference transform is not yet initialized, it returns false 
        {
            return false;
        }

        auto [newTrans, newAxis, newAngle] = convertTtoAA(newT); //converting new transform to axis angle 
        auto [oldTrans, oldAxis, oldAngle] = convertTtoAA(*old_transform_camera); //converting old transform to axis angle 

        // checking how much the new transform deviates from the old transform 
        auto axis_range = newAxis.dot(oldAxis);
        auto transDist = (newTrans - oldTrans).squaredNorm();
        if (is_tolerant(axis_range, 0, eps)
            && is_tolerant(newAngle, oldAngle, eps)
            && is_tolerant(transDist,0, eps))
        {
            return false;
        }

        return true;
    }

    void topic_callback1(const gazebo_msgs::msg::LinkStates msg1) //Get the message of /demo/link_states 
    {
        sleep(10); //delay of 10 seconds between processing messages for better point cloud stitching. We need to give more time for letting the camera move to a location before taking transform
        const std::vector<std::string> &names = msg1.name;
        std::string i = names[3]; //as there are different objects like camera, object, table in gazebo we need to choode camera_link as our desired transformation 
        std::cout<<"i:"<<i<<std::endl; //print to terminal to verify 
        const geometry_msgs::msg::Pose camera_pose = msg1.pose[3]; //msg1.pose[3] needs to be used fof coke can and msg1.pose[2] for other objects 
        //input camera pose information into a new message 
        message1.transform.translation.x = camera_pose.position.x;
        message1.transform.translation.y = camera_pose.position.y;
        message1.transform.translation.z = camera_pose.position.z;
        message1.transform.rotation.x = camera_pose.orientation.x;
        message1.transform.rotation.y = camera_pose.orientation.y;
        message1.transform.rotation.z = camera_pose.orientation.z;
        message1.transform.rotation.w = camera_pose.orientation.w;
        
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        //print tf b/w camera and world in gazebo to the terminal for verification 
        std::cout<<"message1.transform.translation.x:"<<message1.transform.translation.x<<std::endl;
        std::cout<<"message1.transform.translation.y:"<<message1.transform.translation.y<<std::endl;
        std::cout<<"message1.transform.translation.z:"<<message1.transform.translation.z<<std::endl;
        std::cout<<"message1.transform.rotation.x:"<<message1.transform.rotation.x<<std::endl;
        std::cout<<"message1.transform.rotation.y:"<<message1.transform.rotation.y<<std::endl;
        std::cout<<"message1.transform.rotation.z:"<<message1.transform.rotation.z<<std::endl;
        std::cout<<"message1.transform.rotation.w:"<<message1.transform.rotation.w<<std::endl;
        
        // 1. get transformation of camera from message 1 of topic_callback1
        auto latest_transform_camera = get_transform_camera(message1);
        auto latest_transform_optional = get_transform(); //Get tf between camera_link_optical and camera_link because point cloud is initially in camera_link_optical frame as per camera.urdf file
        
        if (!latest_transform_optional)
        {
            return;
        }
        //using new variables to store latest_transform_optional and latest_transform_camera
        auto latest_transform = *latest_transform_optional;
        auto camera_transform = *latest_transform_camera;
        RCLCPP_INFO_STREAM(get_logger(), "got transform: " << latest_transform);

        // 2. check if transform is initialized
        if (!reference_transform)
        {
            RCLCPP_INFO_STREAM(get_logger(), "transform was not initialized");
            reference_transform = latest_transform; //reference_transform is the latest transform 
            old_transform_camera = latest_transform_camera; //latest camera transform is sent to old_transform_camera
            //The current point cloud (initial cloud) is sent to old_stitched_point cloud as the initial stitched point_cloud
            pcl::PointCloud<pcl::PointXYZ> pre_old_st_cloud;
            pcl::PointCloud<pcl::PointXYZ> old_cloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
            auto first_cloud = sensorMsgToXYZCloud(msg);
            pcl::transformPointCloud(*first_cloud,pre_old_st_cloud, latest_transform);
            pcl::transformPointCloud(pre_old_st_cloud,old_cloud, camera_transform);
            *cloudPTR = old_cloud;
            old_stitched_point_cloud = cloudPTR;
            auto first_out_msg = XYZCloudToSensorMsg(*old_stitched_point_cloud);
            stitched_pub_->publish(*first_out_msg); //stitched (initial) point cloud is published to /stitchedObject topic for visualization in Rviz
            return;
        }
        
        // 3. check if transform has changed
        if (!is_transformation_changed(camera_transform))
        {
            RCLCPP_INFO_STREAM(get_logger(), "transform unchanged, jovial mode"); //statement printed to terminal indicating that camera transform is not changed in gazebo 
            stitched_pub_->publish(*msg); //keep publishing the current input point cloud as stitched point cloud until transform is changed 
            return;
        }
        RCLCPP_INFO_STREAM(get_logger(), "transform has changed, SERIOUS MODE -_-"); //statement printed to terminal indicating that camera transform is changed in gazebo 

        // 4. change the callback point cloud for point cloud processing 
        auto input_cloud = sensorMsgToXYZCloud(msg);
        
        pcl::PointCloud<pcl::PointXYZ> pre_transformed_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        pcl::PointCloud<pcl::PointXYZ> t_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tcloudPTR(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*input_cloud,pre_transformed_cloud, latest_transform); //change input cloud from camera_link_optical to camera_link frame 
        pcl::transformPointCloud(pre_transformed_cloud,t_cloud, camera_transform); //change the cloud in camera_link_frame to world_frame 
        *tcloudPTR = t_cloud;
        transformed_cloud = tcloudPTR; //final input cloud in world frame 

        // 5. align the transformed cloud with the old stitched point cloud using ICP 
        int iterations = 60;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations (iterations);
        icp.setInputSource (transformed_cloud);
        icp.setInputTarget (old_stitched_point_cloud);
        icp.align (*transformed_cloud);

        // 6. Stitch the point clouds 
        *old_stitched_point_cloud += *transformed_cloud; //add the aligned point cloud to the old_stitched_cloud which stitches both the point cloud together 
        RCLCPP_INFO_STREAM(get_logger(), "stitched cloud, it's gonna rain");

        // 7. publish the stitched point cloud to /stitchedObject topic for visualization in Rviz
        auto out_msg = XYZCloudToSensorMsg(*old_stitched_point_cloud);
        stitched_pub_->publish(*out_msg);
        old_transform_camera = latest_transform_camera;
        return;

    }

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stitcher>());
  rclcpp::shutdown();
  return 0;
}

