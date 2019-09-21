
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cartographer/common/time.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/probability_grid.h"
#include "cartographer/mapping/probability_grid_range_data_inserter_2d.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/probability_values.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <boost/thread.hpp>
#include "cartographer/location/laser_location.h"
#include <grpcpp/grpcpp.h>
#include "cartographer/cloud/map_builder_server.h"
using namespace cartographer;
using namespace mapping;
using namespace common;
using namespace sensor;
using namespace cartographer::mapping::scan_matching;


// We use SDL_image to load the image from disk
#include <SDL/SDL_image.h>
using namespace std;
//话题回调函数


 float kMinScore = 0.1f;
 cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
 CreateFastCorrelativeScanMatcherTestOptions2D(
     const int branch_and_bound_depth) {
   auto parameter_dictionary =
       common::MakeDictionary(R"text(
       return {
          linear_search_window = 3.,
          angular_search_window = 1.,
          branch_and_bound_depth = )text" +
                              std::to_string(branch_and_bound_depth) + "}");
   return CreateFastCorrelativeScanMatcherOptions2D(parameter_dictionary.get());
 }
std::unique_ptr<cartographer::location::Laserlocation>  location_;
std::unique_ptr<cartographer::cloud::MapBuilderServer>  map_server_;
sensor_msgs::PointCloud2 PreparePointCloud2Message(
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}
common::Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::cartographer::common::FromUniversal(
      (time.sec +
       ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}
sensor_msgs::PointCloud2 ToPointCloud2Message(
   const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud) {
  auto msg = PreparePointCloud2Message(frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const cartographer::sensor::TimedRangefinderPoint& point : point_cloud) {
    stream.next(point.position.x());
    stream.next(point.position.y());
    stream.next(point.position.z());
    stream.next(1);
  }
  return msg;
}
Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}
Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}
std::unique_ptr<sensor::ImuData> ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const common::Time time = FromRos(msg->header.stamp);
//  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
//      time, CheckNoLeadingSlash(msg->header.frame_id));
//  if (sensor_to_tracking == nullptr) {
//    return nullptr;
//  }
//  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
//      << "The IMU frame must be colocated with the tracking frame. "
//         "Transforming linear acceleration into the tracking frame will "
//         "otherwise be imprecise.";sensor_to_tracking->rotation() *
  return absl::make_unique<sensor::ImuData>(sensor::ImuData{
      time,  ToEigen(msg->linear_acceleration),
       ToEigen(msg->angular_velocity)});
}

transform::Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return transform::Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}
transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose) {
  return transform::Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}
transform::Rigid3d ToRigid3d_odom(const geometry_msgs::Pose& pose) {

    const Eigen::AngleAxisd rotation(M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q(rotation);
      Eigen::Vector3d temp(pose.position.x, pose.position.y, pose.position.z);

    Eigen::Vector3d point=q *temp ;
    Eigen::Quaterniond anguler_(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                pose.orientation.z);
    Eigen::Quaterniond a_=anguler_*q;
    auto euler=anguler_.toRotationMatrix().eulerAngles(0, 1, 2);
    const Eigen::AngleAxisd rotation_sub(M_PI-euler[2], Eigen::Vector3d::UnitZ());
     Eigen::Quaterniond q_sub(rotation_sub);
    transform::Rigid3d  out(point, q_sub);
    return out;
}

std::unique_ptr<sensor::OdometryData> ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  const common::Time time = FromRos(msg->header.stamp);
//  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
//      time, CheckNoLeadingSlash(msg->child_frame_id));
//  if (sensor_to_tracking == nullptr) {
//    return nullptr;
//  }
  return absl::make_unique<sensor::OdometryData>(
      sensor::OdometryData{
          time, ToRigid3d_odom(msg->pose.pose)});
}
uint8 ProbabilityToColor(float probability_from_grid) {
  const float probability = probability_from_grid;
  return ::cartographer::common::RoundToInt(
      255 * ((probability - mapping::kMinProbability) /
             (mapping::kMaxProbability - mapping::kMinProbability)));
}
class nodeslam
{
public:
    nodeslam();
    ~nodeslam();
    void ScanCallback(const sensor_msgs::MultiEchoLaserScan::ConstPtr& scan);
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     void IMuCallback(const sensor_msgs::Imu::ConstPtr& imu);
    void publishLoop(double transform_publish_period);
    void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg);
    void publishMap(double transform_publish_period);

private:
     ros::NodeHandle n;	//节点句柄实例化
     ros::Subscriber sublaser;
      ros::Subscriber sublaser_scan;
     ros::Subscriber subimu;
      ros::Subscriber subodometry;
     ros::Publisher mapPub;
     ros::Publisher scan_pub ;
      ros::Publisher point_pub ;
      ros::Publisher path_pub ;
      nav_msgs::Path path;
     transform::Rigid2d pose_estimate;
     boost::thread* transform_thread_;
      boost::thread* pub_map_thread;
      boost::thread* grpc_thread;
      boost::mutex map_to_odom_mutex_;
       tf::TransformBroadcaster broadcaster;
        transform::Rigid2d pose_estimate_temp1;
       bool first_in;
       int count;

};
nodeslam::nodeslam()
{
    mapPub=n.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);
   // sublaser_scan=n.subscribe<sensor_msgs::LaserScan>("scan",1500,&nodeslam::ScanCallback,this);
  //  subimu=n.subscribe<sensor_msgs::Imu>("imu",1000,&nodeslam::IMuCallback,this);
   // subodometry=n.subscribe<nav_msgs::Odometry>("odom",1000,&nodeslam::OdomCallback,this);
    path_pub=n.advertise<nav_msgs::Path>("trajectory",1, true);
    nav_msgs::OccupancyGrid rosMap;
    rosMap.info.resolution = map_server_->location_->probability_grid_->limits().resolution();
    rosMap.info.origin.position.x = -map_server_->location_->probability_grid_->limits().max()[0];
    rosMap.info.origin.position.y = -map_server_->location_->probability_grid_->limits().max()[1];
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;
      mapping::CellLimits cell_limits;
      Eigen::Array2i offset;
    map_server_->location_->probability_grid_->ComputeCroppedLimits(&offset, &cell_limits);

    rosMap.info.width = map_server_->location_->probability_grid_->limits().cell_limits().num_y_cells;
    rosMap.info.height = map_server_->location_->probability_grid_->limits().cell_limits().num_x_cells;
    rosMap.data.reserve(rosMap.info.width * rosMap.info.height);
    int height=rosMap.info.height;
    int width=rosMap.info.width;
     for (int x = height-1; x >=0; --x){
        for (int y = width-1; y >=0; --y)
       // for (int y = 0; y <= width-1; ++y)
        {
          const Eigen::Array2i index(x,y);
           constexpr int kUnknownValue = -1;
         const   int value =
                 map_server_->location_->probability_grid_->IsKnown(index)
                     ? RoundToInt(( ProbabilityToColor(map_server_->location_->probability_grid_->GetProbability(index)) / 255.) * 100.)
                     : kUnknownValue;
        rosMap.data.push_back(value);
       }
     }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";
    path.header.stamp=ros::Time::now();
    path.header.frame_id="path";

  //  mapPub.publish(rosMap);
    first_in=true;
   // scan_pub = n.advertise<sensor_msgs::LaserScan>("scan1", 50);
   // point_pub=n.advertise<sensor_msgs::PointCloud2>("point",50);
    transform_thread_ = new boost::thread(boost::bind(&nodeslam::publishLoop, this, 0.001));
   pub_map_thread=new boost::thread(boost::bind(&nodeslam::publishMap, this, 0.001));

}
nodeslam::~nodeslam()
{
    if(transform_thread_)
     {
       transform_thread_->join();
       delete transform_thread_;
     }
}

void nodeslam::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor::TimedPointCloudData point_cloud;
    count++;
        for(int i=0;i<scan->ranges.size();i++)
        {
            point_cloud.time=FromRos(scan->header.stamp);
            double angle=scan->angle_min+scan->angle_increment*i;
            const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
              const Eigen::AngleAxisf rotation_180(M_PI, Eigen::Vector3f::UnitZ());// add rotation to  fit the
            Eigen::Vector3f point(rotation * rotation_180*(scan->ranges[i] * Eigen::Vector3f::UnitX()));
            float x=(float)(scan->ranges[i]*cos(angle));
            float y=(float)(scan->ranges[i]*sin(angle));
            Eigen::Vector3f point_1(x, y, 0.f);
          //  if(scan->ranges[i].echoes[0]<30.0)
            {
            //  point_cloud.push_back({point});
            point_cloud.ranges.push_back({point});
             }
        }
      //  cout<<"ros:"<<scan->header.stamp.sec<<endl;
      //   cout<<"carto:"<< point_cloud.time<<endl;

         map_server_->ProcessLocalSlamData(point_cloud);

         transform::Rigid3d pose_estimate =
             transform::Embed3D((map_server_->location_->current_pose));

            sensor::TimedPointCloud rotated_point= sensor::TransformTimedPointCloud(point_cloud.ranges, pose_estimate.cast<float>());


           sensor_msgs::PointCloud2 points=ToPointCloud2Message("point", rotated_point);
           sensor_msgs::LaserScan laserscan;
                laserscan.header.stamp = ros::Time::now();
                laserscan.header.frame_id = "scan";
                laserscan.angle_min = scan->angle_min;
                laserscan.angle_max = scan->angle_max;
                laserscan.angle_increment = scan->angle_increment;

                laserscan.range_min = scan->range_min;
                laserscan.range_max = scan->range_max;
                laserscan.ranges.resize(scan->ranges.size());
                laserscan.intensities.resize(scan->ranges.size());
                for(unsigned int i = 0; i < scan->ranges.size(); ++i)
                {
                    laserscan.ranges[i] = scan->ranges[i];

                }
                geometry_msgs::PoseStamped this_pose_stamped;
                       this_pose_stamped.pose.position.x = map_server_->location_->current_pose.translation()[0];
                       this_pose_stamped.pose.position.y = map_server_->location_->current_pose.translation()[1];

                       geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(map_server_->location_->current_pose.rotation().angle());
                       this_pose_stamped.pose.orientation.x = goal_quat.x;
                       this_pose_stamped.pose.orientation.y = goal_quat.y;
                       this_pose_stamped.pose.orientation.z = goal_quat.z;
                       this_pose_stamped.pose.orientation.w = goal_quat.w;

                       this_pose_stamped.header.stamp=ros::Time::now();
                       this_pose_stamped.header.frame_id="odom";
                       path.poses.push_back(this_pose_stamped);

                       path_pub.publish(path);
              scan_pub.publish(laserscan);

                  point_pub.publish(points);

}
void nodeslam::IMuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
//    std::unique_ptr<sensor::ImuData> imu_data = ToImuData(imu);
//     if (imu_data != nullptr) {
//          if(location_->location_ok)
//          {
//          location_->AddImuData(*imu_data);
//          }
//     }
}
void nodeslam::OdomCallback( const nav_msgs::Odometry::ConstPtr& msg)
{

    std::unique_ptr<sensor::OdometryData> odometry_data =
          ToOdometryData(msg);
     // cout<<"odom_befor:"<<msg->pose<<endl;
    // cout<<"odom_after:"<<odometry_data->pose.DebugString()<<endl;

     if (odometry_data != nullptr) {
          if(location_->location_ok)
          {
          location_->AddOdometry(*odometry_data);
          }
     }
}

void nodeslam::publishLoop(double transform_publish_period)
{
     ros::Rate r(1.0 / transform_publish_period);
     while(ros::ok())
     {// tf::Transform(tf::createQuaternionFromYaw(pose_estimate.rotation().angle()), tf::Vector3(pose_estimate.translation()[0], pose_estimate.translation()[1], 0)),
        map_to_odom_mutex_.lock();
          broadcaster.sendTransform(
                tf::StampedTransform(
                  tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                  ros::Time::now(),"base_link", "scan"));//"map", "base_link"
          broadcaster.sendTransform(
                tf::StampedTransform(
                  tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                  ros::Time::now(),"base_link", "path"));//"map", "base_link"
          broadcaster.sendTransform(
                tf::StampedTransform(
                  tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                  ros::Time::now(),"base_link", "point"));//"map", "base_link"
          broadcaster.sendTransform(
               tf::StampedTransform(
                 tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                 ros::Time::now(),"map", "base_link"));
           map_to_odom_mutex_.unlock();
       r.sleep();
     }
}
void nodeslam::publishMap(double transform_publish_period)
{
     ros::Rate r(2.0);
     while(ros::ok())
     {
         nav_msgs::OccupancyGrid rosMap;
         rosMap.info.resolution = map_server_->location_->probability_grid_->limits().resolution();
         rosMap.info.origin.position.x = -map_server_->location_->probability_grid_->limits().max()[0];
         rosMap.info.origin.position.y = -map_server_->location_->probability_grid_->limits().max()[1];
         rosMap.info.origin.position.z = 0.0;
         rosMap.info.origin.orientation.x = 0.0;
         rosMap.info.origin.orientation.y = 0.0;
         rosMap.info.origin.orientation.z = 0.0;
         rosMap.info.origin.orientation.w = 1.0;
           mapping::CellLimits cell_limits;
           Eigen::Array2i offset;
         map_server_->location_->probability_grid_->ComputeCroppedLimits(&offset, &cell_limits);

         rosMap.info.width = map_server_->location_->probability_grid_->limits().cell_limits().num_y_cells;
         rosMap.info.height = map_server_->location_->probability_grid_->limits().cell_limits().num_x_cells;
         rosMap.data.reserve(rosMap.info.width * rosMap.info.height);
         int height=rosMap.info.height;
         int width=rosMap.info.width;
          for (int x = height-1; x >=0; --x){
             for (int y = width-1; y >=0; --y)
            // for (int y = 0; y <= width-1; ++y)
             {
               const Eigen::Array2i index(x,y);
                constexpr int kUnknownValue = -1;
              const   int value =
                      map_server_->location_->probability_grid_->IsKnown(index)
                          ? RoundToInt(( ProbabilityToColor(map_server_->location_->probability_grid_->GetProbability(index)) / 255.) * 100.)
                          : kUnknownValue;
             rosMap.data.push_back(value);
            }
          }

         rosMap.header.stamp = ros::Time::now();
         rosMap.header.frame_id = "map";
         mapPub.publish(rosMap);
       r.sleep();
     }
}
int main(int argc, char **argv)
{

     ros::init(argc, argv, "node_b");	//初始化ROS，节点命名为node_b，节点名必须唯一。
     std::string dir="/root/workspace/carto_server/configuration_files/";
     std::string name="option_sever.lua";
     //location_=absl::make_unique<cartographer::location::Laserlocation>(dir,name);
      map_server_=absl::make_unique<cartographer::cloud::MapBuilderServer>(dir,name);
      map_server_->Start();
      nodeslam slam;
      cout<<"waiting data"<<endl;
     ros::spin();	//程序进入循环，直到ros::ok()返回false，进程结束。

    return 0;
}
