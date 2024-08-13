#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "cartesian_planner/CenterLine.h"
#include "cartesian_planner/Obstacles.h"
#include "cartesian_planner/DynamicObstacles.h"
#include "cartesian_planner/cartesian_planner.h"
#include "nav_msgs/OccupancyGrid.h"
#include "cartesian_planner/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <cmath> //坐标系转换需要
#include <tf/tf.h>

#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <cartesian_planner/Frame.h>
#include <iostream>

using namespace cartesian_planner;

class CartesianPlannerNode {
public:
  explicit CartesianPlannerNode(const ros::NodeHandle &nh) : nh_(nh) {
    env_ = std::make_shared<Environment>(config_);
    planner_ = std::make_shared<CartesianPlanner>(config_, env_);

    path_publisher_ = nh_.advertise<nav_msgs::Path>("path", 1);

    center_line_subscriber_ = nh_.subscribe("/center_line", 1, &CartesianPlannerNode::CenterLineCallback, this);
    map_subscriber_ = nh_.subscribe("mask", 1, &CartesianPlannerNode::mapCallback, this);
    velocity_subscriber = nh_.subscribe("/received_messages", 1, &CartesianPlannerNode::velocitycallback,this);
    self_position_subscriber_ = nh_.subscribe("/tf", 1, &CartesianPlannerNode::SelfPositionCallback, this); 
    map_get_flag_ = true;
  }

 void velocitycallback(const FrameConstPtr& msg){

	if(msg -> id == 0x193){
		std::vector<uint8_t> array;
		for(auto &pt:msg->data){
			array.push_back(pt);
		}
		VehicleArray  velocity;
 
		velocity.array1[0] = array[0]; velocity.array1[1] = array[1];
		velocity_s = velocity.array2 / 32.9542; //  m/s
		 ROS_INFO("velocity1: %f",velocity_s);	
	}	  
}


  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if(map_get_flag_) {
      // 将接收到的地图数据赋值给occupancy_grid变量
      env_->occupancy_grid_ = *msg;
      // std::cout << "Map Got:" << env_->occupancy_grid_.info.resolution <<std::endl;
    }
    // 打印地图信息
    // ROS_INFO("Received map header: ");
    // ROS_INFO("Seq: %d",env_->occupancy_grid_.header.seq);
    // ROS_INFO("0,0num:%f",env_->occupancy_grid_.data[0]);
    // std::cout << "y:" <<  -1.0*env_->occupancy_grid_.info.height * env_->occupancy_grid_.info.resolution / 2 << "~" << env_->occupancy_grid_.info.height * env_->occupancy_grid_.info.resolution / 2 << std::endl;
    // std::cout << "x:" <<  -1.0*env_->occupancy_grid_.info.width * env_->occupancy_grid_.info.resolution / 2 << "~" << env_->occupancy_grid_.info.width * env_->occupancy_grid_.info.resolution / 2 << std::endl;
  } 
  

  //回调函数，当接收到自车位置信息时调用
  void SelfPositionCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    self_position = msg ->transforms[0];
  }

  void CenterLineCallback(const CenterLineConstPtr &msg) {
    map_get_flag_ = false;
    ros::Time start_time = ros::Time::now();
    Trajectory data;
    for (auto &pt: msg->points) {
      TrajectoryPoint tp;
      tp.s = pt.s;
      tp.x = pt.x;
      tp.y = pt.y;
      tp.theta = pt.theta;
      tp.kappa = pt.kappa;
      tp.left_bound = 10;
      tp.right_bound = 10;
      data.push_back(tp);
    }
    env_->SetReference(DiscretizedTrajectory(data));
    env_->Visualize();

    // plan start
    state_.x = data.front().x;
    state_.y = data.front().y;
    state_.theta = data.front().theta;
    state_.v = velocity_s;
    	  ROS_INFO("velocity2: %f",state_.v);	
    state_.phi = 0.0;
    state_.a = 0.0;
    state_.omega = 0.0;

    endState_.x = data.back().x;
    double max_x = env_->occupancy_grid_.info.width * env_->occupancy_grid_.info.resolution / 2;
    if(endState_.x > max_x ) {
      endState_.x = max_x;
    } else if(endState_.x < -max_x) {
      endState_.x = -max_x;
    }
    endState_.y = data.back().y;
    double max_y = env_->occupancy_grid_.info.height * env_->occupancy_grid_.info.resolution / 2;
    if(endState_.y > max_y ) {
      endState_.y = max_y;
    } else if(endState_.y < -max_y) {
      endState_.y = -max_y;
    }
    // endState_.x = data.back().x;
    // endState_.y = data.back().y;
    endState_.theta = data.back().theta;
    endState_.v = 0.0;
    endState_.phi = 0.0;
    endState_.a = 0.0;
    endState_.omega = 0.0;
    // std::cout << "Plan Point" << endState_.x << "," << endState_.y <<std::endl;

    std::vector<double> data_x, data_y;
    for (auto &pt: msg->points) 
    {
       data_x.push_back(pt.x);
       data_y.push_back(pt.y);
    }
    // visualization::Plot(data_x, data_y, 0.05, visualization::Color::Grey, 1, "Coarse Trajectory");
    // visualization::PlotPoints(data_x, data_y, 0.3, visualization::Color::Red, 2, "dp_line2");
    // visualization::Trigger();

    DiscretizedTrajectory result;
    if (planner_->Plan(state_, endState_, result)) {
      visualization::Trigger();
      SendPath(result);
    }
    ros::Time end_time = ros::Time::now();
    double run_time = (end_time - start_time).toSec();
    ROS_INFO("Planning Runtime: %.4f seconds",run_time);
    map_get_flag_ = true;
  }

  void SendPath(DiscretizedTrajectory &result) {
    double nav_yaw =  tf::getYaw(self_position.transform.rotation);
    nav_msgs::Path waypoints;
    
    waypoints.header.stamp = ros::Time::now();
    waypoints.header.frame_id = "map";
    for (int i = 0; i < config_.nfe; i++)
    {
      if (i>=result.data().size()){
        ROS_ERROR_STREAM("ERROR: Vector Wrong!");
        continue;
      }
    auto &pt = result.data().at(i);

    //std::cout << "pt message: " << pt.x << ", " << pt.y << ", " << pt.velocity << ", " << pt.theta << std::endl;

    // 1. 旋转操作
    double x_prime = pt.x * cos(nav_yaw) - pt.y * sin(nav_yaw);
    double y_prime = pt.x * sin(nav_yaw) + pt.y * cos(nav_yaw);

    //std::cout << " message: " << x_prime << ", " << y_prime << std::endl;
    // 2. 平移操作
    double x_double_prime = x_prime + self_position.transform.translation.x;
    double y_double_prime = y_prime + self_position.transform.translation.y;
    // ROS_WARN("x: %f; y: %f, yaw: %f",pt.x,  pt.y, nav_yaw);
    //ROS_WARN("self_position is %f", self_position.transform.translation.y);
    geometry_msgs::PoseStamped this_pose;
    this_pose.header.seq = i;
            //ROS_INFO("path_id is %d", this_pose.header.seq);
    this_pose.header.frame_id = "map";
    this_pose.pose.position.x = x_double_prime;
    this_pose.pose.position.y = y_double_prime;
    this_pose.pose.position.z = pt.velocity;
    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(nav_yaw + pt.theta);
    //ROS_INFO("the yaw is %f",waypoint_vec[i].yaw);
    this_pose.pose.orientation.x = goal_quat.x;
    this_pose.pose.orientation.y = goal_quat.y;
    this_pose.pose.orientation.z = goal_quat.z;
    this_pose.pose.orientation.w = goal_quat.w;

    //std::cout << "this_pose message: " << this_pose.pose.position.x << ", " << this_pose.pose.position.y << ", " << this_pose.pose.position.z << ",||||||," << this_pose.pose.orientation.x << ", " << this_pose.pose.orientation.y << ", " << this_pose.pose.orientation.z << ", " << this_pose.pose.orientation.w << std::endl;

    //ROS_ERROR("path %f", this_pose.pose.orientation.z);
    waypoints.poses.push_back(this_pose);
    }
    std::cout << self_position.transform.translation.x - waypoints.poses.front().pose.position.x << std::endl;
    std::cout << self_position.transform.translation.y - waypoints.poses.front().pose.position.y << std::endl;

    path_publisher_.publish(waypoints);
  }


private:
  ros::NodeHandle nh_;
  cartesian_planner::CartesianPlannerConfig config_;
  Env env_;
  std::shared_ptr<cartesian_planner::CartesianPlanner> planner_;
  CartesianPlanner::StartState state_;
  CartesianPlanner::StartState endState_;
  
  geometry_msgs::TransformStamped self_position;//自车位置
  double last_msg_time,velocity_s;
  ros::Subscriber center_line_subscriber_, obstacles_subscriber_, dynamic_obstacles_subscriber_, goal_subscriber_, map_subscriber_, path_subscriber_, self_position_subscriber_,velocity_subscriber;
  ros::Publisher path_publisher_;
  
  bool center_line_get_flag_, map_get_flag_;
  union VehicleArray{
	  uint8_t array1[2];
	  int16_t array2;
   };
  void PlotVehicle(int id, const math::Pose &pt, double phi) {
    auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi);

    int tire_id = 1;
    for (auto &tire: tires) {
      visualization::PlotPolygon(math::Polygon2d(tire), 0.1, visualization::Color::White, id * (tire_id++),
                                 "Tires");
    }
    visualization::PlotPolygon(math::Polygon2d(config_.vehicle.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.2,
                               visualization::Color::Yellow, id, "Footprint");
    visualization::Trigger();
  }

  std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose, double phi = 0.0) const {
    auto front_pose = pose.extend(config_.vehicle.wheel_base);
    auto track_width = config_.vehicle.width - 0.195;
    double rear_track_width_2 = track_width / 2, front_track_width_2 = track_width / 2;
    double box_length = 0.6345;
    double sin_t = sin(pose.theta());
    double cos_t = cos(pose.theta());
    return {
      math::Box2d({pose.x() - rear_track_width_2 * sin_t, pose.y() + rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({pose.x() + rear_track_width_2 * sin_t, pose.y() - rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({front_pose.x() - front_track_width_2 * sin_t, front_pose.y() + front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
      math::Box2d({front_pose.x() + front_track_width_2 * sin_t, front_pose.y() - front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
    };
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "cartesian_planner_node");

  ros::NodeHandle nh;
  visualization::Init(nh, "velodyne", "cartesian_planner_markers");

  CartesianPlannerNode node(nh);
  ros::spin();
  return 0;
}