#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

void poseCallback(const object_recognition_msgs::RecognizedObjectArrayConstPtr& msg){
  if(msg->objects.empty())
  	return;

  if(msg->objects.size()>1)
  	ROS_WARN("Detected Multiple objects, only broadcasting the first one ..");

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->objects.at(0).pose.pose.pose.position.x, msg->objects.at(0).pose.pose.pose.position.y, msg->objects.at(0).pose.pose.pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->objects.at(0).pose.pose.pose.orientation.x, msg->objects.at(0).pose.pose.pose.orientation.y, msg->objects.at(0).pose.pose.pose.orientation.z, msg->objects.at(0).pose.pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_color_optical_frame", "coke_can"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_obj_to_base");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/recognized_object_array", 10, &poseCallback);
  
  ros::spin();
  return 0;
};