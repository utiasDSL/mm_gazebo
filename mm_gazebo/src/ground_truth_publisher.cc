/*
 * ground_truth_publisher.cc
 *
 *  Created on: 2017-08-14
 *      Author: root
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>

std::vector<int> getIndexes(std::vector<std::string> &input, std::string searched) {
    std::vector<int> result;

    for (int i = 0; i < input.size(); i++) {
        if (input[i] == searched) {
            result.push_back(i);
        }
    }

    return result;
}

std::vector<int> locations;
bool flag_first = true;
geometry_msgs::Pose odom;
void gazeboCallback(const gazebo_msgs::LinkStatesConstPtr &states)
{
	if(flag_first){
		std::vector<std::string> sVector;
		sVector.assign(states->name.data(), states->name.data() + states->name.size());
		locations = getIndexes(sVector,"ridgeback::base_link");
		flag_first = false;
	}

	odom = states->pose[locations[0]];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo_ground_truth_broadcaster");
  ros::NodeHandle n;
  ros::Duration(5).sleep();
  ros::Subscriber ground_truth_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",1,gazeboCallback);

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(50.0);

  while (n.ok()){
    transform.setOrigin( tf::Vector3(odom.position.x, odom.position.y, odom.position.z));
    transform.setRotation(tf::Quaternion(odom.orientation.x,odom.orientation.y,odom.orientation.z,odom.orientation.w));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "real_base_link"));

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};



