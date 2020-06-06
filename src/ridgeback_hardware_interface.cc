#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

boost::thread cm_thread;
class RB : public hardware_interface::RobotHW
{
public:
  ros::NodeHandle n;
  RB(const ros::NodeHandle& nh){

    n = nh;
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_x("ridgeback_x", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_x);

    hardware_interface::JointStateHandle state_handle_y("ridgeback_y", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_y);

    hardware_interface::JointStateHandle state_handle_theta("ridgeback_theta", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_theta);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_x(jnt_state_interface.getHandle("ridgeback_x"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_x);

    hardware_interface::JointHandle vel_handle_y(jnt_state_interface.getHandle("ridgeback_y"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_y);

    hardware_interface::JointHandle vel_handle_theta(jnt_state_interface.getHandle("ridgeback_theta"), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_theta);

    registerInterface(&jnt_vel_interface);

    ROS_INFO_STREAM("loaded everything");
  };

  void init()
  {
    buffer = new tf2_ros::Buffer();
    tfl = new tf2_ros::TransformListener(*buffer);
    platform_pub = n.advertise<geometry_msgs::Twist>("/cart/cmd_vel", 1000, true);
    joint_pub = n.advertise<sensor_msgs::JointState>("/rb_cart_joint_states", 1000, true);
    odom_sub = n.subscribe("/odometry/filtered", 1, &RB::odomCallback, this);

    // below added for joint state limits.. following https://github.com/ros-controls/ros_control/wiki/joint_limits_interface

    hardware_interface::JointHandle x_joint_handle = jnt_vel_interface.getHandle("ridgeback_x");
    hardware_interface::JointHandle y_joint_handle = jnt_vel_interface.getHandle("ridgeback_y");
    hardware_interface::JointHandle theta_joint_handle = jnt_vel_interface.getHandle("ridgeback_theta");

	joint_limits_interface::JointLimits x_limits;
	joint_limits_interface::JointLimits y_limits;
	joint_limits_interface::JointLimits theta_limits;

	// populate limits
    const bool x_rosparam_limits_ok = joint_limits_interface::getJointLimits("ridgeback_x", n, x_limits);
    const bool y_rosparam_limits_ok = joint_limits_interface::getJointLimits("ridgeback_y", n, y_limits);
    const bool theta_rosparam_limits_ok = joint_limits_interface::getJointLimits("ridgeback_theta", n, theta_limits);

    // register handles with joint limits interface

    joint_limits_interface::VelocityJointSaturationHandle x_handle(x_joint_handle, x_limits);
    joint_limits_interface::VelocityJointSaturationHandle y_handle(y_joint_handle, y_limits);
    joint_limits_interface::VelocityJointSaturationHandle theta_handle(theta_joint_handle, theta_limits);

    vel_jnt_limits_interface.registerHandle(x_handle);
    vel_jnt_limits_interface.registerHandle(y_handle);
    vel_jnt_limits_interface.registerHandle(theta_handle);

    registerInterface(&vel_jnt_limits_interface);

  }


  void getTransform(const std::string& target_frame, const std::string& source_frame, geometry_msgs::TransformStamped *t ) {
    try{
	*t = buffer->lookupTransform(target_frame, source_frame,
				     ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
	ROS_WARN("%s",ex.what());
	ros::Duration(1.0).sleep();
    }
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    odom = *msg;
  }

  void read() {
    geometry_msgs::TransformStamped t;
    sensor_msgs::JointState j;
    getTransform("odom","base_link",&t);
    pos[0] = t.transform.translation.x;
    pos[1] = t.transform.translation.y;
    pos[2] = atan2(2*(t.transform.rotation.w*t.transform.rotation.z -t.transform.rotation.y*t.transform.rotation.x),\
                   1-2*(t.transform.rotation.y*t.transform.rotation.y + t.transform.rotation.z*t.transform.rotation.z));//2*asin(t.transform.rotation.z);
    vel[0] = odom.twist.twist.linear.x;
    vel[1] = odom.twist.twist.linear.y;
    vel[2] = odom.twist.twist.angular.z;

    //publish joint state for Ridgeback on a topic
    //work around untill we find out why built-in HW interface method always gives 0
    j.name.push_back("ridgeback_x");
    j.name.push_back("ridgeback_y");
    j.name.push_back("ridgeback_theta");
    j.position.push_back(pos[0]);
    j.position.push_back(pos[1]);
    j.position.push_back(pos[2]);
    j.velocity.push_back(vel[0]);
    j.velocity.push_back(vel[1]);
    j.velocity.push_back(vel[2]);
    joint_pub.publish(j);
  }

  void write(ros::Time time, ros::Duration period) {
	  // added for enforcing velocity limits
      vel_jnt_limits_interface.enforceLimits(period);

      tw.linear.x = cmd[0];
      tw.linear.y = cmd[1];
      tw.angular.z = cmd[2];
      platform_pub.publish(tw);
  }

private:
  tf2_ros::Buffer *buffer;
  tf2_ros::TransformListener *tfl;
  ros::Publisher platform_pub;
  ros::Publisher joint_pub;
  ros::Subscriber odom_sub;
  nav_msgs::Odometry odom;
  geometry_msgs::Twist tw;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[3];
  double pos[3];
  double vel[3];
  double eff[3];
  bool controller_up;

  // added for joint limits
  hardware_interface::PositionJointInterface pos_jnt_interface;
  joint_limits_interface::PositionJointSaturationInterface jnt_limits_interface;
  joint_limits_interface::VelocityJointSaturationInterface vel_jnt_limits_interface;
};

void controller_thread() {
  ros::NodeHandle n;

  RB robot(n);
  ros::Rate r(50);
  controller_manager::ControllerManager cm(&robot,n);
  robot.init();

  while(ros::ok()) {
      robot.read();
      cm.update(ros::Time::now(),ros::Duration(r));
      robot.write(ros::Time::now(), ros::Duration(r));
      r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "akal_interface");

  ROS_INFO("Controller manager launched.");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  controller_thread();
  ros::shutdown();
  return 0;
}
