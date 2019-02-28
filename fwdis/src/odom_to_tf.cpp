#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

std::string ROBOT_FRAME;
const double INTERVAL = 0.01;

class OdomToTF
{
public:
  OdomToTF(void);
  void odom_callback(const nav_msgs::OdometryConstPtr&);
  void process(void);

private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odometry;
  geometry_msgs::TransformStamped odom_tf;
  bool odom_updated;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle nh;

  OdomToTF odom_to_tf;
  odom_to_tf.process();
}

OdomToTF::OdomToTF(void)
{
  odom_sub = nh.subscribe("/odom", 1, &OdomToTF::odom_callback, this);
  odom_updated = false;
}

void OdomToTF::process(void)
{
  ros::Rate loop_rate(1.0 / INTERVAL);

  std::cout << "odom_to_tf" << std::endl;

  while(ros::ok()){
    if(odom_updated){
      odom_updated = false;
      odom_tf.header = odometry.header;
      odom_tf.child_frame_id = odometry.child_frame_id;
      odom_tf.transform.translation.x = odometry.pose.pose.position.x;
      odom_tf.transform.translation.y = odometry.pose.pose.position.y;
      odom_tf.transform.rotation = odometry.pose.pose.orientation;
      odom_tf.header = odometry.header;
    }
    odom_broadcaster.sendTransform(odom_tf);

    ros::spinOnce();
    loop_rate.sleep();

  }
}

void OdomToTF::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
  odometry = *msg;
  odom_updated = true;
}

