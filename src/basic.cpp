#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#define PI 3.14159265

namespace basic{


class Basic{
 public:
  Basic(ros::NodeHandle& nh, ros::NodeHandle& pnh);
 private:
  void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
  ros::Subscriber sub;
  ros::Publisher marker_pub;
  int a ;
  //ros::Rate ;
  uint32_t shape ;
  visualization_msgs::Marker marker;
};

  Basic::Basic(ros::NodeHandle& nh, ros::NodeHandle& pnh){    
    a=0;
    //r(1);
    sub = nh.subscribe("odom", 1, &Basic::odom_cb,this);

    shape = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = a;
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker.type = shape;

  }




void Basic::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){

    float f = 0.0;

    marker.action = visualization_msgs::Marker::ADD;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    //marker.scale.y = 1.0;
    //marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    float x , y,theta = -35;
    geometry_msgs::Point p;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    p.z = msg->pose.pose.position.z;

    p.x = x * cos(theta * PI / 180) - y * sin(theta*PI/180);
    p.y = x * sin(theta * PI / 180) + y * cos(theta*PI/180);
    p.x = -1.6*p.x+11;
    p.y = -1.6*p.y+4.5;

    marker.points.push_back(p);

    
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return ;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  basic::Basic basic1(nh, pnh);
  
  //ros::Subscriber sub = n.subscribe("/odom", 1, odom_cb);
  ros::spin(); 
}
