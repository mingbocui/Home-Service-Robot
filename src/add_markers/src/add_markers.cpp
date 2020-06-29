#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>


geometry_msgs::Pose pick_pos;
geometry_msgs::Pose drop_pos;

// pick_pos.position.x = 1;
// pick_pos.position.y = 2;
// drop_pos.position.x = 0;
// drop_pos.position.y = 0;

visualization_msgs::Marker pick_zone_marker;
visualization_msgs::Marker drop_zone_marker;
ros::Publisher marker_pub;
bool is_preparing = true;

bool check_arriving(const nav_msgs::Odometry::ConstPtr& msg, geometry_msgs::Pose pose)
{
  float dist = abs(pose.position.x - msg->pose.pose.position.x) + abs(pose.position.y - msg->pose.pose.position.y);
  return dist < 0.5;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    bool is_picking = check_arriving(msg, pick_pos);
    bool is_dropping = check_arriving(msg, drop_pos);

    if(is_preparing)
    {
      marker_pub.publish(pick_zone_marker);
      ROS_INFO("Publishing a delivering task!");
    }

    if(is_picking)
    {
        is_preparing = false;
        sleep(5);
        pick_zone_marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(pick_zone_marker);
        ROS_INFO("HomeServiceRobot is picking!");
    } 
    else if(is_dropping)
    {
        sleep(5);
        drop_zone_marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(drop_zone_marker);
        ROS_INFO("HomeServiceRobot is delivering!");
    }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  // ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber marker_sub = n.subscribe("/odom", 1000, odom_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  pick_pos.position.x = 1;
  pick_pos.position.y = 2;
  drop_pos.position.x = 3;
  drop_pos.position.y = 2;

  while (ros::ok())
  {
    // visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    pick_zone_marker.header.frame_id = "map";
    pick_zone_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pick_zone_marker.ns = "add_markers";
    pick_zone_marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    pick_zone_marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    pick_zone_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    pick_zone_marker.pose.position.x = pick_pos.position.x;
    pick_zone_marker.pose.position.y = pick_pos.position.y;
    pick_zone_marker.pose.position.z = 0.0;
    pick_zone_marker.pose.orientation.x = 0.0;
    pick_zone_marker.pose.orientation.y = 0.0;
    pick_zone_marker.pose.orientation.z = 0.0;
    pick_zone_marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    pick_zone_marker.scale.x = 0.3;
    pick_zone_marker.scale.y = 0.3;
    pick_zone_marker.scale.z = 0.3;
    // Set the color -- be sure to set alpha to something non-zero!
    pick_zone_marker.color.r = 0.0f;
    pick_zone_marker.color.g = 0.0f;
    pick_zone_marker.color.b = 1.0f;
    pick_zone_marker.color.a = 1.0;

    pick_zone_marker.lifetime = ros::Duration();
    

    // Publish the marker
    // while (marker_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }
    // marker_pub.publish(pick_zone_marker);
    //ROS_INFO("Created Marker at (1, 2)");
    // wait 5 seconds and 5second pause
    // ros::Duration(10).sleep();
    //ROS_INFO("Waiting");

    // visualization_msgs::Marker marker_deliver;

    drop_zone_marker.header.frame_id = "map";
    drop_zone_marker.header.stamp = ros::Time::now();
    drop_zone_marker.ns = "add_markers";
    drop_zone_marker.id = 1;
    drop_zone_marker.type = shape;
    // drop_zone_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the delivered marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    drop_zone_marker.pose.position.x = drop_pos.position.x;
    drop_zone_marker.pose.position.y = drop_pos.position.y;
    drop_zone_marker.pose.position.z = 0;
    drop_zone_marker.pose.orientation.x = 0.0;
    drop_zone_marker.pose.orientation.y = 0.0;
    drop_zone_marker.pose.orientation.z = 0.0;
    drop_zone_marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    drop_zone_marker.scale.x = 0.3;
    drop_zone_marker.scale.y = 0.3;
    drop_zone_marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    drop_zone_marker.color.r = 0.0f;
    drop_zone_marker.color.g = 0.0f;
    drop_zone_marker.color.b = 1.0f;
    drop_zone_marker.color.a = 1.0;

    // drop_zone_marker.lifetime = ros::Duration();

    marker_pub.publish(pick_zone_marker);
    // delete this marker after 5 seconds;
    // drop_zone_marker.lifetime = ros::Duration(5);

    // Publish the marker
    // while (marker_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }
    // marker_pub.publish(drop_zone_marker);
    // ROS_INFO("Created Marker at (0, 0)");

    // ros::Duration().sleep();
    while(ros::ok()){
      ros::spinOnce();
      r.sleep();
    }
  }
}