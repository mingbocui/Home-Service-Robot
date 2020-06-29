#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 5;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);
    

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ROS_INFO("Created Marker at (5, 0)");
    // wait 5 seconds and 5second pause
    ros::Duration(10).sleep();
    ROS_INFO("Waiting");

    visualization_msgs::Marker marker_deliver;

    marker_deliver.header.frame_id = "map";
    marker_deliver.header.stamp = ros::Time::now();
    marker_deliver.ns = "add_markers";
    marker_deliver.id = 0;
    marker_deliver.type = shape;
    marker_deliver.action = visualization_msgs::Marker::ADD;

    // Set the pose of the delivered marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_deliver.pose.position.x = 2;
    marker_deliver.pose.position.y = 2;
    marker_deliver.pose.position.z = 0;
    marker_deliver.pose.orientation.x = 0.0;
    marker_deliver.pose.orientation.y = 0.0;
    marker_deliver.pose.orientation.z = 0.0;
    marker_deliver.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_deliver.scale.x = 0.3;
    marker_deliver.scale.y = 0.3;
    marker_deliver.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_deliver.color.r = 0.0f;
    marker_deliver.color.g = 0.0f;
    marker_deliver.color.b = 1.0f;
    marker_deliver.color.a = 1.0;
    // delete this marker after 5 seconds;
    marker_deliver.lifetime = ros::Duration(5);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker_deliver);
    ROS_INFO("Created Marker at (2, 2)");

    // ros::Duration().sleep();
    ros::spin();
    return 0;
  }
}