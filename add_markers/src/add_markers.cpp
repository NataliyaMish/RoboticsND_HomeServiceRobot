#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

  if (ros::ok())
  {    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "pick_up";
    marker.id = 0;

    // Set the marker type
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the scale of the marker
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    
    // Set the position of the marker == pick-up location 
    marker.pose.position.x = -2.3;
    marker.pose.position.y = 6.5;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 0.0;
    
    // Set the marker action to add the marker
    marker.action = visualization_msgs::Marker::ADD;

    // Publish the marker to add it
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

    // Pause for 5 seconds
    ros::Duration(5).sleep();

    // Set the marker action and publish it to hide it
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
   }

   // Pause for 5 seconds
   ros::Duration(5).sleep();
   
   while (ros::ok())
   {  
    // Set the new marker unique ID 
    marker.ns = "drop_off";
    marker.id = 1;

    // Set the marker type
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the scale of the marker
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    
    // Set the position of the marker == drop-off location 
    marker.pose.position.x = 3.5;
    marker.pose.position.y = 0.5;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 0.0;
    
    // Set the marker action and publish it to display it at drop-off location
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);

    r.sleep();
  }
}
