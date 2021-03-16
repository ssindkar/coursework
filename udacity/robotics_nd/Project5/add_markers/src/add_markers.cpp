#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Pose goal;
bool goal_reached = false;

void set_marker_and_goal_coordinates(visualization_msgs::Marker* marker, float x, float y) 
{
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker->pose.position.x = x;
    marker->pose.position.y = y;
    marker->pose.position.z = 0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;
    marker->pose.orientation.w = 1.0;

    goal.position.x = x;
    goal.position.y = y;
    goal.position.z = 0;
}

void init_marker(visualization_msgs::Marker* marker) 
{
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker->header.frame_id = "map";
    marker->header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker->ns = "basic_shapes";
    marker->id = 0;

    // Set the marker type to CUBE.
    marker->type = visualization_msgs::Marker::CUBE;

    // Set the scale of the marker -- 1x1x1 means 1m on a side
    marker->scale.x = 0.1;
    marker->scale.y = 0.1;
    marker->scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker->color.r = 0.0f;
    marker->color.g = 1.0f;
    marker->color.b = 0.0f;
    marker->color.a = 1.0;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
  const geometry_msgs::Pose p = msg->pose.pose; 
  // Compute distance between goal and actual position.
  float distance = sqrt(pow(p.position.x - goal.position.x, 2) + 
         pow(p.position.y - goal.position.y, 2));

  // If the robot is within the goal range, assume goal is reached.
  if (distance < 0.25) 
  {
    goal_reached = true;
  } else 
  {
    ROS_INFO("Robot position-> x: [%f], y: [%f]", p.position.x, p.position.y);
    ROS_INFO("Robot is [%f] distance away from goal(%f, %f)", distance, goal.position.x, goal.position.y);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  if (ros::ok())
  {
    visualization_msgs::Marker marker;
    init_marker(&marker);

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // Set the marker action.
    marker.action = visualization_msgs::Marker::ADD;
    set_marker_and_goal_coordinates(&marker, 1.6, -4.6);

    // Publish the marker.
    marker_pub.publish(marker);

    // Subscribe to odom data.
    ros::Subscriber sub = n.subscribe("odom", 1, odom_callback);
 
    while(!goal_reached && ros::ok())
    {
      ros::spinOnce();
    }

    // Hide the marker once the goal is reached.
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Set the drop off zone coordinates.
    marker.action = visualization_msgs::Marker::ADD;
    set_marker_and_goal_coordinates(&marker, -2.75, -9.70);
    marker.lifetime = ros::Duration();
    goal_reached = false;

    while(!goal_reached && ros::ok())
    {
      ros::spinOnce();
    }

    // Publish the marker since the robot is now at the drop off location.
    marker_pub.publish(marker);
    ros::spin();
  }
}
