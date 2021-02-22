#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

bool is_left(int pixel, int step) 
{
  if (pixel > step) 
  {
    pixel = pixel % step;
  }
  return pixel < step / 3;
}

bool is_right(int pixel, int step)
{
  if (pixel > step) 
  {
    pixel = pixel % step;
  }
  return pixel >= step * 2 / 3;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    bool ball_spotted = false;
    // Loop through each pixel in the image and check if there's a bright white one
    // Move only once for each image captured, find where the most of the ball is
    int left, center, right = 0;
    for (int i=0; i < img.height * img.step; i+=3) {
      if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) {
        ball_spotted = true;
        //  Then, identify if this pixel falls in the left, mid, or right side of the image
        if (is_left(i, img.step)) {
          left++;
        } else if (is_right(i, img.step)) {
          right++;
        } else {
          center++;
        }
      }
    }    
    if (ball_spotted) {
      // If most of the ball is in the left third of the image, move left by 10 deg and 0.1m
      if (left > center && left > right) {
        drive_robot(0.1, 0.1745);
      } else if (right > left && right > center){
        // If most of the ball is in the right third of the image, move right by 10 deg and 0.1m
        drive_robot(0.1, -0.1745);
      } else {
        // Else move ahead by 0.1m
        drive_robot(0.1, 0.0);
      }
    } else {
      // Request a stop when there's no white ball seen by the camera
      drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
