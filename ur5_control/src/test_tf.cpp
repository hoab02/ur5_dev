#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pose_extraction");
    ros::NodeHandle nh;

    // Create a TransformListener
    tf::TransformListener listener;

    // Wait for the transform to become available
    ros::Duration(1.0).sleep();

    // Specify the target frame (base_link) and source frame (tool0)
    std::string targetFrame = "/base_link";
    std::string sourceFrame = "/tool0";

    // Get the latest transform between the frames
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        return 1;
    }

    // Extract the pose from the transform
    tf::Vector3 position = transform.getOrigin();
    tf::Quaternion orientation = transform.getRotation();

    // Print the pose
    ROS_INFO("End Effector Pose:");
    ROS_INFO("Position: x=%.3f, y=%.3f, z=%.3f", position.x(), position.y(), position.z());
    ROS_INFO("Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
             orientation.x(), orientation.y(), orientation.z(), orientation.w());

    return 0;
}