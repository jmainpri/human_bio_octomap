#include <human_bio_octomap/scene_subscriber.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <ros/ros.h>

using namespace human_bio;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_bio_voxel_occupancy");

  // Create subscriber and start spinning
  std::shared_ptr<SceneSubscriber> subscriber = CreateFromRosParams();
  subscriber->Start();

  // Create octomap publisher
  std::shared_ptr<ros::NodeHandle> nh = subscriber->node_handle();
  std::string octomap_topic = "/octomap_full";
  ros::Publisher octomap_pub =
      nh->advertise<octomap_msgs::Octomap>(octomap_topic.c_str(), 1000);
  ROS_INFO("octomap_topic : %s", octomap_topic.c_str());

  // Publish octomap
  ros::Rate spin_rate(25);  // 25 Hz
  while (ros::ok()) {
    std::shared_ptr<const octomap::OcTree> octomap = subscriber->octomap();
    octomap_msgs::Octomap msg;
    octomap_msgs::binaryMapToMsg(*octomap, msg);
    std::cout << std::endl;
    msg.header.frame_id = subscriber->frame_id();
    if (!msg.data.empty()) {
      octomap_pub.publish(msg);
    }
    ros::spinOnce();    // Process callbacks.
    spin_rate.sleep();  // Sleep until next cycle.
  }
  subscriber->Stop();
  return 0;
}
