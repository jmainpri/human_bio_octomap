/**
 * Copyright (c) 2019, Jim Mainprice
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
