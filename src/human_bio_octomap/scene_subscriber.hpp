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

#pragma once

#include <ros/ros.h>
#include <mutex>
#include <string>
#include <human_bio_octomap/robot_to_octomap.hpp>
#include <sensor_msgs/JointState.h>
#include <octomap/octomap.h>
#include <gtest/gtest.h>
#include <glog/logging.h>

#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

using std::cout;
using std::flush;
using std::endl;

namespace human_bio
{

class SceneSubscriber
{

 public:
  struct SceneElement {
    Model model;
    std::string name;
  };

  SceneSubscriber(const std::string& frame_id,
                  const ModelsToOctomap::AABB& bounding_box,
                  double resolution,
                  bool publish_as_markers,
                  const std::vector<std::string>& human_names,
                  const std::string& human_description,
                  const std::vector<std::string>& human_joint_states_topics,
                  const std::string& table_name,
                  const std::string& table_description,
                  const std::string& table_frame_id)
      : frame_id_(frame_id),
        publish_as_markers_(publish_as_markers),
        human_names_(human_names),
        human_joint_states_topics_(human_joint_states_topics),
        table_name_(table_name),
        table_frame_id_(table_frame_id)
  {
    CHECK_EQ(human_names_.size(), human_joint_states_topics.size());

    // Get all Models of elements in the scene from the paramter server
    // Associates a constant pointer to a kinematic and geometric
    // model with a string name, this is then used to generate the octomap

    cout << "1) LOAD ELEMENTS" << endl;
    std::vector<SceneElement> elements =
        GetAllModels(human_description, table_description);

    // Allocate Octomap
    cout << "2) CREATE OCTOMAP" << endl;
    CreateOctomap(bounding_box, resolution, elements);

    is_ready_ = false;

    node_handle_.reset(new ros::NodeHandle());

    // Add subscribers each human has a joint subscriber
    // we use the same function and bind the human id
    for (uint32_t i = 0; i < human_joint_states_topics_.size(); i++) {
      subscriber_joint_angles_.push_back(
          node_handle_->subscribe<sensor_msgs::JointState>(
              human_joint_states_topics_[i],
              1,
              boost::bind(&SceneSubscriber::GetJointState, this, i, _1)));
    }

    // Set the table configuration
    Config q;
    Eigen::Affine3d T_table = GetTransform(frame_id_, table_frame_id_);
    cout << "T_table : " << T_table.translation().transpose() << endl;
    models_to_octomap_->SetConfiguration(table_name_, q, T_table);

    // Create empty octomap
    octomap_ = std::make_shared<octomap::OcTree>(resolution);

    cout << "Finished initialization." << endl;
  }

  //! Creates loads the elements from the parameter server using the URDF
  //! format, it suposes that the name of the humans and table have been set
  std::vector<SceneElement> GetAllModels(
      const std::string& human_descriptions,
      const std::string& table_descriptions) const;

  //! Creates the octomap object
  //! bounding_box : voxel grid bounding box
  //! resolution : resolution of the voxel grid
  //! elements : all models to be added
  void CreateOctomap(const ModelsToOctomap::AABB& bounding_box,
                     double resolution,
                     const std::vector<SceneElement>& elements)
  {
    // Create octomap object
    models_to_octomap_ =
        std::make_shared<ModelsToOctomap>(resolution, bounding_box);

    // Add humans and table
    for (auto e : elements) {
      cout << "Add element : " << e.name << endl;
      models_to_octomap_->AddModel(e.name, e.model);
    }
  }

  //! Gets the join state of a particular model
  //! human_id : id in the name structure
  //! joint_config : joint positions, etc. with names
  void GetJointState(uint32_t human_id,
                     sensor_msgs::JointState::ConstPtr joint_config);

  //! Will retrieve the table trasnform
  //! gives the transform original_frame in destination_frame
  Eigen::Affine3d GetTransform(std::string destination_frame,
                               std::string original_frame);

  ~SceneSubscriber() { Stop(); }

  void Start()
  {
    thread_ = std::thread(std::bind(&SceneSubscriber::Run, this));

    //    cout << "waiting until octomap is available" << flush;
    //    ros::Rate spin_rate(100);  // 100 Hz
    //    while (!IsReady()) {
    //      cout << "." << flush;
    //      spin_rate.sleep();
    //    }
    //    cout << "|ready." << endl;
  }

  bool IsReady() const
  {
    std::lock_guard<std::mutex> lock(models_to_octomap_mutex_);
    return is_ready_;
  }

  void Stop()
  {
    if (!finished_) {
      finished_ = true;
      std::cout << "Stopping monitor. Waiting for thread to join..."
                << std::endl;
      thread_.join();
      std::cout << "Done." << std::endl;
    }
  }

  //! Upadtes the octomap object
  bool Update()
  {
    std::lock_guard<std::mutex> lock(models_to_octomap_mutex_);
    bool ready = models_to_octomap_->Update();
    return ready;
  }

  void Run()
  {
    ros::Rate spin_rate(100);  // 100 Hz
    finished_ = false;
    while (ros::ok() && !finished_) {

      //  cout << "Update octomap ... " << endl;
      if (Update()) {
        // Grab the octomap's mutex to copy the octomap.
        std::lock_guard<std::mutex> lock(models_to_octomap_mutex_);
        octomap_ =
            std::make_shared<octomap::OcTree>(*models_to_octomap_->octomap());
        is_ready_ = true;
      } else {
        is_ready_ = false;
      }

      // ROS_INFO("occpancy map : %d", int(is_ready_));

      ros::spinOnce();    // Process callbacks.
      spin_rate.sleep();  // Sleep until next cycle.
    }
  }

  std::string frame_id() const { return frame_id_; }

  std::shared_ptr<octomap::OcTree> octomap() const
  {
    std::lock_guard<std::mutex> lock(models_to_octomap_mutex_);
    return std::make_shared<octomap::OcTree>(*octomap_);
  }

  std::shared_ptr<ros::NodeHandle> node_handle() { return node_handle_; }

 protected:
  std::string frame_id_;
  std::shared_ptr<const octomap::OcTree> octomap_;
  bool publish_as_markers_;
  std::vector<std::string> human_names_;
  std::vector<std::string> human_joint_states_topics_;
  std::string table_name_;
  std::string table_frame_id_;

  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::shared_ptr<ModelsToOctomap> models_to_octomap_;

  std::vector<ros::Subscriber> subscriber_joint_angles_;

  std::atomic_bool finished_;
  std::atomic_bool is_ready_;

  std::thread thread_;

  mutable std::mutex models_to_octomap_mutex_;
};

std::shared_ptr<SceneSubscriber> CreateFromRosParams();
}
