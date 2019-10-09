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

#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/body_operations.h>
#include <octomap/octomap.h>

#include <glog/logging.h>

namespace human_bio
{
typedef std::map<std::string, double> Config;
typedef std::shared_ptr<moveit::core::RobotModel> Model;
typedef std::shared_ptr<moveit::core::RobotState> State;

// TODO move these function to utils package
Model LoadMoveitModelFromString(const std::string& xml_robot);
// TODO move these function to utils package
Model LoadMoveitModelFromString(const std::string& xml_robot);
Model LoadMoveitModelFromParams(const std::string& ros_param);

class ModelToBodies
{
 public:
  ModelToBodies(const std::string name, Model model, double resolution)
      : configuration_initialized_(false),
        name_(name),
        robot_model_(model),
        state_(new moveit::core::RobotState(model)),
        resolution_(resolution),
        base_frame_(Eigen::Affine3d::Identity())
  {
    if (name_ == "table") {
      configuration_initialized_ = true;
    }
  }

  ~ModelToBodies()
  {
    for (auto b : bodies_) {
      delete b;
    }
  }

  // Set the configuration variables
  void SetConfiguration(const Config& q)
  {
    state_->setVariablePositions(q);
    state_->update();
  }

  // Set the base frame
  void set_base_frame(const Eigen::Affine3d& T) { base_frame_ = T; }

  // configuration set
  void set_configuration_initialized(bool v) { configuration_initialized_ = v; }

  // Returns the shapes with thier transforms
  std::vector<bodies::Body*> GetBodies();

  // get name
  const std::string& name() const { return name_; }

  // Configuration is set or not
  bool is_configuration_initialized() const
  {
    return configuration_initialized_;
  }

 private:
  bool configuration_initialized_;
  std::string name_;
  State state_;
  Model robot_model_;
  std::vector<bodies::Body*> bodies_;
  std::vector<EigenSTL::vector_Vector3d> bodies_inner_points_;
  double resolution_;
  Eigen::Affine3d base_frame_;
};

class ModelsToOctomap
{
 public:
  struct AABB {
    AABB() {}
    AABB(const std::vector<double>& bb)
    {
      CHECK_EQ(bb.size(), 6);
      x_min = bb[0], y_min = bb[2], z_min = bb[4];
      x_max = bb[1], y_max = bb[3], z_max = bb[5];
    }
    double x_min, y_min, z_min;
    double x_max, y_max, z_max;
  };

  ModelsToOctomap(double resolution, const AABB& bouding_box);

  // Add new model
  void AddModel(const std::string name, const Model& model)
  {
    auto model_to_bodies = std::shared_ptr<ModelToBodies>(
        new ModelToBodies(name, model, resolution_));
    models_.push_back(model_to_bodies);
  }

  // Set the configuration for the models named name
  // name : name of the model
  // q : map of joint names to positions
  void SetConfiguration(const std::string& name,
                        const Config& q,
                        const Eigen::Affine3d& base_frame)
  {
    for (int i = 0; i < models_.size(); i++) {
      if (models_[i]->name() == name) {
        models_[i]->SetConfiguration(q);
        models_[i]->set_base_frame(base_frame);
        models_[i]->set_configuration_initialized(true);
      }
    }
  }

  // Return true if all models have been set to configuration
  bool ConfigurationsInitialized() const
  {
    for (auto m : models_) {
      if (!m->is_configuration_initialized()) {
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<const octomap::OcTree> octomap() const { return tree_; }

  // Update ocupancy map
  bool Update();

 private:
  std::vector<std::shared_ptr<ModelToBodies> > models_;
  std::shared_ptr<octomap::OcTree> tree_;
  double resolution_;
};
}
