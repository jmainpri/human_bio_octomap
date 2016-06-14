#include <ros/ros.h>
#include <mutex>
#include <string>
#include <human_bio_octomap/robot_to_octomap.hpp>
#include <human_bio_octomap/scene_subscriber.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using std::cout;
using std::endl;
using namespace human_bio;

// TODO move to utils...
std::vector<std::string> GetStringTokens(const std::string& str)
{
  std::vector<std::string> tokens;
  tokens.clear();
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, ' ')) {
    std::cout << token << endl;
    tokens.push_back(token);
  }
  return tokens;
}

std::vector<double> ConvertVectorOfStringsToNum(
    const std::vector<std::string>& tokens)
{
  std::vector<double> numbers(tokens.size());
  for (uint32_t i = 0; i < numbers.size(); i++) {
    numbers[i] = std::stod(tokens[i]);
  }
  return numbers;
}

std::shared_ptr<SceneSubscriber> human_bio::CreateFromRosParams()
{
  std::string frame_id;

  std::vector<std::string> human_names;
  std::string human_description;
  std::vector<std::string> human_joint_states_topics;

  std::string table_name;
  std::string table_description;
  std::string table_frame_id;

  std::vector<double> bounding_box(6);
  double resolution;
  bool publish_as_markers;

  // Get parameters
  ros::NodeHandle nh("~");

  nh.param(std::string("frame_id"), frame_id, std::string("world"));

  std::string human_names_str;
  nh.param(std::string("human_names"),
           human_names_str,
           std::string("human_1 human_2"));
  nh.param(std::string("human_description"),
           human_description,
           std::string("/human_1/robot_description"));
  std::string human_joint_states_topic_str;
  nh.param(std::string("human_joint_state_topics"),
           human_joint_states_topic_str,
           std::string("/human_1/joint_states /human_2/joint_states"));

  nh.param(std::string("table_name"), table_name, std::string("table"));
  nh.param(std::string("table_description"),
           table_description,
           std::string("/table/robot_description"));
  nh.param(std::string("table_frame_id"),
           table_frame_id,
           std::string("table/table_top"));

  std::string bounding_box_str;
  nh.param(std::string("bounding_box"),
           bounding_box_str,
           std::string("0 3 0 3 0 2.0"));
  nh.param(std::string("resolution"), resolution, double(0.04));
  nh.param(std::string("publish_as_markers"), publish_as_markers, bool(false));

  // Publish as markers
  cout << "publish_as_markers : " << publish_as_markers << endl;

  // Get human names
  human_names = GetStringTokens(human_names_str);

  // Get human_topic_names
  human_joint_states_topics = GetStringTokens(human_joint_states_topic_str);

  // Get bounding box
  bounding_box = ConvertVectorOfStringsToNum(GetStringTokens(bounding_box_str));
  ModelsToOctomap::AABB bb(bounding_box);

  // Create subscriber
  return std::make_shared<SceneSubscriber>(frame_id,
                                           bb,
                                           resolution,
                                           publish_as_markers,
                                           human_names,
                                           human_description,
                                           human_joint_states_topics,
                                           table_name,
                                           table_description,
                                           table_frame_id);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

std::vector<SceneSubscriber::SceneElement> SceneSubscriber::GetAllModels(
    const std::string& human_descriptions,
    const std::string& table_descriptions) const
{
  // Get all elements in the scene
  std::vector<SceneElement> elements;

  Model human_model = LoadMoveitModelFromParams(human_descriptions);
  for (uint32_t i = 0; i < human_names_.size(); i++) {
    SceneElement e_human;
    e_human.name = human_names_[i];
    e_human.model = human_model;
    elements.push_back(e_human);
  }
  SceneElement e_table;
  e_table.name = table_name_;
  e_table.model = LoadMoveitModelFromParams(table_descriptions);
  elements.push_back(e_table);

  return elements;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

Eigen::Affine3d SceneSubscriber::GetTransform(std::string destination_frame,
                                              std::string original_frame)
{
  tf::TransformListener listener;

  std::pair<bool, Eigen::Affine3d> eigen_transform;
  eigen_transform.first = true;
  eigen_transform.second = Eigen::Affine3d::Identity();

  tf::StampedTransform tf_transform;

  try {
    listener.waitForTransform(
        destination_frame, original_frame, ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform(
        destination_frame, original_frame, ros::Time(0), tf_transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    eigen_transform.first = false;
  }

  if (eigen_transform.first) {
    transformTFToEigen(tf_transform, eigen_transform.second);
  }
  return eigen_transform.second;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void SceneSubscriber::GetJointState(
    uint32_t human_id, sensor_msgs::JointState::ConstPtr joint_config)
{
  //  cout << human_id << " , " << __PRETTY_FUNCTION__ << endl;
  //  cout << human_id
  //       << " , human time stamp : " << joint_config->header.stamp.toSec()
  //       << endl;
  //  cout << human_id << " , topic : " << human_joint_states_topics_[human_id]
  //       << endl;
  //  cout << human_id << " , name : " << human_names_[human_id] << endl;

  Config q_map;

  try {
    for (size_t idx = 0; idx < joint_config->name.size(); idx++) {
      std::string name = joint_config->name[idx];
      q_map[name] = joint_config->position[idx];
    }
  } catch (...) {
    ROS_ERROR("Could not map joint correctly");
    return;
  }

  try {
    std::lock_guard<std::mutex> lock(models_to_octomap_mutex_);
    models_to_octomap_->SetConfiguration(
        human_names_[human_id], q_map, Eigen::Affine3d::Identity());
    // This might called concurently for right and left arm (which is ok but not
    // checked)
  } catch (...) {
    ROS_ERROR("Error setting human current config");
    return;
  }
}
