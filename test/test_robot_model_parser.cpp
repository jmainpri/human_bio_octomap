#include <human_bio_octomap/robot_to_octomap.hpp>
#include <moveit/robot_model/robot_model.h>
#include <ros/package.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <glog/logging.h>

using std::cerr;
using std::cout;
using std::endl;
using namespace human_bio;

// Reads a whole file and puts it in an std::string
// TODO move to utils ... eventually ..
std::string ReadFile(const std::string& filepath)
{
  std::ifstream file(filepath.c_str(), std::ifstream::in);
  if (!(file.good() && file.is_open())) {
    cout << "could not open file : " << filepath << endl;
    return std::string("");
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();
  return buffer.str();
}

// Get the model from file or from the paramter server
// by loading the file into the paramter server first
Model LoadModel(const std::string& filepath)
{
  Model model;

  std::string xml_file = ReadFile(filepath);
  cout << "PRINT XML ROBOT ( " << filepath << " )" << endl;
  // cout << xml_file << endl;

  const bool with_ros_core = false;
  if (with_ros_core) {
    // Load xml file to rosparam
    ros::NodeHandle nh;
    std::string ros_param = "/robot_description";
    nh.setParam(ros_param, xml_file);

    // Load model
    model = human_bio::LoadMoveitModelFromParams(ros_param);
  } else {
    model = human_bio::LoadMoveitModelFromString(xml_file);
  }

  return model;
}

int PrintModelInfo(human_bio::Model model)
{
  cout << "------------------------------" << endl;
  cout << model->getName() << endl;
  cout << "nb of links : " << model->getLinkModels().size() << endl;
  for (int i = 0; i < model->getLinkModels().size(); i++) {
    cout << "Link(" << i << ") : " << model->getLinkModels()[i]->getName()
         << endl;
  }

  return model->getLinkModels().size();
}

bool CheckModels()
{
  std::string filename;
  std::string filepath;

  filename = "/urdf/human_bio.urdf";
  filepath = ros::package::getPath("human_bio_urdf") + filename;
  human_bio::Model human = LoadModel(filepath);
  if (PrintModelInfo(human) != 53) {
    return false;
  }

  filename = "/urdf/table.urdf";
  filepath = ros::package::getPath("human_bio_scene") + filename;
  human_bio::Model table = LoadModel(filepath);
  if (PrintModelInfo(table) != 11) {
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_load_model");
  if (CheckModels()) {
    cout << "Models OK!" << endl;
  } else {
    cout << "Error loading models" << endl;
  }
  return 0;
}
