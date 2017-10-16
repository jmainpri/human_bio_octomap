#include <Eigen/Geometry>
#include <geometric_shapes/body_operations.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <human_bio_octomap/robot_to_octomap.hpp>
#include <moveit/distance_field/find_internal_points.h>
#include <moveit/robot_model/robot_model.h>
#include <octomap/octomap.h>
#include <urdf/model.h>

using std::cerr;
using std::cout;
using std::endl;
using namespace human_bio;

std::string shape_to_string(shapes::ShapeType v) {
  switch (v) {
  case shapes::ShapeType::UNKNOWN_SHAPE:
    return "UNKNOWN_SHAPE";
  case shapes::ShapeType::SPHERE:
    return "SPHERE";
  case shapes::ShapeType::CYLINDER:
    return "CYLINDER";
  case shapes::ShapeType::CONE:
    return "CONE";
  case shapes::ShapeType::BOX:
    return "BOX";
  case shapes::ShapeType::PLANE:
    return "PLANE";
  case shapes::ShapeType::MESH:
    return "MESH";
  case shapes::ShapeType::OCTREE:
    return "OCTREE";
  default:
    return "[Unknown OS_type]";
  }
}

void findInternalPointsConvex_local(const bodies::Body &body, double resolution,
                                    EigenSTL::vector_Vector3d &points) {
  bodies::BoundingSphere sphere;
  body.computeBoundingSphere(sphere);
  double xval_s = std::floor((sphere.center.x() - sphere.radius - resolution) /
                             resolution) *
                  resolution;
  double yval_s = std::floor((sphere.center.y() - sphere.radius - resolution) /
                             resolution) *
                  resolution;
  double zval_s = std::floor((sphere.center.z() - sphere.radius - resolution) /
                             resolution) *
                  resolution;
  double xval_e = sphere.center.x() + sphere.radius + resolution;
  double yval_e = sphere.center.y() + sphere.radius + resolution;
  double zval_e = sphere.center.z() + sphere.radius + resolution;

  //  if (body.getType() == shapes::ShapeType::SPHERE) {
  //    cout << "radius : " << body.getDimensions()[0] << endl;
  //  }

  //  cout << " bs : center : " << sphere.center.transpose() << endl;
  //  cout << " bs : radius : " << sphere.radius << endl;
  //  cout << " bs : resolution : " << resolution << endl;

  double anti_alias = 0.2; // TODO remove this eventually
  double step = resolution * anti_alias;
  Eigen::Vector3d pt(Eigen::Vector3d::Zero());
  for (pt.x() = xval_s; pt.x() <= xval_e; pt.x() += step) {
    for (pt.y() = yval_s; pt.y() <= yval_e; pt.y() += step) {
      for (pt.z() = zval_s; pt.z() <= zval_e; pt.z() += step) {
        if (body.containsPoint(pt)) {
          points.push_back(pt);
        }
      }
    }
  }
}

Model human_bio::LoadMoveitModelFromString(const std::string &xml_robot) {
  // Get model
  cout << "Create URDF model" << endl;
  urdf::Model model;
  model.initString(xml_robot);
  auto urdf_model = std::make_shared<urdf::ModelInterface>(model);

  // Semantic model
  cout << "Create SRDF model" << endl;
  srdf::Model semantic_model;
  semantic_model.initString(*urdf_model, "");
  auto srf_model = std::make_shared<const srdf::Model>(semantic_model);

  return Model(new moveit::core::RobotModel(urdf_model, srf_model));
}

Model human_bio::LoadMoveitModelFromParams(const std::string &ros_param) {
  ros::NodeHandle node_handle;

  std::string full_ros_param;
  if (!node_handle.searchParam(ros_param, full_ros_param)) {
    cerr << "Could not find ros_parameter " << ros_param
         << " on ros_parameter server" << endl;
    return Model();
  }

  std::string xml_robot;
  if (!node_handle.getParam(full_ros_param, xml_robot)) {
    cerr << "Could read ros_parameter " << full_ros_param
         << " on ros_parameter server" << endl;
    return Model();
  }

  return human_bio::LoadMoveitModelFromString(xml_robot);
}

std::vector<bodies::Body *> ModelToBodies::GetBodies() {
  bool tranform_from_base(
      (base_frame_.matrix() - Eigen::Affine3d::Identity().matrix()).norm() >
      1e-9);
  uint32_t k = 0;
  std::vector<bodies::Body *> bodies;
  for (int i = 0; i < robot_model_->getLinkModels().size(); i++) {

    const moveit::core::LinkModel *link = robot_model_->getLinkModels()[i];
    // std::string link_name = link->getName();
    const Eigen::Affine3d &link_pose = state_->getGlobalLinkTransform(link);

    for (int j = 0; j < link->getShapes().size(); j++) {

      bodies::Body *body = NULL;

      const Eigen::Affine3d &shape_pose =
          link->getCollisionOriginTransforms()[j];

      if (bodies_.empty()) {
        body = bodies::createBodyFromShape(link->getShapes()[j].get());
        bodies.push_back(body);
      } else {
        body = bodies_[k++];
      }
      Eigen::Affine3d pose;
      if (!tranform_from_base) {
        pose = link_pose * shape_pose;
      } else {
        pose = base_frame_ * link_pose * shape_pose;
      }
      body->setPose(pose);
    }
  }
  if (bodies_.empty()) {
    bodies_ = bodies;
  }
  return bodies_;
}

ModelsToOctomap::ModelsToOctomap(double resolution, const AABB &bb)
    : resolution_(resolution) {
  tree_ = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(resolution_));
  octomap::point3d bb_min(bb.x_min, bb.y_min, bb.z_min);
  octomap::point3d bb_max(bb.x_max, bb.y_max, bb.z_max);
  tree_->useBBXLimit(true);
  tree_->setBBXMin(bb_min);
  tree_->setBBXMax(bb_max);
  cout << "Create octomap with dim min : " << bb_min << endl;
  cout << "                    dim max : " << bb_max << endl;
}

bool ModelsToOctomap::Update() {
  // Remove all points from the OcTree
  tree_->clear();

  if (!ConfigurationsInitialized()) {
    return false;
  }

  uint32_t nb_points = 0;
  uint32_t nb_in_bb = 0;
  for (auto m : models_) {
    std::vector<bodies::Body *> bodies = m->GetBodies();

    for (auto b : bodies) {
      EigenSTL::vector_Vector3d point_vec;
      findInternalPointsConvex_local(*b, 2. * resolution_, point_vec);

      for (auto p : point_vec) {
        octomap::point3d point(p.x(), p.y(), p.z());
        if (tree_->inBBX(point)) {
          nb_in_bb++;
          tree_->updateNode(point, true, true);
        }
        nb_points++;
      }
    }
  }

  // cout << "update inner occupancy ( "
  //  << nb_points << " , " << nb_in_bb << " )" << endl;
  tree_->updateInnerOccupancy();
  return true;
}
