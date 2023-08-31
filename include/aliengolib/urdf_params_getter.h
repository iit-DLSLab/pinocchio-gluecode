#ifndef IIT_ALIENGO_URDF_PARAMS_GETTER_H
#define IIT_ALIENGO_URDF_PARAMS_GETTER_H

#include "aliengolib/dog/kin_dyn_params.h"
#include "aliengolib/geometry/rotations.h"
#include "aliengolib/utils.hpp"
#include <urdf/model.h>
#include <Eigen/Core>

#define TRUNK_NAME "trunk"

namespace iit {
namespace dog {

class UrdfParamsGetter : public dog::KinDynParams
{
public:
  UrdfParamsGetter(const urdf::Model &model) {
    resetDefaults();
    loadUrdfModel(model);
  }
  ~UrdfParamsGetter() {}

private:
  void loadUrdfModel(const urdf::Model &model)
  {
    model_ = model;

    assert(model_.getLink(TRUNK_NAME));
    //assert(model_.getLink(MULTISENSE_NAME));
    //assert(model_.getLink(IMU_NAME));

    model_.getLinks(links_);

    /////////////TRUNK

    //0 - Get only the links with relevant mass and which are not part of the legs
    for(unsigned int i=0; i<links_.size(); i++)
    {
      if(links_[i]->inertial && links_[i]->inertial->mass > 0.001) // Check if the inertial information exists and if the mass is relevant
        if ( links_[i]->name.find("leg") == std::string::npos ) // Exclude leg links
          if ( links_[i]->name.find("hip") == std::string::npos ) // Exclude the hipassembly links
            if ( links_[i]->name.find("ext") == std::string::npos ) // Exclude the external links (e.g. external arm, cart...)
            {
              links_with_mass_.push_back(links_[i]);
              ROS_DEBUG_STREAM("UrdfParamsGetter: Robot "<< model_.name_ << " Link " << links_[i]->name << " Mass " << links_[i]->inertial->mass);
            }
    }


    //1 - Compute the trunk mass
    trunk_mass_ = 0.0;
    for(unsigned int i=0; i<links_with_mass_.size(); i++)
      trunk_mass_ = trunk_mass_ + links_with_mass_[i]->inertial->mass;

    //2 - Compute the trunk com
    com_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d curr_com = Eigen::Vector3d::Zero();
    std::shared_ptr<urdf::Link> curr_link, start_link, end_link;
    std::vector<std::shared_ptr<urdf::Joint> > curr_joints;
    std::string curr_joint_name;

    for(unsigned int i=0; i<links_with_mass_.size(); i++)
    {

      curr_com(0) = links_with_mass_[i]->inertial->origin.position.x;
      curr_com(1) = links_with_mass_[i]->inertial->origin.position.y;
      curr_com(2) = links_with_mass_[i]->inertial->origin.position.z;

      // Dig for the trunk
      curr_link = links_with_mass_[i]; // While Init
      start_link = curr_link;
      while(curr_link->name != TRUNK_NAME) // While End
      {
        curr_joint_name = curr_link->child_joints[0]->name; // Take the first joint
        curr_com = transformCom(curr_joint_name,curr_com);
        ROS_DEBUG_STREAM("UrdfParamsGetter: Robot "<< model_.name_ << " Link " << curr_link->name << " Child Joint  " << curr_joint_name);
        end_link = curr_link;
        curr_link = curr_link->getParent();// While ++
      }

      if(start_link->name != TRUNK_NAME)
      {
        curr_joints = curr_link->child_joints; // We are in the trunk
        for(unsigned int j=0;j<curr_joints.size();j++)
          if(curr_joints[j]->parent_link_name == TRUNK_NAME && curr_joints[j]->child_link_name == end_link->name) // Find the connection joint between the trunk and the sensor mount
          {
            curr_joint_name = curr_joints[j]->name;
            ROS_DEBUG_STREAM("UrdfParamsGetter: The connection joint between trunk and "<< end_link->name << " is " << curr_joint_name);
          }

        curr_com = transformCom(curr_joint_name,curr_com);
      }

      com_ = com_ + links_with_mass_[i]->inertial->mass * curr_com;
    }
    com_ = com_ / trunk_mass_;

    //3 - Calculate the inertia of the trunk in the base frame, we ignore the sensors inertia
    Eigen::MatrixXd I_com = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd I_b = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd b_X_com = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd com_X_b = Eigen::MatrixXd::Zero(6,6);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    I_com(0,0) = model_.getLink(TRUNK_NAME)->inertial->ixx;
    I_com(2,2) = model_.getLink(TRUNK_NAME)->inertial->izz;
    I_com(1,1) = model_.getLink(TRUNK_NAME)->inertial->iyy;

    I_com(1,0) = model_.getLink(TRUNK_NAME)->inertial->ixy;
    I_com(0,1) = model_.getLink(TRUNK_NAME)->inertial->ixy;

    I_com(2,0) = model_.getLink(TRUNK_NAME)->inertial->ixz;
    I_com(0,2) = model_.getLink(TRUNK_NAME)->inertial->ixz;

    I_com(1,2) = model_.getLink(TRUNK_NAME)->inertial->iyz;
    I_com(2,1) = model_.getLink(TRUNK_NAME)->inertial->iyz;

    Vector3d m;
    m << trunk_mass_, trunk_mass_, trunk_mass_;
    I_com.block<3,3>(3,3) = m.asDiagonal();

    b_X_com = forceVectorTransform(-com_,R);
    com_X_b = motionVectorTransform(com_,R);

    I_b = b_X_com * I_com * com_X_b;
    trunk_inertia_ = I_b.block<3,3>(0,0);

    // Simmetry check
    assert(std::abs(trunk_inertia_(0,1) - trunk_inertia_(1,0)) <= std::numeric_limits<double>::epsilon());
    assert(std::abs(trunk_inertia_(0,2) - trunk_inertia_(2,0)) <= std::numeric_limits<double>::epsilon());
    assert(std::abs(trunk_inertia_(1,2) - trunk_inertia_(2,1)) <= std::numeric_limits<double>::epsilon());

    // Note: RobCoGen requires the off-diagonal inertia terms to be multiplied by -1
    trunk_inertia_(0,1) = trunk_inertia_(1,0) = -1 * trunk_inertia_(1,0);
    trunk_inertia_(0,2) = trunk_inertia_(2,0) = -1 * trunk_inertia_(2,0);
    trunk_inertia_(1,2) = trunk_inertia_(2,1) = -1 * trunk_inertia_(2,1);

    //////////Compute total robot mass
    // Get only the links with relevant mass
    links_with_mass_.resize(0);
    for(unsigned int i=0; i<links_.size(); i++)
    {
      if(links_[i]->inertial && links_[i]->inertial->mass > 0.001) // Check if the inertial information exists and if the mass is relevant
             if ( links_[i]->name.find("ext") == std::string::npos ) // Exclude the external links (e.g. external arm, cart...)
            {
              links_with_mass_.push_back(links_[i]);
              ROS_DEBUG_STREAM("UrdfParamsGetter: Robot "<< model_.name_ << " Link " << links_[i]->name << " Mass " << links_[i]->inertial->mass);
            }
    }
    //Compute the total robot mass
    robot_mass_ = 0.0;
    for(unsigned int i=0; i<links_with_mass_.size(); i++)
      robot_mass_ = robot_mass_ + links_with_mass_[i]->inertial->mass;

    ///////////////////////Kinematic stuff

    assert(model_.getJoint("lf_foot_joint")->parent_to_joint_origin_transform.position.x ==
           model_.getJoint("rf_foot_joint")->parent_to_joint_origin_transform.position.x);

    assert(model_.getJoint("lh_foot_joint")->parent_to_joint_origin_transform.position.x ==
           model_.getJoint("rh_foot_joint")->parent_to_joint_origin_transform.position.x);

    assert(model_.getJoint("lf_foot_joint")->parent_to_joint_origin_transform.position.x ==
           model_.getJoint("lh_foot_joint")->parent_to_joint_origin_transform.position.x);

    foot_x_ = model_.getJoint("rh_foot_joint")->parent_to_joint_origin_transform.position.x;

    // Check simmetry on the x axis
    assert(model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.x ==
           model_.getJoint("rf_haa_joint")->parent_to_joint_origin_transform.position.x);
    assert(model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.x ==
           -model_.getJoint("rh_haa_joint")->parent_to_joint_origin_transform.position.x);
    assert(model_.getJoint("lh_haa_joint")->parent_to_joint_origin_transform.position.x ==
           model_.getJoint("rh_haa_joint")->parent_to_joint_origin_transform.position.x);

    // Check simmetry on the y axis
    assert(model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.y ==
           -model_.getJoint("rf_haa_joint")->parent_to_joint_origin_transform.position.y);
    assert(model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.y ==
           -model_.getJoint("rh_haa_joint")->parent_to_joint_origin_transform.position.y);
    assert(model_.getJoint("lh_haa_joint")->parent_to_joint_origin_transform.position.y ==
           -model_.getJoint("rh_haa_joint")->parent_to_joint_origin_transform.position.y);

    //NOTE we don't check the symmetry on z
    haa_x_ = model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.x;
    haa_y_ = model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.y;
    haa_z_ = model_.getJoint("lf_haa_joint")->parent_to_joint_origin_transform.position.z;
    LF_shin_ = RF_shin_ = LH_shin_ = RH_shin_ = foot_x_;

    if(model_.getName() == "hyq") //FIXME checks?
      haa_hfe_ = model_.getJoint("lf_hfe_joint")->parent_to_joint_origin_transform.position.x;
    else
      haa_hfe_ = -model_.getJoint("lf_hfe_joint")->parent_to_joint_origin_transform.position.y;


    upper_leg_ = model_.getJoint("lf_kfe_joint")->parent_to_joint_origin_transform.position.x; //FIXME checks?

    lower_leg_ = model_.getJoint("lf_foot_joint")->parent_to_joint_origin_transform.position.x; //FIXME checks?

  }

public:
  virtual void resetDefaults() {

    trunk_inertia_ = Eigen::Matrix3d::Zero();
    foot_x_ = 0.0;
    trunk_mass_ = 0.0;
    com_ = Eigen::Vector3d::Zero();
    LF_shin_ = RF_shin_ = LH_shin_ = RH_shin_ = 0.0;
    haa_x_ = haa_y_ = haa_z_ = 0.0;
    model_.clear();
  }

  //getters
  double getValue_robot_total_mass() const {
    return robot_mass_;
  }

  double getValue_trunk_mass() const {
    return trunk_mass_;
  }
  double getValue_trunk_com_x() const {
    return com_(0);
  }
  double getValue_trunk_com_y() const {
    return com_(1);
  }
  double getValue_trunk_com_z() const {
    return com_(2);
  }
  double getValue_trunk_Ix() const {
    return trunk_inertia_(0,0);
  }
  double getValue_trunk_Iy() const {
    return trunk_inertia_(1,1);
  }
  double getValue_trunk_Iz() const {
    return trunk_inertia_(2,2);
  }
  double getValue_trunk_Ixy() const {
    return trunk_inertia_(0,1);
  }
  double getValue_trunk_Ixz() const {
    return trunk_inertia_(0,2);
  }
  double getValue_trunk_Iyz() const {
    return trunk_inertia_(1,2);
  }
  //setters
  // the getters are not virtual (the setters are!) so
  // if you inherit this class, to avoid the use of the default
  // implementation we need to set the functions as virtual!)
  virtual void setValue_robot_total_mass(double val)  {
    assert(val >= 0.0);
    robot_mass_ = val;
  }
  virtual void setValue_trunk_mass(double val)  {
    assert(val >= 0.0);
    trunk_mass_ = val;
  }
  virtual void setValue_trunk_com_x(double val)  {
    com_(0) = val;
  }
  virtual void setValue_trunk_com_y(double val)  {
    com_(1) = val;
  }
  virtual void setValue_trunk_com_z(double val)  {
    com_(2) = val;
  }
  virtual void setValue_trunk_Ix(double val)  {
    trunk_inertia_(0,0) = val;
  }
  virtual void setValue_trunk_Iy(double val)  {
    trunk_inertia_(1,1) = val;
  }
  virtual void setValue_trunk_Iz(double val)  {
    trunk_inertia_(2,2) = val;
  }
  virtual void setValue_trunk_Ixy(double val)  {
    trunk_inertia_(0,1) = val;
    trunk_inertia_(1,0) = val;
  }
  virtual void setValue_trunk_Ixz(double val)  {
    trunk_inertia_(0,2) = val;
    trunk_inertia_(2,0) = val;
  }
  virtual void setValue_trunk_Iyz(double val)  {
    trunk_inertia_(1,2) = val;
    trunk_inertia_(2,1) = val;
  }

  virtual void setValue_foot_x(double val)
  {
    foot_x_ = val;
  }

  double getValue_foot_x() const {
    return foot_x_;
  }

  double getValue_LF_shin() const {
    return LF_shin_;
  }
  double getValue_RF_shin() const {
    return RF_shin_;
  }
  double getValue_LH_shin() const {
    return LH_shin_;
  }
  double getValue_RH_shin() const {
    return RH_shin_;
  }

  virtual void setValue_LF_shin(double val)
  {
    LF_shin_ = val;
  }
  virtual void setValue_RF_shin(double val)
  {
    RF_shin_ = val;
  }
  virtual void setValue_LH_shin(double val)
  {
    LH_shin_ = val;
  }
  virtual void setValue_RH_shin(double val)
  {
    RH_shin_ =  val;
  }

  // TODO
  // these are not used in the actual implementation
  // the generated code for HyQ is not yet parametric
  // for these parameters
  virtual double getValue_haa_x() const {
    return haa_x_;
  }
  virtual double getValue_haa_y() const {
    return haa_y_;
  }
  virtual double getValue_haa_z() const {
    return haa_z_;
  }
  virtual double getValue_haa_hfe() const {
    return haa_hfe_;
  }
  virtual double getValue_upper_leg() const {
    return upper_leg_;
  }
  virtual double getValue_lower_leg() const {
    return lower_leg_;
  }

  virtual void setValue_haa_x(double val) {haa_x_ = val;}
  virtual void setValue_haa_y(double val) {haa_y_ = val;}
  virtual void setValue_haa_z(double val) {haa_z_ = val;}
  virtual void setValue_haa_hfe(double val) {haa_hfe_ = val;} //FIXME checks?
  virtual void setValue_upper_leg(double val) {upper_leg_ = val;} //FIXME checks?
  virtual void setValue_lower_leg(double val) {lower_leg_ = val;} //FIXME checks?


private:

  Eigen::Vector3d transformCom(std::string join_name, const Eigen::Vector3d& com)
  {

    Eigen::Vector3d curr_rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d cur_pos = Eigen::Vector3d::Zero();
    //Eigen::Vector3d curr_com = Eigen::Vector3d::Zero();   // TODO: Not used. Commented to remove warning

    model_.getJoint(join_name)->parent_to_joint_origin_transform.rotation.getRPY(curr_rpy(0),curr_rpy(1),curr_rpy(2));
    cur_pos(0) = model_.getJoint(join_name)->parent_to_joint_origin_transform.position.x;
    cur_pos(1) = model_.getJoint(join_name)->parent_to_joint_origin_transform.position.y;
    cur_pos(2) = model_.getJoint(join_name)->parent_to_joint_origin_transform.position.z;

    return commons::rpyToRot(curr_rpy) * com + cur_pos;
  }

  urdf::Model model_;
  double trunk_mass_, robot_mass_, foot_x_, LF_shin_, RF_shin_, LH_shin_, RH_shin_,
  haa_x_, haa_y_, haa_z_, haa_hfe_, upper_leg_, lower_leg_;
  Eigen::Vector3d com_;
  Eigen::Matrix3d trunk_inertia_;
  std::vector<std::shared_ptr<urdf::Link> > links_, links_with_mass_;
};


}
}
#endif
