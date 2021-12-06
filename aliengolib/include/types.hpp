#ifndef _CREX_KINEMATICS_TYPES_HPP_
#define _CREX_KINEMATICS_TYPES_HPP_

namespace aliengolib{
struct FK_In
{
	double cur_joints[4];
};

struct FK_Out
{
	double pos_x, pos_y, pos_z;
	double rot_x, rot_y, rot_z, rot_w;
};

struct IK_In
{
	double pos_x, pos_y, pos_z;
  double last_joints[4];
};

struct IK_Out
{
	double des_joints[4];
	double min_dist;
	double min_dist_dof;
};

struct KinematicsConfig
{
  short thorax_conf; // defining wheter to put the thorax joint to the front or rear
  double thigh, thigh_horizontal, thigh_vertical;
  double knee_offset;
  double base_offset;
  double shank;
  double min_joints[4], max_joints[4];
};
}

#endif // _CREX_KINEMATICS_TYPES_HPP_
