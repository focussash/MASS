#ifndef __MASS_EXOSKELETON_H__
#define __MASS_EXOSKELETON_H__
#include "dart/dart.hpp"

namespace MASS
{

struct BushingParams
{
	Eigen::Vector3d k_trans;        // Translational stiffness [N/m]
	Eigen::Vector3d c_trans;        // Translational damping [N·s/m]
	Eigen::Vector3d k_rot;         // Rotational stiffness [N·m/rad]
	Eigen::Vector3d c_rot;         // Rotational damping [N·m·s/rad]
	Eigen::Vector3d exo_offset;    // Strap point in exo body local frame
	Eigen::Vector3d human_offset;  // Strap point in human body local frame
};

class Exoskeleton
{
public:
	Exoskeleton();

	void LoadSkeleton(const std::string& path, bool load_obj = false);
	void SetHumanSkeleton(const dart::dynamics::SkeletonPtr& human_skel);
	void Initialize();
	void ComputeStrapOffsets();  // Call after skeletons are positioned (e.g., after Reset)

	// Bushing configuration
	void SetWaistBushingParams(const BushingParams& params) { mWaistBushing = params; }
	void SetThighBushingParams(const BushingParams& params) { mThighBushing = params; }

	// Per-step operations
	void ApplyBushingForces();
	void SetMotorTorques(const Eigen::Vector2d& torques);

	// State readout
	Eigen::Vector2d GetHipAngles() const;
	Eigen::Vector2d GetHipVelocities() const;

	// PD controller (paper eq. 15)
	Eigen::Vector2d ComputePDTorques(const Eigen::Vector2d& target_angles,
		double kp, double kv, double torque_limit) const;

	const dart::dynamics::SkeletonPtr& GetSkeleton() const { return mSkeleton; }
	double GetTotalMass() const;

private:
	void ApplyBushingForce(
		dart::dynamics::BodyNode* exo_body,
		dart::dynamics::BodyNode* human_body,
		const BushingParams& params);

	dart::dynamics::SkeletonPtr mSkeleton;
	dart::dynamics::SkeletonPtr mHumanSkeleton;

	// Body node pointers (resolved during Initialize)
	dart::dynamics::BodyNode* mExoWaist;
	dart::dynamics::BodyNode* mExoThighR;
	dart::dynamics::BodyNode* mExoThighL;
	dart::dynamics::BodyNode* mHumanPelvis;
	dart::dynamics::BodyNode* mHumanFemurR;
	dart::dynamics::BodyNode* mHumanFemurL;

	// Bushing parameters
	BushingParams mWaistBushing;
	BushingParams mThighBushing;  // Same for both thighs (symmetric)

	// Joint DOF indices in exo skeleton
	int mThighRDofIdx;
	int mThighLDofIdx;
};

};

#endif
