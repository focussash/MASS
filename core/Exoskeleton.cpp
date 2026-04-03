#include "Exoskeleton.h"
#include "DARTHelper.h"
#include <iostream>

using namespace dart::dynamics;
using namespace Eigen;

MASS::Exoskeleton::Exoskeleton()
	: mExoWaist(nullptr),
	  mExoThighR(nullptr),
	  mExoThighL(nullptr),
	  mHumanPelvis(nullptr),
	  mHumanFemurR(nullptr),
	  mHumanFemurL(nullptr),
	  mThighRDofIdx(-1),
	  mThighLDofIdx(-1)
{
	// Default bushing params — waist (firmly strapped, slight play at front)
	// High translational stiffness to keep waist on pelvis.
	// Zero rotational — the translational constraint at offset points
	// already resists rotation, and explicit rotational springs on light
	// bodies cause instability (tiny I -> huge angular acceleration).
	mWaistBushing.k_trans = Vector3d(8000.0, 10000.0, 8000.0);
	mWaistBushing.c_trans = Vector3d(300.0, 350.0, 300.0);
	mWaistBushing.k_rot   = Vector3d::Zero();
	mWaistBushing.c_rot   = Vector3d::Zero();
	mWaistBushing.exo_offset   = Vector3d::Zero();  // Computed in ComputeStrapOffsets()
	mWaistBushing.human_offset = Vector3d::Zero();

	// Default bushing params — thigh cuffs (high translational, no rotational)
	// Thigh cuffs constrain position only — orientation is handled by the
	// revolute joint at the hip. Body frame orientations differ significantly
	// between exo and human, so rotational bushing would be destabilizing.
	mThighBushing.k_trans = Vector3d(5000.0, 5000.0, 5000.0);
	mThighBushing.c_trans = Vector3d(200.0, 200.0, 200.0);
	mThighBushing.k_rot   = Vector3d::Zero();
	mThighBushing.c_rot   = Vector3d::Zero();
	mThighBushing.exo_offset   = Vector3d::Zero();  // Computed in ComputeStrapOffsets()
	mThighBushing.human_offset = Vector3d::Zero();
}

void
MASS::Exoskeleton::LoadSkeleton(const std::string& path, bool load_obj)
{
	mSkeleton = MASS::BuildFromFile(path, load_obj);
	if (!mSkeleton) {
		std::cerr << "Failed to load exoskeleton from: " << path << std::endl;
	}
}

void
MASS::Exoskeleton::SetHumanSkeleton(const SkeletonPtr& human_skel)
{
	mHumanSkeleton = human_skel;
}

void
MASS::Exoskeleton::Initialize()
{
	// Resolve exo body nodes
	mExoWaist  = mSkeleton->getBodyNode("ExoWaist");
	mExoThighR = mSkeleton->getBodyNode("ExoThighR");
	mExoThighL = mSkeleton->getBodyNode("ExoThighL");

	if (!mExoWaist || !mExoThighR || !mExoThighL) {
		std::cerr << "Exoskeleton: missing body nodes in skeleton" << std::endl;
		return;
	}

	// Resolve human body nodes
	mHumanPelvis = mHumanSkeleton->getBodyNode("Pelvis");
	mHumanFemurR = mHumanSkeleton->getBodyNode("FemurR");
	mHumanFemurL = mHumanSkeleton->getBodyNode("FemurL");

	if (!mHumanPelvis || !mHumanFemurR || !mHumanFemurL) {
		std::cerr << "Exoskeleton: missing human body nodes" << std::endl;
		return;
	}

	// Find hip joint DOF indices in the exo skeleton
	auto* jointR = mSkeleton->getJoint("ExoThighR");
	auto* jointL = mSkeleton->getJoint("ExoThighL");
	mThighRDofIdx = jointR->getDof(0)->getIndexInSkeleton();
	mThighLDofIdx = jointL->getDof(0)->getIndexInSkeleton();

	std::cout << "Exoskeleton initialized:" << std::endl;
	std::cout << "  Total mass: " << GetTotalMass() << " kg" << std::endl;
	std::cout << "  DOFs: " << mSkeleton->getNumDofs() << std::endl;
	std::cout << "  ThighR DOF idx: " << mThighRDofIdx << std::endl;
	std::cout << "  ThighL DOF idx: " << mThighLDofIdx << std::endl;
}

void
MASS::Exoskeleton::ComputeStrapOffsets()
{
	// Compute strap offsets so attachment points are co-located at current pose.
	// Call after both skeletons are positioned (e.g., after Environment::Reset).
	// Ensures zero initial bushing force.
	auto computeOffsets = [](BodyNode* exo_body, BodyNode* human_body,
		BushingParams& params) {
		Vector3d exo_com   = exo_body->getCOM();
		Vector3d human_com = human_body->getCOM();
		Vector3d meeting   = 0.5 * (exo_com + human_com);
		params.exo_offset   = exo_body->getTransform().inverse() * meeting;
		params.human_offset = human_body->getTransform().inverse() * meeting;
	};

	computeOffsets(mExoWaist,  mHumanPelvis, mWaistBushing);
	computeOffsets(mExoThighR, mHumanFemurR, mThighBushing);

	std::cout << "Strap offsets computed:" << std::endl;
	std::cout << "  Waist exo: " << mWaistBushing.exo_offset.transpose()
	          << "  human: " << mWaistBushing.human_offset.transpose() << std::endl;
	std::cout << "  ThighR exo: " << mThighBushing.exo_offset.transpose()
	          << "  human: " << mThighBushing.human_offset.transpose() << std::endl;
}

void
MASS::Exoskeleton::ApplyBushingForce(
	BodyNode* exo_body,
	BodyNode* human_body,
	const BushingParams& params)
{
	// --- Translational bushing at strap attachment points ---
	// Convert local strap offsets to world positions
	Isometry3d T_exo   = exo_body->getTransform();
	Isometry3d T_human = human_body->getTransform();

	Vector3d exo_strap_world   = T_exo * params.exo_offset;
	Vector3d human_strap_world = T_human * params.human_offset;

	Vector3d d = exo_strap_world - human_strap_world;

	// Velocity at strap points (v_com + omega x r)
	Vector3d exo_vel   = exo_body->getLinearVelocity(params.exo_offset);
	Vector3d human_vel = human_body->getLinearVelocity(params.human_offset);
	Vector3d d_dot = exo_vel - human_vel;

	// Restoring force: pulls exo toward human
	Vector3d F = -params.k_trans.cwiseProduct(d) - params.c_trans.cwiseProduct(d_dot);

	// Apply force at the strap attachment point (offset in local frame)
	exo_body->addExtForce(F, params.exo_offset, /*isForceLocal=*/false, /*isOffsetLocal=*/true);
	human_body->addExtForce(-F, params.human_offset, /*isForceLocal=*/false, /*isOffsetLocal=*/true);

	// --- Rotational bushing ---
	Matrix3d R_exo   = T_exo.linear();
	Matrix3d R_human = T_human.linear();

	// Relative rotation from human frame to exo frame
	Matrix3d R_rel = R_human.transpose() * R_exo;
	AngleAxisd aa(R_rel);
	// Rotation error in world frame
	Vector3d rot_error = R_human * (aa.angle() * aa.axis());

	// Angular velocity difference in world frame
	Vector3d omega_exo   = exo_body->getAngularVelocity();
	Vector3d omega_human = human_body->getAngularVelocity();
	Vector3d omega_diff  = omega_exo - omega_human;

	// Restoring torque: aligns exo orientation with human
	Vector3d tau = -params.k_rot.cwiseProduct(rot_error) - params.c_rot.cwiseProduct(omega_diff);

	exo_body->addExtTorque(tau, false);     // world frame
	human_body->addExtTorque(-tau, false);
}

void
MASS::Exoskeleton::ApplyBushingForces()
{
	// Waist pack <-> Pelvis (moderate stiffness, loose fit)
	ApplyBushingForce(mExoWaist, mHumanPelvis, mWaistBushing);

	// Right thigh cuff <-> Right femur (high stiffness, secure strap)
	ApplyBushingForce(mExoThighR, mHumanFemurR, mThighBushing);

	// Left thigh cuff <-> Left femur (mirrored offsets)
	BushingParams thighL = mThighBushing;
	thighL.exo_offset[0]   = -mThighBushing.exo_offset[0];    // Mirror X for left side
	thighL.human_offset[0] = -mThighBushing.human_offset[0];
	ApplyBushingForce(mExoThighL, mHumanFemurL, thighL);
}

void
MASS::Exoskeleton::SetMotorTorques(const Vector2d& torques)
{
	VectorXd forces = VectorXd::Zero(mSkeleton->getNumDofs());
	forces[mThighRDofIdx] = torques[0];  // Right hip motor
	forces[mThighLDofIdx] = torques[1];  // Left hip motor
	mSkeleton->setForces(forces);
}

Vector2d
MASS::Exoskeleton::GetHipAngles() const
{
	Vector2d angles;
	angles[0] = mSkeleton->getPosition(mThighRDofIdx);  // Right
	angles[1] = mSkeleton->getPosition(mThighLDofIdx);  // Left
	return angles;
}

Vector2d
MASS::Exoskeleton::GetHipVelocities() const
{
	Vector2d vels;
	vels[0] = mSkeleton->getVelocity(mThighRDofIdx);  // Right
	vels[1] = mSkeleton->getVelocity(mThighLDofIdx);  // Left
	return vels;
}

Vector2d
MASS::Exoskeleton::ComputePDTorques(const Vector2d& target_angles,
	double kp, double kv, double torque_limit) const
{
	Vector2d angles = GetHipAngles();
	Vector2d vels   = GetHipVelocities();

	// PD control (paper eq. 15)
	Vector2d tau = kp * (target_angles - angles) - kv * vels;

	// Clamp to motor torque limit
	tau[0] = std::max(-torque_limit, std::min(torque_limit, tau[0]));
	tau[1] = std::max(-torque_limit, std::min(torque_limit, tau[1]));

	return tau;
}

double
MASS::Exoskeleton::GetTotalMass() const
{
	double mass = 0.0;
	for (std::size_t i = 0; i < mSkeleton->getNumBodyNodes(); i++) {
		mass += mSkeleton->getBodyNode(i)->getMass();
	}
	return mass;
}
