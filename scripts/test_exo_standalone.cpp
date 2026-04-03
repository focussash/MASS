/**
 * Outputs body node COM positions for stick-figure animation.
 */
#include "DARTHelper.h"
#include "Exoskeleton.h"
#include <dart/dart.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <iostream>
#include <fstream>

void writePos(std::ofstream& f, dart::dynamics::BodyNode* bn) {
    Eigen::Vector3d p = bn->getCOM();
    f << p[0] << "," << p[1] << "," << p[2];
}

int main()
{
    std::string mass_root = std::string(MASS_ROOT_DIR);

    auto human_skel = MASS::BuildFromFile(mass_root + "/data/human.xml");
    auto ground_skel = MASS::BuildFromFile(mass_root + "/data/ground.xml");

    MASS::Exoskeleton exo;
    exo.LoadSkeleton(mass_root + "/data/exo_model.xml");
    exo.SetHumanSkeleton(human_skel);
    exo.Initialize();

    auto world = std::make_shared<dart::simulation::World>();
    world->setGravity(Eigen::Vector3d(0, -9.8, 0));
    world->setTimeStep(1.0 / 500.0);
    world->getConstraintSolver()->setCollisionDetector(
        dart::collision::BulletCollisionDetector::create());
    world->addSkeleton(human_skel);
    world->addSkeleton(ground_skel);
    world->addSkeleton(exo.GetSkeleton());
    exo.ComputeStrapOffsets();

    // Body nodes to track
    const char* human_bodies[] = {
        "Pelvis", "Spine", "Torso", "Neck", "Head",
        "FemurR", "TibiaR", "TalusR",
        "FemurL", "TibiaL", "TalusL",
        "ShoulderR", "ArmR", "ForeArmR",
        "ShoulderL", "ArmL", "ForeArmL"
    };
    int nh = 17;
    const char* exo_bodies[] = {"ExoWaist", "ExoThighR", "ExoThighL"};
    int ne = 3;

    std::ofstream csv(mass_root + "/scripts/exo_anim_data.csv");
    // Header
    csv << "time";
    for (int i = 0; i < nh; i++)
        csv << "," << human_bodies[i] << "_x," << human_bodies[i] << "_y," << human_bodies[i] << "_z";
    for (int i = 0; i < ne; i++)
        csv << "," << exo_bodies[i] << "_x," << exo_bodies[i] << "_y," << exo_bodies[i] << "_z";
    csv << std::endl;

    int total_steps = 400;
    int record_every = 5;  // Record at 100 Hz (every 5th step at 500 Hz)
    for (int i = 0; i < total_steps; i++) {
        exo.ApplyBushingForces();
        exo.SetMotorTorques(Eigen::Vector2d::Zero());
        world->step();

        bool ok = human_skel->getPositions().allFinite() &&
                  exo.GetSkeleton()->getPositions().allFinite();
        if (!ok) { std::cerr << "NaN at step " << i << std::endl; break; }

        if (i % record_every == 0) {
            csv << (i + 1) * (1.0 / 500.0);
            for (int j = 0; j < nh; j++) {
                csv << ",";
                writePos(csv, human_skel->getBodyNode(human_bodies[j]));
            }
            for (int j = 0; j < ne; j++) {
                csv << ",";
                writePos(csv, exo.GetSkeleton()->getBodyNode(exo_bodies[j]));
            }
            csv << std::endl;
        }
    }
    csv.close();
    std::cout << "Animation data saved (" << total_steps/record_every << " frames)" << std::endl;
    return 0;
}
