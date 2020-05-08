/**
 * @file JumperModel.cpp
 * @brief Contains the definition of the members of the class JumperModel.
 */

// This module
#include "JumperModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

#define _USE_MATH_DEFINES
#include <math.h>

namespace
{
    const struct Config
    {
        double density;
        double square_side_radius;
        double leg_radius;
        double stiffness;
        double damping;
        double pretension;
        double leg_length;
        double squar_side_length;
        bool hist;
        double maxTension;
        double targetVelocity;

    }c = {
       5,           // density (mass / length^3)
       0.20,        // radius (length)
       0.20,
       3000.0,      // stiffness (mass / sec^2)
       30.0,        // damping (mass / sec)
       12000.0,     // pretension (mass * length / sec^2)
       10.0,        // length of leg
       10.0,        // length of one of the square side 
       0,           // history logging (boolean)
       30000,       // max tension
       30           // target velocity
    };
}

JumperModel::JumperModel(btVector3 pos, double angle[2], double leg_angle[2]) : tgModel(), starting_coordinates(pos)
{
    starting_angle[0] = angle[0];
    starting_angle[1] = angle[1];
    starting_leg_angle[0] = leg_angle[0];
    starting_leg_angle[1] = leg_angle[1];
}

JumperModel::~JumperModel()
{
}

void JumperModel::addNodes(tgStructure& s, double leg_angle[2])
{
    // y z x

    s.addNode(c.squar_side_length/2.0,0,c.squar_side_length/2.0);
    s.addNode(-c.squar_side_length/2.0,0,c.squar_side_length/2.0);
    s.addNode(-c.squar_side_length/2.0,0,-c.squar_side_length/2.0);
    s.addNode(c.squar_side_length/2.0,0,-c.squar_side_length/2.0);

    double y_offset = sin(leg_angle[0]*M_PI/180.0)*(2*c.leg_length);
    double x_offset = sin(leg_angle[1]*M_PI/180.0)*(2*c.leg_length); 
    double z_offset = sqrt(4*pow(c.leg_length,2)-pow(y_offset,2)-pow(x_offset, 2));

    s.addNode(0,-c.leg_length,0);
    s.addNode(0+y_offset,-c.leg_length+z_offset,0+x_offset);

}


void JumperModel::addRods(tgStructure& s)
{
    s.addPair(0,1,"square_rod");
    s.addPair(1,2,"square_rod");
    s.addPair(2,3,"square_rod");
    s.addPair(3,0,"square_rod");
    
    s.addPair(4,5,"leg_rod");
}

void JumperModel::addMuscles(tgStructure& s)
{
    // Upper Strings
    s.addPair(4,0,"muscle");
    s.addPair(1,4,"muscle");
    s.addPair(2,4,"muscle");
    s.addPair(3,4,"muscle");


    // Lower Strings
    s.addPair(5,0,"muscle");
    s.addPair(1,5,"muscle");
    s.addPair(2,5,"muscle");
    s.addPair(3,5,"muscle");

}

void JumperModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config leg_rod_config(c.leg_radius, c.density);
    const tgRod::Config square_rod_config(c.square_side_radius, c.density);
    
    const tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
        c.hist, c.maxTension, c.targetVelocity);

    
    // Create a structure that will hold the details of this model
    tgStructure s;

    // Add nodes to the structure
    addNodes(s, starting_leg_angle);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // // Move the structure so it doesn't start in the ground
    // s.move(btVector3(0, 50, 0));
    
    s.move(starting_coordinates);
    const btVector3 fixed_point (0,-c.leg_length,0);
    const btVector3 axis_rotation_x (0,0,1);
    s.addRotation(fixed_point, axis_rotation_x, starting_angle[0]*M_PI/180.0);
    const btVector3 axis_rotation_y (0,0,1);
    s.addRotation(fixed_point, axis_rotation_y, starting_angle[1]*M_PI/180.0);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("square_rod", new tgRodInfo(square_rod_config));
    spec.addBuilder("leg_rod", new tgRodInfo(leg_rod_config));

    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    allRods = tgCast::filter<tgModel, tgRod> (getDescendants());

    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void JumperModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void JumperModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

std::vector<tgBasicActuator*>& JumperModel::getAllActuators()
{
    return allActuators;
}

std::vector<tgRod*>& JumperModel::getAllRods()
{
    return allRods;
}

void JumperModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
