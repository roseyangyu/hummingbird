/*
 * gazebo_ts_aerodynamic_plugin.cpp
 *
 *  Created on: Nov 11, 2017
 *      Author: yilun
 */

//Aerodynamic Load Plugin for STARS Hummingbird Tailsitter based on the following wing profile:
//E-168 (12.45%) Low-Reynold Number symmetric airfoil, with an wingspan of 20cm on each side

//For now, taking angle of attack out of calculation assuming in hovering regime

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo_ts_aerodynamic_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TSAeroPlugin)

/////////////////////////////////////////////////
TSAeroPlugin::TSAeroPlugin() : k_lift(3.48e-6), k_drag(1.75e-6), k_pitch(-3.44e-7) {
    this->cp = ignition::math::Vector3d(0, 0, 0);
    this->forward = ignition::math::Vector3d(0, 0, 1);
    this->upward = ignition::math::Vector3d(1, 0, 0);

}

/////////////////////////////////////////////////
TSAeroPlugin::~TSAeroPlugin() {
}

/////////////////////////////////////////////////
void TSAeroPlugin::Load(physics::ModelPtr _model,
                        sdf::ElementPtr _sdf) {
    GZ_ASSERT(_model, "TSAeroPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "TSAeroPlugin _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "TSAeroPlugin world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "TSAeroPlugin physics pointer is NULL");

    GZ_ASSERT(_sdf, "TSAeroPlugin _sdf pointer is NULL");

    if (_sdf->HasElement("k_lift"))
        this->k_lift = _sdf->Get<double>("k_lift");

    if (_sdf->HasElement("k_drag"))
        this->k_drag = _sdf->Get<double>("k_drag");

    if (_sdf->HasElement("k_pitch"))
        this->k_pitch = _sdf->Get<double>("k_pitch");

    if (_sdf->HasElement("cp"))
        this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

    // blade forward (-drag) direction in link frame
    if (_sdf->HasElement("forward"))
        this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
    this->forward.Normalize();

    // blade upward (+lift) direction in link frame
    if (_sdf->HasElement("upward"))
        this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
    this->upward.Normalize();


    if (_sdf->HasElement("link_name")) {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        // GZ_ASSERT(elem, "Element link_name doesn't exist!");
        std::string linkName = elem->Get<std::string>();
        this->link = this->model->GetLink(linkName);
        // GZ_ASSERT(this->link, "Link was NULL");

        if (!this->link) {
            gzerr << "Link with name[" << linkName << "] not found. "
                  << "The TSAeroPlugin will not generate forces\n";
        } else {
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&TSAeroPlugin::OnUpdate, this));
        }
    }

    if (_sdf->HasElement("motor_joint_name")) {
        std::string motorJointName = _sdf->Get<std::string>("motor_joint_name");
        this->motorJoint = this->model->GetJoint(motorJointName);
        if (!this->motorJoint) {
            gzerr << "Joint with name[" << motorJointName << "] does not exist.\n";
        }
    }

    if (_sdf->HasElement("control_joint_name")) {
        std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
        this->controlJoint = this->model->GetJoint(controlJointName);
        if (!this->controlJoint) {
            gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
        }
    }

}

/////////////////////////////////////////////////
void TSAeroPlugin::OnUpdate() {
    GZ_ASSERT(this->link, "Link was NULL");

    //Get propeller speed and elevon deflection
    double prop_rads = motorJoint->GetVelocity(0) * 10.0; //Multiply by slow-down sim factor
    double delta = controlJoint->Position(0);
    // pose of body
    ignition::math::Pose3d pose = this->link->WorldPose();



    // rotate forward and upward vectors into inertial frame
    ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
    ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);


    // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
    ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

    // compute lift force at cp
    ignition::math::Vector3d lift = k_lift * prop_rads * prop_rads * delta * upwardI;

    //gzerr << this->controlJoint->GetName() << "lift:" << lift << "delta:"<< delta << "\n";
    // compute drag at cp
    ignition::math::Vector3d drag = k_drag * prop_rads * prop_rads * delta * delta * -forwardI;

    // compute moment (torque) at cp
    ignition::math::Vector3d moment = k_pitch * prop_rads * prop_rads * delta * spanwiseI;

    // moment arm from cg to cp in inertial plane
    ignition::math::Vector3d momentArm = pose.Rot().RotateVector(
            this->cp - this->link->GetInertial()->CoG());

    // force and torque about cg in inertial frame
    ignition::math::Vector3d force = lift + drag;
    // + moment.Cross(momentArm);

    ignition::math::Vector3d torque = moment;
    // - lift.Cross(momentArm) - drag.Cross(momentArm);

    // Correct for nan or inf
    force.Correct();
    this->cp.Correct();
    torque.Correct();

    // apply forces at cg (with torques for position shift)
    this->link->AddForceAtRelativePosition(force, this->cp);
    this->link->AddTorque(torque);
}


