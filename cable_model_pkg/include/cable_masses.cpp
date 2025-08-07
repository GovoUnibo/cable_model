
#include "cable_masses.hpp"


using namespace mass;
using namespace std;

Particle::Particle() {}
Particle::~Particle() {}

void Particle::setLink(gazebo::physics::LinkPtr link) { this->link = link; }

void Particle::setCollisionElement(bool collision){ /*Set whether this body will collide with others in the model*/
    link->SetSelfCollide(collision); 
}

bool Particle::isCollisionElement(){ /*if this is true, this body will collide with other bodies even if they share the same parent.*/
    return link->GetSelfCollide();
}

void Particle::setCollideMode(std::string mode){
    /*  all: collides with everything 
        none: collides with nothing 
        sensors: collides with everything else but other sensors 
        fixed: collides with everything else but other fixed 
        ghost: collides with everything else but other ghost
    */
    link->SetCollideMode(mode);
}

void Particle::setGravityMode(bool _mode){
    if (!fixed)
        link->SetGravityMode(_mode);
}

void Particle::setStatic(bool static_) {
    link->SetStatic(static_);
}

void Particle::setFixed(bool fixed) {  //fixes the particle by disabling gravity and setting the link static (not updating forces)
    this->fixed = fixed;
    link->SetLinkStatic(fixed);
    link->SetGravityMode(!fixed);
    // link->SetWorldPose(link->WorldPose());
}

void Particle::setGrasped(bool grasped){ this->grasped = grasped; }
bool Particle::isGrasped(){ return this->grasped;}

bool Particle::isFixed() { return fixed; }

void Particle::updateVelocity(ignition::math::Vector3d vel) {   
    link->SetLinearVel(vel);
}

void Particle::updateForce(ignition::math::Vector3d force_applied){
    //saturare ad una forza lungo le componenti x, y, z

    // double max_force = 3.0;
    // double fx = std::clamp(force_applied.X(), -max_force, max_force);
    // double fy = std::clamp(force_applied.Y(), -max_force, max_force);
    // double fz = std::clamp(force_applied.Z(), -max_force, max_force); 

    // ignition::math::Vector3d force_applied(fx, fy, fz);

    // cout << "Force applied: " << force_applied << endl;


    if(!fixed)
        link->AddLinkForce(force_applied);

        // link->SetForce(force_applied);
}

void Particle::updateTorque(ignition::math::Vector3d torque_applied){
    if(!fixed) {link->AddTorque(torque_applied);}
        // link->SetTorque(torque_applied);
}

void Particle::updateAcceleration(ignition::math::Vector3d acc){
    // if(!fixed)
    //     link->SetLinearAccel(acc);  dice che non esiste mentre nella documentazione esiste... deprecated?
}




void Particle::updatePosition(ignition::math::Pose3d pos){
    link->SetWorldPose(pos);
}

void Particle::updateAngularVelocity(ignition::math::Vector3d vel) { 
    return link->SetAngularVel(vel); 
}

const gazebo::physics::LinkPtr Particle::getLink() { return link; }

ignition::math::Vector3d Particle::getAbsolutePosition(){
    return link->WorldPose().Pos();
}

ignition::math::Quaterniond Particle::getAbsoluteRotationQuaternion(){
    return link->WorldPose().Rot();
}

ignition::math::Vector3d Particle::getAbsoluteRotationEuler(){
   return link->WorldPose().Rot().Euler();
}


ignition::math::Vector3d Particle::getAbsoluteVelocity(){
    return link->WorldCoGLinearVel();
    // return link->WorldLinearVel();
}

ignition::math::Vector3d Particle::getAbsoluteAngularVelocity(){
    return link->WorldAngularVel();
}

            
ignition::math::Vector3d Particle::getAbosulteAcceleration(){
    return link->WorldLinearAccel();
}


ignition::math::Vector3d Particle::getInitialRelPosition(){
    return link->InitialRelativePose().Pos();
}

ignition::math::Vector3d Particle::getForce() { 
    return link->WorldForce(); 
}


