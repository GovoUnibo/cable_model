#include "cable_model.hpp"

using namespace cable_utils;
using namespace std;
using namespace mass;
using namespace cable_dynamics;




Cable::Cable(gazebo::physics::ModelPtr model, double x_origin, double y_origin, double z_origin, float mass, double width, double length, int num_of_masses, string link_prefix)
    : model(model)
    , MassSpringDamping(num_of_masses, length, mass, width)
    , x(x_origin), y(y_origin), z(z_origin)
    , w(width), l(length)
    , num_masses(num_of_masses)
    , cable_masses(vector<Particle>(num_masses))
    , resultant_force(num_masses)
{

    //Initialize rows
    for(int i = 0; i < num_masses; i++){
        cable_masses[i].setLink(model->GetLink(link_prefix + std::to_string(i))); // get link from model by name
        // cable_masses[i].setCollisionElement((i%2 == 0 ? true : false)); // set collision if is a even element //non funziona
        //cable_masses[i].setCollideMode(collision_mode_all); // set collision mode
        MassSpringDamping::setMassInitialPosition(cable_masses[i].getInitialPosition(), i); // set initial position
    }
    // cout << "Mass Distance: " << (cable_masses[29].getInitialPosition() - cable_masses[28].getInitialPosition()).Length() << endl;
    // cout << "Initial displacement: " << round_to((cable_masses[29].getInitialPosition() - cable_masses[28].getInitialPosition()).Length() - length/(num_masses - 1), 1e-4) << endl;

    this->setFirstMassFixed(true);
    this->setLastMassFixed(true);
}

Cable::~Cable() {}

void Cable::useGravity(bool use_gravity){
    this->use_gravity = use_gravity;
    for (int i = 0; i < num_masses; i++)
        cable_masses[i].setGravityMode(use_gravity);
}


void Cable::setDamperCoef(float K_d){ MassSpringDamping::setDamperCoef(K_d);}
void Cable::setYoungModulus(double young_modulus){ MassSpringDamping::setYoungModulus(young_modulus);}
void Cable::setPoissonRatio(double poisson_ratio){ MassSpringDamping::setPoissonRatio(poisson_ratio);}



const gazebo::physics::LinkPtr Cable::getLink(int i) { return cable_masses[i].getLink(); }

int Cable::getResolution(){ return num_masses; }

ignition::math::Vector3d Cable::getMassPos(int i) { /*wrt world*/  return cable_masses[i].getAbsolutePosition(); }
ignition::math::Vector3d Cable::getLeftNeighbourPos(int i) {  /*wrt world*/ return cable_masses[i-1].getAbsolutePosition(); }
ignition::math::Vector3d Cable::getRightNeighbourPos(int i) { /*wrt world*/ return cable_masses[i+1].getAbsolutePosition(); }
ignition::math::Vector3d Cable::getMassInitialPos(int i) { /*wrt world*/  return cable_masses[i].getInitialPosition(); }

ignition::math::Vector3d Cable::getMassVel(int i) { /*wrt world*/ return cable_masses[i].getAbsoluteVelocity(); }
ignition::math::Vector3d Cable::getLeftNeighbourVel(int i) { /*wrt world*/ return cable_masses[i-1].getAbsoluteVelocity(); }
ignition::math::Vector3d Cable::getRightNeighbourVel(int i) { /*wrt world*/ return cable_masses[i+1].getAbsoluteVelocity(); }

ignition::math::Vector3d Cable::getVector(int i) {     return this->getMassPos(i) - this->getMassPos(i-1);}

double Cable::getBeta(int i) {   return atan((this->getVector(i+1).Cross(this->getVector(i))).Length() / this->getVector(i+1).Dot(this->getVector(i)));}

ignition::math::Vector3d Cable::getParticlesRelativePos(int i, int j) {     return cable_masses[i].getLink()->WorldPose().Pos() - cable_masses[j].getLink()->WorldPose().Pos();}

ignition::math::Vector3d Cable::tripleCross(ignition::math::Vector3d u1, ignition::math::Vector3d u2, ignition::math::Vector3d u3) {    return u1.Cross(u2.Cross(u3));}

ignition::math::Vector3d Cable::getForceWrtWorld(int i){    return this->cable_masses[i].getForce();}

void Cable::setFirstMassFixed(bool is_fixed){  cable_masses[0].setFixed(is_fixed); }
void Cable::setLastMassFixed(bool is_fixed){  cable_masses[num_masses-1].setFixed(is_fixed); }
void Cable::setFirstMassGrasped(bool is_grasped){ cable_masses[0].setGrasped(is_grasped); }
void Cable::setLastMassGrasped(bool is_grasped){ this->cable_masses[num_masses-1].setGrasped(is_grasped); }

void Cable::updateModel(){

    // cable_masses[0].setFixed(true); //fissa prima massa
    // cable_masses[num_of_masses - 1].setFixed(true); //fissa ultima massa

    // ignition::math::Vector3d point1(1, 0, 0);
    // cable_masses[1].updateVelocity(point1);
    // bool once = true;
    // if (once){
    //     cable_masses[2].updatePosition(ignition::math::Pose3d{ignition::math::Vector3d{3, 0, 0}, ignition::math::Quaterniond{0, 0, 0, 1}});
    //     once = false;
    // }
    // cable_masses[1].updatePosition(ignition::math::Pose3d{ignition::math::Vector3d{7, 0, 0}, ignition::math::Quaterniond{0, 0, 0, 1}});

    for(int i=0; i<MassSpringDamping::num_of_masses; i++){
        MassSpringDamping::updateMassPosition(cable_masses[i].getAbsolutePosition(), i);
        MassSpringDamping::updateMassVelocity(cable_masses[i].getAbsoluteVelocity(), i);
        MassSpringDamping::updateMassAcceleration(cable_masses[i].getAbosulteAcceleration(), i);
    }


    MassSpringDamping::updateCableTwist(this->cable_masses[0].getAbsoluteRotation(), this->cable_masses[num_masses-1].getAbsoluteRotation());


    MassSpringDamping::computeSpringsForces();
    MassSpringDamping::computeDampingForces();
    // MassSpringDamping::computeInertiaForces(); // gazebo le calcola automaticamente cout << cable_masses[2].getForce() << endl;
    // MassSpringDamping::evaluateGravity();
    cout << cable_masses[2].getForce() << endl;

    if (cable_masses[0].isFixed() || cable_masses[0].isGrasped())
        MassSpringDamping::addInitialConstrainSpring();

    if (cable_masses[num_masses-1].isFixed() || cable_masses[num_masses-1].isGrasped())
        MassSpringDamping::addFinalConstrainSpring();



    for(int i=0; i< MassSpringDamping::num_of_masses; i++){
        cable_masses[i].updateForce(MassSpringDamping::getResultantForce(i));
        // cable_masses[i].updateTorque(MassSpringDamping::getTwistingForce(i));
    }




}


