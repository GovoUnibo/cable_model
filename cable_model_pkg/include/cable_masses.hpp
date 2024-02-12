
#ifndef CABE_MASSES
#define CABE_MASSES


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <iostream>

namespace mass{

    class Particle {
        private:
            gazebo::physics::LinkPtr link;
            ignition::math::Vector3d force_cache;
            ignition::math::Vector3d initial_pos;
            bool fixed = false, grasped = false;


        public:
            Particle();
            ~Particle();
            void setLink(gazebo::physics::LinkPtr link);

            void setCollideMode(std::string mode);
            void setGravityMode(bool _mode);

            void setCollisionElement(bool collision);
            bool isCollisionElement();

            const gazebo::physics::LinkPtr getLink();
            ignition::math::Vector3d getForce();
            ignition::math::Vector3d getInitialPosition();
            ignition::math::Quaterniond getAbsoluteOrientation();
            ignition::math::Vector3d getAbsoluteRotation();
            ignition::math::Vector3d getAbsolutePosition();
            ignition::math::Vector3d getAbsoluteVelocity();
            ignition::math::Vector3d getAbosulteAcceleration();
            


            void updateForce(ignition::math::Vector3d force_applied);
            void updateTorque(ignition::math::Vector3d torque_applied);
            
            void updatePosition(ignition::math::Pose3d pos);
            void updateVelocity(ignition::math::Vector3d vel);
            void updateAcceleration(ignition::math::Vector3d acc);
            

            void updateAngularVelocity(ignition::math::Vector3d vel) ;
            
            bool isFixed();
            void setFixed(bool fixed);
            void setStatic(bool static_);
            void setGrasped(bool grasped);
            bool isGrasped();


    };

};
#endif