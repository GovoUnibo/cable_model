#ifndef CABLE_UTILS
#define CABLE_UTILS


#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <vector>
#include "cable_masses.hpp"
#include "cable_dynamics.hpp"




namespace cable_utils {

    class Cable : protected cable_dynamics::MassSpringDamping {
        
        private:
            double x, y, z, w, l; // offset xyz, width, length
            int num_masses; //horizontal and vertical resolutions
            std::vector<mass::Particle> cable_masses;
            gazebo::physics::ModelPtr model;

            std::vector<ignition::math::Vector3d> resultant_force;

            
            std::string collision_mode_all = "all"; // collides with everything
            std::string collision_mode_none = "none"; //collides with nothing 
            std::string collision_mode_sensors = "sensors"; //collides with everything else but other sensors
            std::string collision_mode_fixed = "fixed";  // collides with everything else but other fixed
            std::string collision_mode_ghost = "ghost"; //collides with everything else but other ghost
            
            ignition::math::Vector3d getParticlesRelativePos(int i, int j);
            ignition::math::Vector3d getLinkLength(int i);

            ignition::math::Vector3d getLeftNeighbourPos(int i);
            ignition::math::Vector3d getRightNeighbourPos(int i);
            ignition::math::Vector3d getMassPos(int i);
            ignition::math::Vector3d getMassInitialPos(int i);

            ignition::math::Vector3d getLeftNeighbourVel(int i);
            ignition::math::Vector3d getRightNeighbourVel(int i);
            ignition::math::Vector3d getMassVel(int i);
            double getBeta(int i);
            ignition::math::Vector3d getVector(int i);

            
            ignition::math::Vector3d tripleCross(ignition::math::Vector3d u1, ignition::math::Vector3d u2, ignition::math::Vector3d u3);

            bool isFirstMassGrasped, isLastMassGrasped;


        public:
            Cable();
            Cable(gazebo::physics::ModelPtr model, double x_origin, double y_origin, double z_origin, float mass, double width, double length, int num_of_masses, std::string link_prefix) ;
            ~Cable();
            
            void useGravity(bool use_gravity);
            void setDamperCoef(float K_d);
            void setYoungModulus(double young_modulus);
            void setPoissonRatio(double poisson_ratio);
            
            
            
            std::vector<ignition::math::Vector3d> damping_forces;
            const gazebo::physics::LinkPtr getLink(int i);
            int getResolution();
            ignition::math::Vector3d getForceWrtWorld(int);
            void updateModel();
            void updateModel(bool isFirstMassGrasped, bool isLastMassGrasped);

            void setFirstMassGrasped(bool is_grasped);
            void setLastMassGrasped(bool is_grasped);
            void setFirstMassFixed(bool is_fixed);
            void setLastMassFixed(bool is_fixed);
            
            
    };

}




#endif