/* 
 * File:   cable_model.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef CABLE_MODEL
#define CABLE_MODEL

#include <ros/ros.h>
#include <iostream>
#include <ignition/math/Pose3.hh>
#include <cmath>


// # define M_PI 3.141592  /* pi */
double round_to(double value, double precision);


namespace cable_dynamics //si usa solitamente il nome del pacchetto dentro cui è messa la libreria
{
    class MassSpringDamping    //solitamente si da il nome della classe al file per immediatezza, prime lettere maiuscole attaccate.
    {
        public:

            MassSpringDamping();           //costruttore della classe, aggiungi parametri se vuoi che vengano dati dall'esterno quando crei l'oggetto
            MassSpringDamping(int num_of_points, float cable_length, float total_mass, float cable_diameter);
            virtual ~MassSpringDamping();  //distruttore

            void setDamperCoef(float K_d);
            void setYoungModulus(double young_modulus);
            void setPoissonRatio(double poisson_ratio);
            void setTorsionDamperCoef(float K_t);
            
            void updateMassAcceleration(ignition::math::Vector3d acc, int i);
            void updateMassVelocity(ignition::math::Vector3d velocity, int i);
            void updateAngularVelocity(ignition::math::Vector3d angular_velocity, int i);
            void setMassInitialPosition(ignition::math::Vector3d position, int i);
            void updateMassPosition(ignition::math::Vector3d position, int i);

            // void updateBeta(float beta_i, int i);
            
            void evaluateGravity();
            void computeInertiaForces();
            void computeDampingForces();
            void computeSpringsForces();
            void addInitialConstrainSpring();
            void addFinalConstrainSpring();
            ignition::math::Vector3d getResultantForce(int i);
            ignition::math::Vector3d getTwistingTorque(int i);
            void updateCableTwist(const ignition::math::Quaterniond&, const ignition::math::Quaterniond&);

            int num_of_masses, num_of_links;
        
        private:

            //class variables, lettere minuscole con "_" tipo: control_reference

            
            void updateBeta();
            void updatePsi();
            void updateRelativeVelocities();
            
            
            void twistingSpringForces();
            void linearSpringForces();
            void bendingSpringForces();

            
            

            void setLinearElasticCoef(double length);
            void setBendingElasticCoef(double length);
            void setTwistingElasticCoef(double length);
            void setMaxTensileStress(double length);

            ignition::math::Vector3d fixed_axis = {1.0, 0.0, 0.0};

            ignition::math::Vector3d getUnitVersor(int i);
            ignition::math::Vector3d getLinkLength(int i); 
            ignition::math::Vector3d tripleCross(int i, int j, int k);
            ignition::math::Vector3d tripleCross(ignition::math::Vector3d u1,
                                                        ignition::math::Vector3d u2,
                                                        ignition::math::Vector3d u3);
            double getBeta(ignition::math::Vector3d link_vec, ignition::math::Vector3d fixed_vec) ;

            std::vector<ignition::math::Vector3d> relative_velocities, mass_velocities, mass_accelerations, mass_angular_velocities;
            std::vector<ignition::math::Vector3d> damping_forces, inertia_forces;
            ignition::math::Vector3d material_twisting_angle;

            
            std::vector<ignition::math::Vector3d> linear_forces, bending_forces, twisting_forces;
            std::vector<ignition::math::Vector3d> mass_positions, mass_initial_positions;
            ignition::math::Vector3d gravity_acceleration, gravity_forces;
            
            float l0; //total cable length
            float total_mass;
            
            
            float discrete_mass, diameter;
            
            int count = 0;
            float linear_spring;
            float bending_spring;
            float twisting_spring;
            float damping_factor;
            float constrain_spring;
            float torsion_damper;
            std::vector<double> link_displacement; //l_i
            std::vector<float> beta, psi;


            double G, I_p; // shear modulus and polar moment of inertia
            double E, I, A; // Young modulus, moment of inertia and cross section area
            double F_max; // Forza prima della rottura

    };

}
#endif


