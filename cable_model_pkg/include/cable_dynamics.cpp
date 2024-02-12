#include "cable_dynamics.hpp"


double round_to(double value, double precision){
    return std::round(value / precision) * precision;
}

using namespace cable_dynamics;
using namespace std;


MassSpringDamping::MassSpringDamping() {}

MassSpringDamping::MassSpringDamping(int num_of_masses, float cable_length, float total_mass, float cable_diameter)
    :A(M_PI*pow(cable_diameter, 2)/4)
    ,I(M_PI*pow(cable_diameter, 4)/64)
    ,I_p(M_PI*pow(cable_diameter, 4)/32)
    ,gravity_acceleration(ignition::math::Vector3d(0, 0, -9.81))
    ,gravity_forces(ignition::math::Vector3d(0, 0, 0))
{

    this->num_of_masses = num_of_masses;
    this->l0 = cable_length;
    this->total_mass = total_mass;

    this->diameter = cable_diameter;


    this->num_of_links = this->num_of_masses - 1;
    this->discrete_mass = this->total_mass/this->num_of_masses;


    this->mass_velocities.resize(this->num_of_masses);
    this->mass_accelerations.resize(this->num_of_masses);
    this->mass_positions.resize(this->num_of_masses);
    this->mass_initial_positions.resize(this->num_of_masses);
    

    this->relative_velocities.resize(this->num_of_links);
    this->damping_forces.resize(this->num_of_masses);



    this->linear_forces.resize(this->num_of_masses);
    this->inertia_forces.resize(this->num_of_masses);
    this->bending_forces.resize(this->num_of_masses);
    this->twisting_forces.resize(this->num_of_masses);
    this->link_displacement.resize(this->num_of_masses);

    this->beta.resize(this->num_of_masses);
    this->psi.resize(this->num_of_masses);
}

MassSpringDamping::~MassSpringDamping(){}




void MassSpringDamping::setYoungModulus(double young_modulus){ 
    this->E = young_modulus; 
    cout << "Young modulus: " << this->E << endl;
    this->setLinearElasticCoef(this->l0/this->num_of_links);
    this->setBendingElasticCoef(this->l0/this->num_of_links); // in general E_linear = E_bending, overmore initial_segm_length of bending is l_i = (|l_(i)| + |l_(i+1)|)/2 (Veronoi initial length)
    this->setMaxTensileStress(this->l0);
}

void MassSpringDamping::setMaxTensileStress(double length){
    double maximum_permissible_deformation = 1E-3; // mm
    double strain = (maximum_permissible_deformation)/this->l0;
    double tensile_stress = this->E*strain;
    this->F_max = tensile_stress*this->A;
    cout << "Maximum tensile stress: " << this->F_max << endl;
}

void MassSpringDamping::setPoissonRatio(double poisson_ratio){ 
    this->G = this->E/(2*(1+poisson_ratio)); 
    cout << "Shear Modulus: " << this->G << endl;
    this->setTwistingElasticCoef(this->l0/this->num_of_links); //take Veronoi initial length
}

void MassSpringDamping::setLinearElasticCoef(double initial_segm_length){
    this->linear_spring = this->E*this->A/initial_segm_length;
    // this->linear_spring = 100; /// chaange
    cout << "Linear spring: " << this->linear_spring << endl;
}

void MassSpringDamping::setBendingElasticCoef(double initial_segm_length){
    this->bending_spring = (this->E*this->I)/(initial_segm_length);
    this->constrain_spring = 3*this->bending_spring;
    cout << "Bending spring: " << this->bending_spring << endl;
}

void MassSpringDamping::setTwistingElasticCoef(double initial_segm_length){    
    this->twisting_spring = (this->G*this->I_p)/(initial_segm_length);
    cout << "Twisting spring: " << this->twisting_spring << endl;
}

void MassSpringDamping::setDamperCoef(float K_d){ 
    this->damping_factor = K_d;
    // this->damping_factor = 2*sqrt(this->linear_spring);
    }


void MassSpringDamping::updateMassVelocity(ignition::math::Vector3d velocity, int i)    { this->mass_velocities[i]          = velocity; }
void MassSpringDamping::updateMassAcceleration(ignition::math::Vector3d acc, int i)     { this->mass_accelerations[i]       = acc;      }
void MassSpringDamping::setMassInitialPosition(ignition::math::Vector3d position, int i){ this->mass_initial_positions[i]   = position; }
void MassSpringDamping::updateMassPosition(ignition::math::Vector3d position, int i)    { this->mass_positions[i]           = position; }
// void MassSpringDamping::updateBeta(float beta_i, int i) { this->beta[i] = beta_i; }


void MassSpringDamping::computeSpringsForces(){
    this->linearSpringForces();
    this->bendingSpringForces();
    this->twistingSpringForces();
}
void MassSpringDamping::computeInertiaForces(){ 
    for(int i=0; i<num_of_masses; i++)
        this->inertia_forces[i] = discrete_mass *this->mass_accelerations[i];
}

void MassSpringDamping::updateRelativeVelocities(){
    for(int i=0; i<this->num_of_links; i++){
        this->relative_velocities[i] = this->mass_velocities[i+1] - this->mass_velocities[i];
        // cout << "Link " << i << " relative velocity: " << MassSpringDamping::relative_velocities[i] << endl;
    }
}

void MassSpringDamping::computeDampingForces() {
    // m2*x_dotdot + d(x2_dot - x1_dot) + d(x2_dot - x3_dot) = 0 ==> mx_dotdot +  d(x2_dot - x1_dot) + d(x2_dot - x3_dot)
    // f_1 = d(x1_dot - x2_dot) --> 0 elemento
    // f_2_d = - x1_dot + 2 x2_dot - x3_dot
    // f_3 = d(x3_dot - x2_dot) --> n-1 elemento dell'array
    this->updateRelativeVelocities();
    this->damping_forces.front() = this->damping_factor*this->relative_velocities.front() - this->damping_factor*this->relative_velocities[1];
    for(int i=1; i<this->num_of_masses -1; i++)
        this->damping_forces[i] =   - this->damping_factor*this->relative_velocities[i-1] 
                                    + 2*this->damping_factor*this->relative_velocities[i]
                                    - this->damping_factor*this->relative_velocities[i+1];
    this->damping_forces.back() = this->damping_factor*this->relative_velocities.back() - this->damping_factor*this->relative_velocities[this->num_of_masses-2];

    //PRINT
    // for(int i=0; i<this->num_of_masses; i++)
        // std::cout << "Damping force " << i << ": " << this->damping_forces[i] << std::endl;
}



void MassSpringDamping::linearSpringForces(){
    // m1*xdd + k(x1-x2) +k(telaio-> la mass prima non esiste) ==> -k (x2- x1)
    // m2*xdd + k(x2 - x1) + k(x2 - x3) = 0 ==> k(x2 - x1) - k (k3 - x2) ==> consisntency with 1 function getLinkLength(i - (i-1))
    // f_3 = k(x3 - x2) + k(la massa dopo non esiste) 
   // questa formula è presa pari pari dal paper di cable dynamics
    
    this->linear_forces.front() = - this->linear_spring*round_to((this->getLinkLength(1).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(1);
    for(int i=1; i<this->num_of_masses; i++){
        this->linear_forces[i] =  -this->linear_spring*round_to((this->getLinkLength(i).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(i)
                                 +this->linear_spring*round_to((this->getLinkLength(i+1).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(i+1);    
    
    }             
    this->linear_forces.back() = + this->linear_spring*round_to((this->getLinkLength(num_of_masses-1).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(this->num_of_masses-1);
    
    // cable_tension_right = ( abs(cable_tension_right) < this->F_max) ? cable_tension_right : this->F_max;



    
    
    
    //PRINT
    // for(int i=0; i<this->num_of_masses; i++)
        // std::cout << "Linear force " << i << ": " << this->linear_forces[i] << std::endl;
}

void MassSpringDamping::updateBeta(){
    this->beta.front() = 0.0;
    for(int i=1; i<this->num_of_masses -1; i++)
        this->beta[i] = atan(
                            ( (this->mass_positions[i+1] - this->mass_positions[i]).Cross(this->mass_positions[i] - this->mass_positions[i-1]) ).Length()
                            /
                            ( ((this->mass_positions[i+1] - this->mass_positions[i])).Dot(this->mass_positions[i] - this->mass_positions[i-1]) )
                            );

    this->beta.back() = 0.0;
    //PRINT
    // for(int i=0; i<this->num_of_masses; i++)
    //     std::cout << "Beta" << i << ": " << this->beta[i] << std::endl;
}

ignition::math::Vector3d MassSpringDamping::getLinkLength(int i) {
    return this->mass_positions[i] - mass_positions[i-1];
}

ignition::math::Vector3d MassSpringDamping::getUnitVersor(int i) {
    ignition::math::Vector3d vector = this->getLinkLength(i);
    return vector / vector.Length();
}

ignition::math::Vector3d MassSpringDamping::tripleCross(int i, int j, int k) {
    ignition::math::Vector3d u1, u2, u3;
    u1 = this->getUnitVersor(i);
    u2 = this->getUnitVersor(j);
    u3 = this->getUnitVersor(k);
    return u1.Cross(u2.Cross(u3));
}





void MassSpringDamping::bendingSpringForces(){
    this->updateBeta();
    this->bending_forces[0] =     ( this->bending_spring*this->beta[1] / this->getLinkLength(1).Length() )   * ( this->tripleCross(1, 1, 2) / sin(this->beta[1]) );

    this->bending_forces[1] =   - ( this->bending_spring * this->beta[1] / this->getLinkLength(1).Length() ) * ( this->tripleCross(1, 1, 2) / sin(this->beta[1]) )
                                - ( this->bending_spring * this->beta[1] / this->getLinkLength(2).Length() ) * ( this->tripleCross(2, 1, 2) / sin(this->beta[1]) )
                                + ( this->bending_spring * this->beta[2] / this->getLinkLength(2).Length() ) * ( this->tripleCross(2, 2, 3) / sin(this->beta[2]) );

    for (int i=2;i<this->num_of_masses-2;i++){
        this->bending_forces[i] =   ( this->bending_spring * this->beta[i-1]  / this->getLinkLength(i).Length()   ) * ( this->tripleCross(i, i-1, i)     / sin(this->beta[i-1]))
                                  - ( this->bending_spring * this->beta[i]    / this->getLinkLength(i).Length()   ) * ( this->tripleCross(i, i, i+1)     / sin(this->beta[i])  )
                                  - ( this->bending_spring * this->beta[i]    / this->getLinkLength(i+1).Length() ) * ( this->tripleCross(i+1, i, i+1)   / sin(this->beta[i])  )
                                  + ( this->bending_spring * this->beta[i+1]  / this->getLinkLength(i+1).Length() ) * ( this->tripleCross(i+1, i+1, i+2) / sin(this->beta[i+1]));

    }

    this->bending_forces[num_of_masses - 2] =    ( this->bending_spring * this->beta[num_of_masses - 3]  / this->getLinkLength(num_of_masses - 2).Length() ) * ( this->tripleCross(num_of_masses - 2, num_of_masses - 3, num_of_masses - 2) / sin(this->beta[num_of_masses - 3]))
                                               - ( this->bending_spring * this->beta[num_of_masses - 2]  / this->getLinkLength(num_of_masses - 2).Length() ) * ( this->tripleCross(num_of_masses - 2, num_of_masses - 2, num_of_masses - 1) / sin(this->beta[num_of_masses - 2]))
                                               - ( this->bending_spring * this->beta[num_of_masses - 2]  / this->getLinkLength(num_of_masses - 1).Length() ) * ( this->tripleCross(num_of_masses - 1, num_of_masses - 2, num_of_masses - 1) / sin(this->beta[num_of_masses - 2]));

    this->bending_forces.back() = (this->bending_spring * this->beta[num_of_masses -2] / this->getLinkLength(num_of_masses - 1).Length()) * tripleCross(num_of_masses - 1, num_of_masses - 2, num_of_masses - 1) /sin(this->beta[num_of_masses - 1]);
    //   ignition::math::Vector3d::Zero;
    for(int i=0; i<this->num_of_masses; i++){
        //if (!this->bending_forces[i].IsFinite())  //*See if a point is finite (e.g., not nan)
        this->bending_forces[i].Correct();
    }


}

ignition::math::Vector3d MassSpringDamping::tripleCross(ignition::math::Vector3d u1,
                                                        ignition::math::Vector3d u2,
                                                        ignition::math::Vector3d u3) {
    return u1.Cross(u2.Cross(u3));
}

double MassSpringDamping::getBeta(ignition::math::Vector3d link_vec, ignition::math::Vector3d fixed_vec) {
  return atan((link_vec.Cross(fixed_vec)).Length() / link_vec.Dot(fixed_vec));
}

void MassSpringDamping::addInitialConstrainSpring(){

    this->bending_forces[1]  += ( this->constrain_spring * this->getBeta(getLinkLength(1), fixed_axis) / this->getLinkLength(1).Length() ) * ( this->tripleCross(getUnitVersor(1), fixed_axis, getUnitVersor(1)) / sin(this->getBeta(getLinkLength(1), fixed_axis)) );
    this->bending_forces[1].Correct();
    // this->linear_forces[0]= this->bending_forces[1];
}

void MassSpringDamping::addFinalConstrainSpring()
{
    //penuiltimo elemento
    this->bending_forces[num_of_masses - 2]  += ( this->constrain_spring * this->getBeta(fixed_axis, -getLinkLength(num_of_masses - 1)) / this->getLinkLength(num_of_masses - 2).Length() ) * ( this->tripleCross(getUnitVersor(num_of_masses - 1), getUnitVersor(num_of_masses - 1), fixed_axis) / sin(this->getBeta(fixed_axis, -getLinkLength(num_of_masses - 1))) );
    // if (this->bending_forces[1].X()>1) this->bending_forces[1].X() = 1;
    this->bending_forces[num_of_masses - 2].Correct();
    // this->linear_forces[num_of_masses - 1] = this->bending_forces[num_of_masses - 2];
}

void MassSpringDamping::updateCableTwist(ignition::math::Vector3d twist_angle_0, ignition::math::Vector3d twist_angle_n){
    this->material_twisting_angle = twist_angle_n - twist_angle_0;
}


void MassSpringDamping::updatePsi(){
    ignition::math::Vector3d a,b;
    float phi, theta;
    
    theta = 0; // = this->material_twisting_angle.Length() / this->num_of_masses; // Da calcolare
    // cout << "theta: " << theta << endl;
    this->psi[0] = theta;
    // a = this->getLinkLength(1).Cross(this->getLinkLength(1));
    // b = this->getLinkLength(1).Cross(this->getLinkLength(2).Cross(this->getLinkLength(1)));
    // phi = atan(a.Cross(b).Length() / a.Dot(b));
    
    this->psi[1] = theta;
    for (int i=2; i<this->num_of_masses-1; i++){
        a = tripleCross(getLinkLength(i), getLinkLength(i), getLinkLength(i-1)); // this->getLinkLength(i).Cross(this->getLinkLength(i).Cross(this->getLinkLength(i-1)));
        b = tripleCross(getLinkLength(i), getLinkLength(i+1), getLinkLength(i));  // this->getLinkLength(i).Cross(this->getLinkLength(i+1).Cross(this->getLinkLength(i)));
        phi = atan(a.Cross(b).Length() / a.Dot(b));
        
        // phi = 0;
        // cout << "phi"<< i << ": " << phi << endl;
        this->psi[i] = phi + theta;
    }

    a = this->tripleCross(getLinkLength(num_of_masses-1), getLinkLength(num_of_masses-1), getLinkLength(num_of_masses-2));
    b = this->getLinkLength(num_of_masses-1).Cross(this->getLinkLength(num_of_masses-1));
    phi = atan(a.Cross(b).Length() / a.Dot(b));
    this->psi[num_of_masses-1] = phi + theta;
}

void MassSpringDamping::twistingSpringForces(){
    this->updatePsi();
    this->twisting_forces[0] = 0;
    // this->twisting_forces[0]    = ( (this->twisting_spring * this->psi[0] ) / (this->getLinkLength(1).Length() * sin(this->beta[0])) ) * (this->getUnitVersor(0).Cross(this->getUnitVersor(1)) / sin(this->beta[0]))
    //                             + ( (this->twisting_spring * this->psi[0] ) / (this->getLinkLength(0).Length() * tan(this->beta[0])) ) * (this->getUnitVersor(0).Cross(this->getUnitVersor(1)) / sin(this->beta[0]))
    //                             - ( (this->twisting_spring * this->psi[1] ) / (this->getLinkLength(1).Length() * sin(this->beta[1])) ) * (this->getUnitVersor(0).Cross(this->getUnitVersor(1)) / sin(this->beta[0])   )
    //                             - ( (this->twisting_spring * this->psi[1] ) / (this->getLinkLength(1).Length() * tan(this->beta[0])) ) * (this->getUnitVersor(0).Cross(this->getUnitVersor(1)) / sin(this->beta[0])   )
    //                             - ( (this->twisting_spring * this->psi[1] ) / (this->getLinkLength(1).Length() * tan(this->beta[1])) ) * (this->getUnitVersor(1).Cross(this->getUnitVersor(2)) / sin(this->beta[1]) )
    //                             + ( (this->twisting_spring * this->psi[2] ) / (this->getLinkLength(1).Length() * sin(this->beta[1])) ) * (this->getUnitVersor(1).Cross(this->getUnitVersor(2)) / sin(this->beta[1]) );
//    this->twisting_forces[1] =         ( (this->twisting_spring * this->psi[1]   ) / (this->getLinkLength(1+1).Length() * sin(this->beta[1]))   ) * (this->getUnitVersor(1).Cross(this->getUnitVersor(1+1))   / sin(this->beta[1])   )
//                                     + ( (this->twisting_spring * this->psi[1]   ) / (this->getLinkLength(1).Length()   * tan(this->beta[1]))   ) * (this->getUnitVersor(1).Cross(this->getUnitVersor(1+1))   / sin(this->beta[1])   )
//                                     - ( (this->twisting_spring * this->psi[1+1] ) / (this->getLinkLength(1).Length()   * sin(this->beta[1]))   ) * (this->getUnitVersor(1).Cross(this->getUnitVersor(1+1))   / sin(this->beta[1])   )
//                                     - ( (this->twisting_spring * this->psi[1+1] ) / (this->getLinkLength(1+1).Length() * tan(this->beta[1]))   ) * (this->getUnitVersor(1).Cross(this->getUnitVersor(1+1))   / sin(this->beta[1])   )
//                                     - ( (this->twisting_spring * this->psi[1+1] ) / (this->getLinkLength(1+1).Length() * tan(this->beta[1+1])) ) * (this->getUnitVersor(1+1).Cross(this->getUnitVersor(1+2)) / sin(this->beta[1+1]) )
//                                     + ( (this->twisting_spring * this->psi[1+2] ) / (this->getLinkLength(1+1).Length() * sin(this->beta[1+1])) ) * (this->getUnitVersor(1+1).Cross(this->getUnitVersor(1+2)) / sin(this->beta[1+1]) ); 

    for(int i=2; i<this->num_of_masses-2; i++){
        // cout << this->psi[i] << endl;
        this->twisting_forces[i] =  - ( (this->twisting_spring * this->psi[i-1] ) / (this->getLinkLength(i).Length()   * sin(this->beta[i-1])) ) * (this->getUnitVersor(i-1).Cross(this->getUnitVersor(i))   / sin(this->beta[i-1]) )
                                    + ( (this->twisting_spring * this->psi[i]   ) / (this->getLinkLength(i+1).Length() * sin(this->beta[i]))   ) * (this->getUnitVersor(i).Cross(this->getUnitVersor(i+1))   / sin(this->beta[i])   )
                                    + ( (this->twisting_spring * this->psi[i]   ) / (this->getLinkLength(i).Length()   * tan(this->beta[i]))   ) * (this->getUnitVersor(i).Cross(this->getUnitVersor(i+1))   / sin(this->beta[i])   )
                                    + ( (this->twisting_spring * this->psi[i]   ) / (this->getLinkLength(i).Length()   * tan(this->beta[i-1])) ) * (this->getUnitVersor(i-1).Cross(this->getUnitVersor(i))   / sin(this->beta[i-1]) )
                                    - ( (this->twisting_spring * this->psi[i+1] ) / (this->getLinkLength(i).Length()   * sin(this->beta[i]))   ) * (this->getUnitVersor(i).Cross(this->getUnitVersor(i+1))   / sin(this->beta[i])   )
                                    - ( (this->twisting_spring * this->psi[i+1] ) / (this->getLinkLength(i+1).Length() * tan(this->beta[i]))   ) * (this->getUnitVersor(i).Cross(this->getUnitVersor(i+1))   / sin(this->beta[i])   )
                                    - ( (this->twisting_spring * this->psi[i+1] ) / (this->getLinkLength(i+1).Length() * tan(this->beta[i+1])) ) * (this->getUnitVersor(i+1).Cross(this->getUnitVersor(i+2)) / sin(this->beta[i+1]) )
                                    + ( (this->twisting_spring * this->psi[i+2] ) / (this->getLinkLength(i+1).Length() * sin(this->beta[i+1])) ) * (this->getUnitVersor(i+1).Cross(this->getUnitVersor(i+2)) / sin(this->beta[i+1]) );
    }

    // this->twisting_forces[this->num_of_masses-2] =  - ( (this->twisting_spring * this->psi[this->num_of_masses - 3] ) / (this->getLinkLength(this->num_of_masses - 2).Length() * sin(this->beta[this->num_of_masses - 3])) ) * (this->getUnitVersor(this->num_of_masses - 3).Cross(this->getUnitVersor(this->num_of_masses -  2)) / sin(this->beta[this->num_of_masses - 3]) )
    //                                                 + ( (this->twisting_spring * this->psi[this->num_of_masses - 2] ) / (this->getLinkLength(this->num_of_masses - 1).Length() * sin(this->beta[this->num_of_masses - 2])) ) * (this->getUnitVersor(this->num_of_masses - 2).Cross(this->getUnitVersor(this->num_of_masses -  1)) / sin(this->beta[this->num_of_masses - 2]) )
    //                                                 + ( (this->twisting_spring * this->psi[this->num_of_masses - 2] ) / (this->getLinkLength(this->num_of_masses - 2).Length() * tan(this->beta[this->num_of_masses - 2])) ) * (this->getUnitVersor(this->num_of_masses - 2).Cross(this->getUnitVersor(this->num_of_masses -  1)) / sin(this->beta[this->num_of_masses - 2]) )
    //                                                 + ( (this->twisting_spring * this->psi[this->num_of_masses - 2] ) / (this->getLinkLength(this->num_of_masses - 2).Length() * tan(this->beta[this->num_of_masses - 3])) ) * (this->getUnitVersor(this->num_of_masses - 3).Cross(this->getUnitVersor(this->num_of_masses -  2)) / sin(this->beta[this->num_of_masses - 3]) )
    //                                                 - ( (this->twisting_spring * this->psi[this->num_of_masses - 1] ) / (this->getLinkLength(this->num_of_masses - 2).Length() * sin(this->beta[this->num_of_masses - 2])) ) * (this->getUnitVersor(this->num_of_masses - 2).Cross(this->getUnitVersor(this->num_of_masses -  1)) / sin(this->beta[this->num_of_masses - 2]) )
    //                                                 - ( (this->twisting_spring * this->psi[this->num_of_masses - 1] ) / (this->getLinkLength(this->num_of_masses - 1).Length() * tan(this->beta[this->num_of_masses - 2])) ) * (this->getUnitVersor(this->num_of_masses - 2).Cross(this->getUnitVersor(this->num_of_masses -  1)) / sin(this->beta[this->num_of_masses - 2]) );
                                                    

                                                    
    // this->twisting_forces[this->num_of_masses-1] = - ( (this->twisting_spring * this->psi[this->num_of_masses - 2] ) / (this->getLinkLength(this->num_of_masses - 1).Length() * sin(this->beta[this->num_of_masses - 2])) ) * (this->getUnitVersor(this->num_of_masses - 2).Cross(this->getUnitVersor(this->num_of_masses - 1)) / sin(this->beta[this->num_of_masses - 2]) )
    //                                                + ( (this->twisting_spring * this->psi[this->num_of_masses - 1]  ) / (this->getLinkLength(this->num_of_masses - 1).Length() * tan(this->beta[this->num_of_masses - 2])) ) * (this->getUnitVersor(this->num_of_masses - 2).Cross(this->getUnitVersor(this->num_of_masses - 1)) / sin(this->beta[this->num_of_masses - 2]) );

    for(int i=0; i<this->num_of_masses; i++){
        if (!this->bending_forces[i].IsFinite()) this->twisting_forces[i] = 0; //*See if a point is finite (e.g., not nan)
        this->twisting_forces[i].Correct();
    }

}

ignition::math::Vector3d MassSpringDamping::getTwistingForce(int i){ // Torque is a twisting or turning force 
    // cout << "Twisting force " << i << ": "   << endl;
    return this->twisting_forces[i];
}

void MassSpringDamping::evaluateGravity()
{   
    this->gravity_forces =  this->discrete_mass * this->gravity_acceleration;
}

ignition::math::Vector3d MassSpringDamping::getResultantForce(int i) {
    // cout << "Damping force " << i << ": " << this->damping_forces[i] << endl;
    // cout << "Linear force " << "0" << ": " << linear_forces[0] << endl;
    // cout << "Bending force " << i << ": " << this->bending_forces[i] << endl;
    // cout << "Twisting force " << i << ": " << this->twisting_forces[i] << endl;
    // cout << "Result Forces" << i << ": " << this->damping_forces[i] + linear_forces[i] + this->bending_forces[i] /*+ this->twisting_forces[i]*/ << endl;
    // cout << "Result Forces" << " 0" << ": " << this->damping_forces[0] + linear_forces[0] + this->bending_forces[0] /*+ this->twisting_forces[i]*/ << endl;
    // cout << "Result Forces" << " 8" << ": " << this->damping_forces[8] + linear_forces[8] + this->bending_forces[8] /*+ this->twisting_forces[i]*/ << endl;
    // cout << "Gravity" << ": " << this->gravity_forces << endl;
    // cout << "Intertia Forces " << i << ": " << this->inertia_forces[i] << endl;
    
    return this->inertia_forces[i] + linear_forces[i] + this->damping_forces[i] + this->bending_forces[i] + this->gravity_forces;//+ this->twisting_forces[i]/1000000;
}







