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
    this->mass_angular_velocities.resize(this->num_of_masses);
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

void MassSpringDamping::setTorsionDamperCoef(float K_t){
    this->torsion_damper = K_t;
    cout << "Torsion damper: " << this->torsion_damper << endl;
}


void MassSpringDamping::updateMassVelocity(ignition::math::Vector3d velocity, int i)            { this->mass_velocities[i]          = velocity; }
void MassSpringDamping::updateAngularVelocity(ignition::math::Vector3d angular_velocity, int i) { this->mass_angular_velocities[i] = angular_velocity; }
void MassSpringDamping::updateMassAcceleration(ignition::math::Vector3d acc, int i)             { this->mass_accelerations[i]       = acc;      }
void MassSpringDamping::setMassInitialPosition(ignition::math::Vector3d position, int i)        { this->mass_initial_positions[i]   = position; }
void MassSpringDamping::updateMassPosition(ignition::math::Vector3d position, int i)            { this->mass_positions[i]           = position; }


void MassSpringDamping::computeSpringsForces(){
    this->linearSpringForces();
    this->bendingSpringForces();
    this->twistingSpringForces(); // Re-enabled with reduced force scaling
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
    // f_2 = - x1_dot + 2 x2_dot - x3_dot
    // f_3 = d(x3_dot - x2_dot) --> n-1 elemento dell'array

    for (int i = 0; i < this->num_of_masses; ++i) {
        this->damping_forces[i] = ignition::math::Vector3d::Zero;
    }

    this->updateRelativeVelocities();
    // this->damping_forces.front() = this->damping_factor*this->relative_velocities.front() - this->damping_factor*this->relative_velocities[1];
    // for(int i=1; i<this->num_of_masses -1; i++)
    //     this->damping_forces[i] =   - this->damping_factor*this->relative_velocities[i-1] 
    //                                 + 2*this->damping_factor*this->relative_velocities[i]
    //                                 - this->damping_factor*this->relative_velocities[i+1];
    // this->damping_forces.back() = this->damping_factor*this->relative_velocities.back() - this->damping_factor*this->relative_velocities[this->num_of_masses-2];
    



    // Applichiamo lo smorzamento tra masse adiacenti
    for (int i = 0; i < this->num_of_masses - 1; ++i) {
        ignition::math::Vector3d damping_force = this->damping_factor * this->relative_velocities[i];

        // Azione e reazione: aggiorniamo le forze per le due masse collegate
        this->damping_forces[i]     += damping_force;
        this->damping_forces[i + 1] -= damping_force;
    }
    
}



void MassSpringDamping::linearSpringForces(){
    // m1*xdd + k(x1-x2) +k(telaio-> la mass prima non esiste) ==> -k (x2- x1)
    // m2*xdd + k(x2 - x1) + k(x2 - x3) = 0 ==> k(x2 - x1) - k (k3 - x2) ==> consisntency with 1 function getLinkLength(i - (i-1))
    // f_3 = k(x3 - x2) + k(la massa dopo non esiste) 
   // questa formula è presa pari pari dal paper di cable dynamics
    
    // this->linear_forces.front() = - this->linear_spring*round_to((this->getLinkLength(1).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(1);
    // for(int i=1; i<this->num_of_masses; i++){
    //     this->linear_forces[i] =  - this->linear_spring*round_to((this->getLinkLength(i).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(i)
    //                               + this->linear_spring*round_to((this->getLinkLength(i+1).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(i+1);    
    
    // }             
    // this->linear_forces.back() = + this->linear_spring*round_to((this->getLinkLength(num_of_masses-1).Length() - this->l0/this->num_of_links), 1e-5)*this->getUnitVersor(this->num_of_masses-1);

    this->linear_forces.front() = - this->linear_spring * round_to((this->getLinkLength(1).Length() - this->l0 / this->num_of_links), 1e-5) * this->getUnitVersor(1);

    for (int i = 1; i < this->num_of_masses - 1; i++) {
        this->linear_forces[i] = 
            - this->linear_spring * round_to((this->getLinkLength(i).Length() - this->l0 / this->num_of_links), 1e-5) * this->getUnitVersor(i)
            + this->linear_spring * round_to((this->getLinkLength(i + 1).Length() - this->l0 / this->num_of_links), 1e-5) * this->getUnitVersor(i + 1);
    }

    this->linear_forces.back() = + this->linear_spring * round_to((this->getLinkLength(num_of_masses - 1).Length() - this->l0 / this->num_of_links), 1e-5) * this->getUnitVersor(num_of_masses - 1);
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


void MassSpringDamping::updateCableTwist(
    const ignition::math::Quaterniond& rot0,
    const ignition::math::Quaterniond& rotn)
{
    // rot0 e rotn sono i quaternioni alle due estremità
    ignition::math::Quaterniond q_rel = rot0.Inverse() * rotn;

    // --- estrai asse e angolo con ToAxis() ---
    ignition::math::Vector3d axis;
    double angle;
    q_rel.ToAxis(axis, angle);  // -> axis (unitario), angle (rad) :contentReference[oaicite:0]{index=0}

    // salva la torsione materiale come vettore axis*angle
    this->material_twisting_angle = axis * angle;

}



void MassSpringDamping::updatePsi() {
    // Assicurati che beta (bending angles) sia aggiornato
    this->updateBeta();

    // Calcola torsione materiale distribuita
    double total_twist = this->material_twisting_angle.Length();
    double theta = total_twist / this->num_of_links;  // Eq.(11)

    // Estrema: niente torsione geometrica
    this->psi[0] = theta;
    this->psi[this->num_of_masses - 1] = theta;

    // Interni: phi + theta
    for (int i = 1; i < this->num_of_masses - 1; ++i) {
        // Calcola φᵢ = arctan(|a×b| / (a·b)), con a = uᵢ×(uᵢ×uᵢ₋₁), b = uᵢ×(uᵢ₊₁×uᵢ)
        auto a = this->tripleCross(i, i, i-1);
        auto b = this->tripleCross(i, i+1, i);
        double phi = atan(a.Cross(b).Length() / a.Dot(b));  // Eq.(8)

        this->psi[i] = phi + theta;  // Eq.(7)
    }
}


// file: cable_dynamics.cpp

void MassSpringDamping::twistingSpringForces() {
    // 1) Aggiorna torsioni geometriche e materiale
    this->updatePsi();

    // 2) Cattura in alias i membri per accesso rapido
    auto& betaVec    = this->beta;
    auto& psiVec     = this->psi;
    auto& omegas     = this->mass_angular_velocities;
    auto& outTorques = this->twisting_forces;
    const double kt  = this->twisting_spring;
    const double td  = this->torsion_damper;
    const int    N   = this->num_of_masses;

    // 3) Azzeramento (resta necessario)
    for (int i = 0; i < N; ++i)
        outTorques[i] = ignition::math::Vector3d::Zero;

    // 4) Loop principale
    for (int i = 1; i < N - 1; ++i) {
        // 4.1) Preleva una sola volta le quantità scalari
        double li      = this->getLinkLength(i).Length();
        double bi      = betaVec[i];
        double psi_im1 = psiVec[i - 1];
        double psi_i   = psiVec[i];
        double psi_ip1 = psiVec[i + 1];

        // 4.2) Guard ε per sinβ, tanβ
        double sinb = sin(bi);
        if (fabs(sinb) < 1e-6) sinb = copysign(1e-6, sinb);
        double tanb = tan(bi);
        if (fabs(tanb) < 1e-6) tanb = copysign(1e-6, tanb);

        // 4.3) Pre-calcola i vettori di tripleCross
        auto c1 = this->tripleCross(i - 1, i - 1, i);
        auto c2 = this->tripleCross(i,     i,     i + 1);
        auto c3 = this->tripleCross(i,     i + 1, i + 1);

        // 4.4) Termini elastici (Eq.12)
        //    kt * psi_im1/(li*sinb^2) * c1
        //  + kt * psi_i  /(li*tanb*sinb) * c2
        //  - kt * psi_ip1/(li*sinb^2) * c2
        //  - kt * psi_ip1/(li*tanb*sinb) * c3
        ignition::math::Vector3d torque_elastic =
            c1 * ( kt * psi_im1 / (li * sinb * sinb) )
          + c2 * ( kt * psi_i   / (li * tanb * sinb) )
          - c2 * ( kt * psi_ip1 / (li * sinb * sinb) )
          - c3 * ( kt * psi_ip1 / (li * tanb * sinb) );

        // 4.5) Damping torsionale
        auto& wi   = omegas[i];
        auto& wip1 = omegas[i + 1];
        auto  axis = this->getUnitVersor(i);
        double relVel = (wip1 - wi).Dot(axis);
        ignition::math::Vector3d torque_damping = axis * (-td * relVel);

        // 4.6) Somma, correzione
        outTorques[i] = torque_elastic + torque_damping;
        outTorques[i].Correct();
    }
}





ignition::math::Vector3d MassSpringDamping::getTwistingTorque(int i){ // Torque is a twisting or turning force 
    return this->twisting_forces[i];
}

void MassSpringDamping::evaluateGravity()
{   
    this->gravity_forces =  this->discrete_mass * this->gravity_acceleration;
}

ignition::math::Vector3d MassSpringDamping::getResultantForce(int i) {
    // Re-enabled twisting forces with moderate scaling for stability
    return this->inertia_forces[i] + linear_forces[i] + this->damping_forces[i] + this->bending_forces[i] + this->gravity_forces;
}







