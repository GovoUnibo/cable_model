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
        MassSpringDamping::setMassInitialPosition(cable_masses[i].getInitialRelPosition(), i); // set initial position
    }
    // cout << "Mass Distance: " << (cable_masses[29].getInitialRelPosition() - cable_masses[28].getInitialRelPosition()).Length() << endl;
    // cout << "Initial displacement: " << round_to((cable_masses[29].getInitialRelPosition() - cable_masses[28].getInitialRelPosition()).Length() - length/(num_masses - 1), 1e-4) << endl;

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
void Cable::setTorsionDamperCoef(float K_t){ MassSpringDamping::setTorsionDamperCoef(K_t);}



const gazebo::physics::LinkPtr Cable::getLink(int i) { return cable_masses[i].getLink(); }

int Cable::getResolution(){ return num_masses; }

ignition::math::Vector3d Cable::getMassPos(int i) { /*wrt world*/  return cable_masses[i].getAbsolutePosition(); }
ignition::math::Vector3d Cable::getLeftNeighbourPos(int i) {  /*wrt world*/ return cable_masses[i-1].getAbsolutePosition(); }
ignition::math::Vector3d Cable::getRightNeighbourPos(int i) { /*wrt world*/ return cable_masses[i+1].getAbsolutePosition(); }
ignition::math::Vector3d Cable::getMassInitialPos(int i) { /*wrt world*/  return cable_masses[i].getInitialRelPosition(); }

ignition::math::Vector3d Cable::getMassVel(int i) { /*wrt world*/ return cable_masses[i].getAbsoluteVelocity(); }
ignition::math::Vector3d Cable::getLeftNeighbourVel(int i) { /*wrt world*/ return cable_masses[i-1].getAbsoluteVelocity(); }
ignition::math::Vector3d Cable::getRightNeighbourVel(int i) { /*wrt world*/ return cable_masses[i+1].getAbsoluteVelocity(); }

ignition::math::Vector3d Cable::getVector(int i) {     return this->getMassPos(i) - this->getMassPos(i-1);}

double Cable::getBeta(int i) {   return atan((this->getVector(i+1).Cross(this->getVector(i))).Length() / this->getVector(i+1).Dot(this->getVector(i)));}

ignition::math::Vector3d Cable::getParticlesRelativePos(int i, int j) {     return cable_masses[i].getLink()->WorldPose().Pos() - cable_masses[j].getLink()->WorldPose().Pos();}

ignition::math::Vector3d Cable::tripleCross(ignition::math::Vector3d u1, ignition::math::Vector3d u2, ignition::math::Vector3d u3) {    return u1.Cross(u2.Cross(u3));}

ignition::math::Vector3d Cable::getForceWrtWorld(int i){    return this->cable_masses[i].getForce();}
ignition::math::Vector3d Cable::getPositionWrtWorld(int i){    return this->cable_masses[i].getAbsolutePosition();}
ignition::math::Vector3d Cable::getRotationWrtWorld(int i){    return this->cable_masses[i].getAbsoluteRotationEuler();}
ignition::math::Vector3d Cable::getVelocityWrtWorld(int i){    return this->getMassVel(i);}

void Cable::setFirstMassFixed(bool is_fixed){  cable_masses[0].setFixed(is_fixed); }
void Cable::setLastMassFixed(bool is_fixed){  cable_masses[num_masses-1].setFixed(is_fixed); }
void Cable::setFirstMassGrasped(bool is_grasped){ cable_masses[0].setGrasped(is_grasped); }
void Cable::setLastMassGrasped(bool is_grasped){ this->cable_masses[num_masses-1].setGrasped(is_grasped); }

void Cable::setMassPosition(int i, ignition::math::Vector3d position, ignition::math::Quaterniond rotation) {
    this->cable_masses[i].updatePosition(ignition::math::Pose3d{position, rotation});
}

void Cable::alignMassFrames()
{
  bool grasp0 = this->cable_masses[0].isGrasped();        // :contentReference[oaicite:1]{index=1}
  bool graspN = this->cable_masses[num_masses-1].isGrasped();// :contentReference[oaicite:2]{index=2}

  // rotazioni ancorate
  ignition::math::Quaterniond q0 = this->cable_masses[0].getAbsoluteRotationQuaternion();
  ignition::math::Quaterniond qN = this->cable_masses.back().getAbsoluteRotationQuaternion();

  for (int i = 0; i < this->num_masses; ++i)
  {
    // 1) calcolo asse tra masse
    auto pos_i = cable_masses[i].getAbsolutePosition();
    ignition::math::Vector3d axis;
    if (i < this->num_masses - 1)
      axis = cable_masses[i+1].getAbsolutePosition() - pos_i;
    else
      axis = pos_i - cable_masses[i-1].getAbsolutePosition();
    axis.Normalize();

    // 2) quaternion che porta UnitX → axis
    ignition::math::Vector3d v0 = ignition::math::Vector3d::UnitX;
    double cosTheta = v0.Dot(axis);
    ignition::math::Quaterniond rot_local;
    if (cosTheta < -1.0 + 1e-6) {
      auto ortho = v0.Cross(ignition::math::Vector3d::UnitY);
      if (ortho.Length() < 1e-6)
        ortho = v0.Cross(ignition::math::Vector3d::UnitZ);
      ortho.Normalize();
      rot_local = ignition::math::Quaterniond(ortho, M_PI);
    } else {
      auto rotAxis = v0.Cross(axis);
      rotAxis.Normalize();
      rot_local = ignition::math::Quaterniond(rotAxis, std::acos(cosTheta));
    }

    // 3) decido rotazione di ancoraggio
    ignition::math::Quaterniond anchor_q;
    if (grasp0 && graspN) {
      double t = double(i) / double(num_masses - 1);
      anchor_q = ignition::math::Quaterniond::Slerp(t, q0, qN);
    }
    else if (grasp0) {
      anchor_q = q0;
    }
    else if (graspN) {
      anchor_q = qN;
    }
    else {
      // nessun vincolo: comportamento “classico”
      cable_masses[i].updatePosition({pos_i, rot_local});
      continue;
    }

    // 4) composizione finale e update
    auto final_q = anchor_q * rot_local;
    cable_masses[i].updatePosition({pos_i, final_q});
  }
}


bool Cable::isMassFix(int i){     return this->cable_masses[i].isFixed(); }
bool Cable::isMassGrasped(int i){ return this->cable_masses[i].isGrasped(); }

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
        MassSpringDamping::updateAngularVelocity(cable_masses[i].getAbsoluteAngularVelocity(), i);
        MassSpringDamping::updateMassAcceleration(cable_masses[i].getAbosulteAcceleration(), i);
    }


    MassSpringDamping::updateCableTwist(this->cable_masses[0].getAbsoluteRotationQuaternion(), this->cable_masses[num_masses-1].getAbsoluteRotationQuaternion());


    MassSpringDamping::computeSpringsForces();
    MassSpringDamping::computeDampingForces();
    // MassSpringDamping::computeInertiaForces(); // gazebo le calcola automaticamente cout << cable_masses[2].getForce() << endl;
    // MassSpringDamping::evaluateGravity();
    // cout << cable_masses[2].getForce() << endl;

    if (cable_masses[0].isFixed() || cable_masses[0].isGrasped())
        MassSpringDamping::addInitialConstrainSpring();

    if (cable_masses[num_masses-1].isFixed() || cable_masses[num_masses-1].isGrasped())
        MassSpringDamping::addFinalConstrainSpring();


       
    for(int i=0; i< MassSpringDamping::num_of_masses; i++){
        cable_masses[i].updateForce(MassSpringDamping::getResultantForce(i));

        auto rawT = MassSpringDamping::getTwistingTorque(i);
        ignition::math::Vector3d tangent;
        if (i < this->num_masses - 1)
            tangent = this->cable_masses[i+1].getAbsolutePosition()
                    - this->cable_masses[i].getAbsolutePosition();
        else
            tangent = this->cable_masses[i].getAbsolutePosition()
                    - this->cable_masses[i-1].getAbsolutePosition();
        tangent.Normalize();

        // proiezione del torque lungo la tangente
        double mag = rawT.Dot(tangent);
        ignition::math::Vector3d Tproj = tangent * mag;

        // applichiamo il momento puro proiettato
        // this->cable_masses[i].updateTorque(Tproj);
    }


}



double Cable::getpanicAdaptiveDt(double baseDt) {
  // ---- Lettura posizioni e parametri geometrici ----
  const auto pos   = this->MassSpringDamping::getMassPositions();
  const int  N     = static_cast<int>(pos.size());
  if (N < 2 || this->num_of_links <= 0) return baseDt;

  const double L0seg = static_cast<double>(this->MassSpringDamping::getMassToMassDistance());

  // ---- Parametri fisici/rigidezze ----
  const double k_lin  = static_cast<double>(this->MassSpringDamping::getCableLinearSpring());
  const double k_bend = static_cast<double>(this->MassSpringDamping::getCableBendingSpring());
  const double k_tw   = static_cast<double>(this->MassSpringDamping::getCableTwistingSpring());
  const double m      = static_cast<double>(this->MassSpringDamping::getCableDiscretizedMass());

  // dt "massimo taglio" (paper): dtPaper = sqrt(m * L0seg / k_max). Se mancano dati, resta prudente.
  const double k_max = std::max(k_lin, std::max(k_bend, k_tw));
  double dtPaper = baseDt;
  if (m > 0.0 && L0seg > 0.0 && k_max > 0.0) {
    dtPaper = std::sqrt((m * L0seg) / k_max);
  }
  // floor RT di sicurezza
  const double dtFloor = std::max(baseDt * 0.005, 1e-6); // 0.5% del baseDt o 1e-6 s
  dtPaper = std::clamp(dtPaper, dtFloor, baseDt);

  // ---- Soglie di FORZA (tensione & "compressione") ----
  const double E    = static_cast<double>(this->MassSpringDamping::getCableYoungModulus());
  const double A    = static_cast<double>(this->MassSpringDamping::getCableCrossSectionArea());
  const double Fmax = static_cast<double>(this->MassSpringDamping::getCableMaxTensileStress());

  // Tensione: se Fmax (forza) è noto, usalo; altrimenti E*A*2% come fallback.
  const double F_TENS_THRESH =
      (Fmax > 0.0) ? Fmax
                   : ((E > 0.0 && A > 0.0) ? (E * A * 0.02) : std::numeric_limits<double>::infinity());

  // "Compressione": il cavo idealmente non porta compressione; usiamo una soglia pseudo-forza
  // proporzionale a quanto permetti di accorciare: F_c_thresh = k_lin * (COMPRESS_PCT * L0seg)
  const double COMPRESS_PCT  = 0.30; // consenti fino al 30% di accorciamento
  const double F_COMP_THRESH =
      (k_lin > 0.0 && L0seg > 0.0) ? (k_lin * (COMPRESS_PCT * L0seg))
                                   : std::numeric_limits<double>::infinity();

  // ---- Misura della forza assiale massima per link (tensione e compressione) ----
  double F_tens_max = 0.0;
  double F_comp_max = 0.0;

  for (int i = 1; i < N; ++i) {
    const double L = (pos[i] - pos[i - 1]).Length();
    if (!std::isfinite(L)) continue;
    const double dl = L - L0seg;

    // modello assiale elementare: F = k_lin * dl
    if (dl >= 0.0) {
      F_tens_max = std::max(F_tens_max, k_lin * dl);
    } else {
      // "pseudo" forza compressiva: quanto vorrebbe spingere la molla lineare
      F_comp_max = std::max(F_comp_max, k_lin * (-dl));
    }
  }

  // ---- Rapporto rispetto alle soglie ----
  const double rF_t = (std::isfinite(F_TENS_THRESH)  && F_TENS_THRESH  > 0.0) ? (F_tens_max / F_TENS_THRESH)  : 0.0;
  const double rF_c = (std::isfinite(F_COMP_THRESH) && F_COMP_THRESH > 0.0) ? (F_comp_max / F_COMP_THRESH) : 0.0;
  const double r    = std::max(rF_t, rF_c); // misura "quanto sei oltre" (o vicino) al limite

  // ---- Finestra di scalatura: appena superi la soglia (r0) inizi a scalare fino a dtPaper a r1 ----
  const double r0 = 1.0;  // SOGLIA: appena F supera la soglia, inizia a ridurre
  const double r1 = 2.0;  // quando F è 2× la soglia, sei al taglio massimo (dtPaper)

  double dt = baseDt;
  if (r <= r0) {
    dt = baseDt; // sotto soglia: non toccare
  } else if (r >= r1) {
    dt = dtPaper; // oltre finestra: taglio massimo
  } else {
    // dentro la finestra: blend lineare da baseDt -> dtPaper
    const double t = (r - r0) / (r1 - r0); // 0..1
    dt = baseDt * (1.0 - t) + dtPaper * t;
  }

  // clamp finale
  dt = std::clamp(dt, dtPaper, baseDt);

  // logging (saltuario) quando stai tagliando
  static int cnt = 0;
  if (r > r0 && ((cnt++ % 30) == 0)) {
    std::cout << "[cable FORCE scale] F_t(max)=" << F_tens_max << " / " << F_TENS_THRESH
              << "  F_c(max)=" << F_comp_max << " / " << F_COMP_THRESH
              << "  r=" << r << "  dtPaper=" << dtPaper << "  dt=" << dt << std::endl;
  }

  return dt;
}

