#include "ModelCablePlugin.hpp"

using namespace gazebo;
using namespace std;
using namespace cable_utils;

// void GripInfoCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//   setFirstMassGripped(msg->data);
// }
#define RESET "\033[0m"
#define ORANGE "\033[38;5;214m"

void CableModelPlugin::open_forces_csv(std::string folder_path){
    // Apri file CSV per scrivere le forze
    this->start_time = std::chrono::steady_clock::now();
    std::string file_path = folder_path + "forces_data.csv";
    forces_csv_file.open(file_path, std::ios::out);
    if (!forces_csv_file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file forces_data.csv!" << std::endl;
        exit(0);  // Puoi decidere di uscire dalla funzione o lanciare un'eccezione
    }
    else {
        std::cout << ORANGE << "File forces_data.csv aperto correttamente." << RESET << std::endl;
    }

    forces_csv_file << "time";  // Prima colonna: tempo
    for (int i = 0; i < this->num_particles; ++i) {
        forces_csv_file << ",force_magnitude_mass_" << i;  // Modifica per scrivere il modulo della forza
    }
    forces_csv_file << "\n";
}


void CableModelPlugin::open_positions_csv(std::string folder_path) {
    // Apri file CSV per scrivere le posizioni
    this->start_time = std::chrono::steady_clock::now();
    std::string file_path= folder_path  + "positions_data.csv";
    position_csv_file.open(file_path, std::ios::out);
    if (!position_csv_file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file positions_data.csv!" << std::endl;
        exit(0);  // Puoi decidere di uscire dalla funzione o lanciare un'eccezione
    } else {
        std::cout << ORANGE << "File positions_data.csv aperto correttamente." << RESET << std::endl;
    }

    // Scrivi le intestazioni per il tempo, le posizioni x e z
    position_csv_file << "time";  // Prima colonna: tempo
    for (int i = 0; i < this->num_particles; ++i) {
        position_csv_file << ",position_x_" << i << ",position_z_" << i;  // Posizioni x e z per ogni particella
    }
    position_csv_file << "\n";
}


void CableModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){


    // Store the pointer to the model
    this->model = _parent;

    double poisson_ratio, young_modulus, damping; 

    std::vector<double> pos;
    std::vector<double> rot;
    // ros::NodeHandle ros_nh;
    ros_nh.getParam("/cable/width", this->width);
    ros_nh.getParam("/cable/length", this->length);
    ros_nh.getParam("/cable/resolution", this->resolution);
    ros_nh.getParam("/cable/rotation", rot);
    ros_nh.getParam("/cable/position", pos);
    ros_nh.getParam("/cable/mass", this->mass);
    ros_nh.getParam("/cable/damping", damping);
    ros_nh.getParam("/cable/gravity", this->gravity);
    ros_nh.getParam("/cable/num_particles", this->num_particles);
    ros_nh.getParam("/cable/prefix_mass_names", this->prefix_mass_names);
    ros_nh.getParam("/cable/poisson_ratio", poisson_ratio);
    ros_nh.getParam("/cable/young_modulus", young_modulus);

    this->pos_x = pos[0];
    this->pos_y = pos[1];
    this->pos_z = pos[2];
    this->rot_x = rot[0];
    this->rot_y = rot[1];
    this->rot_z = rot[2];

    cout << poisson_ratio << endl;
    cout << young_modulus << endl;
    
    // cout <<this->model->RelativePose().Pos().X() << endl; //ignition::math::Pose3d
    cable = std::make_shared<cable_utils::Cable>(this->model, this->pos_x, this->pos_y, this->pos_z, this-> mass, this->width, this->length, this->num_particles, this->prefix_mass_names);

    cable->useGravity(this->gravity);
    cable->setDamperCoef(damping);
    cable->setYoungModulus(young_modulus);
    cable->setPoissonRatio(poisson_ratio);
    // cout << link->RelativePose().Pos().X() << endl;


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CableModelPlugin::OnUpdate, this));

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "cable_plugin_client",
            ros::init_options::NoSigintHandler);
    }
    std::string package_path = ros::package::getPath("cable_model_pkg");
    // this-> open_forces_csv(package_path + "/data/");
    // this-> open_positions_csv(package_path + "/data/");

//   publish_force_mass_0 = ros_nh.advertise<cable_model_pkg::coordinates>("/force_mass_0", 1);

    this->sub_mass0_pos = ros_nh.subscribe("/mass0/position", 1, &CableModelPlugin::mass0PositionCallback, this);
    this->sub_massN_pos = ros_nh.subscribe("/massN/position", 1, &CableModelPlugin::massNPositionCallback, this);

    this->mass_0_pos = this->cable->getPositionWrtWorld(0); // Inizializza la posizione della massa 0
}

void CableModelPlugin::mass0PositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    this->mass_0_pos = ignition::math::Vector3d(msg->x, msg->y, msg->z);
    // std::cout << "Mass 0 position updated: " << this->mass_0_pos << std::endl;
}

// Callback per massa n-1
void CableModelPlugin::massNPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    this->mass_N_pos = ignition::math::Vector3d(msg->x, msg->y, msg->z);
}

bool CableModelPlugin::callbackGraspServer(cable_model_pkg::GraspMsg::Request &rqst, cable_model_pkg::GraspMsg::Response &res){
                            this->isMass0Gripped = rqst.grasp_mass_0;
                            this->isMassNGripped = rqst.grasp_mass_N;
                            return true;
                        }
                        
void CableModelPlugin::OnUpdate(){
    // auto t = std::chrono::steady_clock::now();
    
    // force_mass_0 = cable->getForceWrtWorld(0);

    // force_mass_0_msg.x = force_mass_0.X();
    // force_mass_0_msg.y = force_mass_0.Y();
    // force_mass_0_msg.z = force_mass_0.Z();

    // publish_force_mass_0.publish(force_mass_0_msg);

    // // std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t - t0).count() << " ms" << std::endl;
    // t0 = t;


    // this->write_forces();
    // this->write_positions();
    // update_0_mass_position of 1 mm usando una sinusoidale

    // ignition::math::Vector3d new_pos = cable->getPositionWrtWorld(0);
    // new_pos.X() += 0.004 * sin(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() / 500.0); // Aggiungi un piccolo offset alla posizione X della massa 0
    // printa la mass pos 
    
    if (this->isMass0Gripped) 
        cable->setMassPosition(0, mass_0_pos);

        

    cable->updateModel();

}

void CableModelPlugin::write_forces(){
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    // Scrivi il tempo corrente sul file
    forces_csv_file << elapsed;

    // Per ogni massa, ottieni la forza e calcola il modulo
    for (int i = 0; i < this->num_particles; ++i) {
        auto force = cable->getForceWrtWorld(i);
        
        // Calcolo del modulo della forza
        double force_magnitude = sqrt(force.X() * force.X() + force.Y() * force.Y() + force.Z() * force.Z());

        // Scrivi il modulo della forza sul CSV
        forces_csv_file << "," << force_magnitude;
    }
    
    // Vai alla riga successiva
    forces_csv_file << "\n";
}

void CableModelPlugin::write_positions(){
    // solo su Z
    // Ottieni il tempo corrente
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    // Scrivi il tempo corrente sul file
    position_csv_file << elapsed;

    // Per ogni massa, ottieni la posizione lungo Z 
    for (int i = 0; i < this->num_particles; ++i) {
        auto position = cable->getPositionWrtWorld(i);
        
        // Calcolo del modulo della forza
        double position_x = position.X();
        double position_z = position.Z();

        // Scrivi il modulo della forza sul CSV
        position_csv_file << "," << position_x << "," << position_z;
    }

    // Vai alla riga successiva
    position_csv_file << "\n";
}
