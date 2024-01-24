#include <sdf_builder.hpp>

using namespace sdf_builder;

using namespace std;

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

SdfBuilder::SdfBuilder(/* args */)
{
}
    
SdfBuilder::~SdfBuilder()
{
}


void SdfBuilder::setModelName(string model_name)
{
    open_model      = "<model name ='"+model_name+"'>\n";
}

void SdfBuilder::setLinkName(string link_name)
{
    this->link_name_list.push_back(link_name);
    open_link.push_back("<link name ='" + link_name +"'>\n");
    open_collision  = "<collision name='collision'>\n";
    open_visual     = "<visual name='visual'>\n";

    
    this->link_pose.push_back(to_string(0) +" "+ to_string(0) +" "+ to_string(0) +" "+ to_string(0) +" "+ to_string(0) +" "+ to_string(0));
    this->model_pose = to_string(0) +" "+ to_string(0) +" "+ to_string(0) +" "+ to_string(0) +" "+ to_string(0) +" "+ to_string(0);
    
    this->mass.push_back("1");
    this->object_dimension.push_back("1");
    this->self_collision.push_back("false");
    
    this->mu1.push_back("1"); 
    this->mu2.push_back("1");
    this->fdir1.push_back("0 0 0");
    this->slip1.push_back("0");
    this->slip2.push_back("0");
    this->tortional_friction.push_back("-1");

}

void SdfBuilder::setModelType(ModelGeometry model)
{
    this->model_type = model;
    switch (model)
    {
        case sphere:
            open_geometry_model  = "<sphere>"; 
            close_geometry_model = "</sphere>";
            open_dimension  = "<radius>"; 
            close_dimension = "</radius>";
        break;

        case box:
            open_geometry_model  = "<box>"; 
            close_geometry_model = "</box>";
            open_dimension  = "<size>"; 
            close_dimension = "<size/>\n";
        break;
    
        case cylinder:
            open_geometry_model  = "<cylinder>"; 
            close_geometry_model = "</cylinder>";
        break;
    }

}

void SdfBuilder::setJoint(std::string joint_name, std::string parent, std::string child, TypeOfJoint type, vector<int> axis = vector<int>{1,0,0})
{
    switch (type)
    {
        case fixed:
            joint.push_back("<joint name='" + joint_name +"' type='fixed'>\n" +
                            "<parent>" + parent + "</parent>\n" +
                            "<child>" + child + "</child>\n" +
                            "</joint>\n"
                            );
        break;

        case revolute:
            joint.push_back("<joint name='" + joint_name +"' type='revolute'>\n" +
                "<parent>" + parent + "</parent>" +
                "<child>" + child + "</child>" +
                "<axis>" + to_string(axis[0]) +to_string(axis[1]) + to_string(axis[2])  +"</axis>"+
                "</joint>"
                );
        break;
    
        case prismatic:
            joint.push_back("<joint name='" + joint_name +"' type='prismatic'>\n" +
                "<parent>" + parent + "</parent>" +
                "<child>" + child + "</child>" +
                "<axis>" + to_string(axis[0]) +to_string(axis[1]) + to_string(axis[2])  +"</axis>"+
                "</joint>"
                );
        break;

        case revolute2:
           ;
        break;
    }

}

void SdfBuilder::setGravity(bool gravity){
    this->gravity.push_back("<gravity>" + to_string(gravity) + "</gravity>\n");
}

void SdfBuilder::setModelPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{
    this->model_pose = to_string(x) +" "+ to_string(y) +" "+ to_string(z) +" "+ to_string(roll) +" "+ to_string(pitch) +" "+ to_string(yaw);
}

void SdfBuilder::setLinkPose(float x=0.0, float y=0.0, float z=0.0, float roll=0.0, float pitch=0.0, float yaw=0.0)
{
    this->link_pose.back() = to_string(x) +" "+ to_string(y) +" "+ to_string(z) +" "+ to_string(roll) +" "+ to_string(pitch) +" "+ to_string(yaw);
}

void SdfBuilder::setInertiaMomentParam(float i_xx,float i_xy,float i_xz,float i_yy,float i_yz,float i_zz)
{
    this->i_xx.push_back(to_string(i_xx));
    this->i_xy.push_back(to_string(i_xy));
    this->i_xz.push_back(to_string(i_xz));
    this->i_yy.push_back(to_string(i_yy));
    this->i_yz.push_back(to_string(i_yz));
    this->i_zz.push_back(to_string(i_zz));
}
void SdfBuilder::setMass(float mass) {this->mass.back() = to_string(mass);}

void SdfBuilder::setLinkDimension(std::string dimension){this->object_dimension.back() = dimension;}

void SdfBuilder::setMu1(float mu1){     this->mu1.back() = to_string(mu1);}
void SdfBuilder::setMu2(float mu2){     this->mu2.back() = to_string(mu2);}
void SdfBuilder::setFdir1(vector<float> fdir1){ this->fdir1.back() = to_string(fdir1[0]) + " " + to_string(fdir1[2]) + " " + to_string(fdir1[2]);}
void SdfBuilder::setSlip1(float slip1){ this->slip1.back() = to_string(slip1);}
void SdfBuilder::setSlip2(float slip2){ this->slip2.back() = to_string(slip2);}
void SdfBuilder::setTortionalFriction(float t_f){this->tortional_friction.back() = to_string(t_f);}

void SdfBuilder::setSelfCollide(bool self_collide){ this->self_collision.back() = BoolToString(self_collide);}
 

void SdfBuilder::setPlugin(std::string plugin_name, std::string plugin_filename){
    this->plugin.push_back("<plugin name='" + plugin_name + "' filename='" + plugin_filename + "'></plugin>");
}






string SdfBuilder::getXmlVersion() { return "<?xml version='1.0'?>\n";}

string SdfBuilder::getOpenSdf() { return "<sdf version ='1.7'>\n";}
string SdfBuilder::getCloseSdf(){return "</sdf>";}

string SdfBuilder::getOpenModel() {return open_model;}
string SdfBuilder::getCloseModel() {return "</model>";}

string SdfBuilder::getOpenLink(int index_link=0){return open_link[index_link];}
string SdfBuilder::getCloseLink(){return "</link>\n";}

string SdfBuilder::getMass(int index_link=0){return "<mass>"+ mass[index_link] +"</mass>\n";}

string SdfBuilder::getInertial(int index_link=0)
{
    return "<inertial>" +
            getMass(index_link)+
            "<inertia>\n" +
            "<ixx>"+this->i_xx[index_link]+"</ixx>" +
            "<ixy>"+this->i_xy[index_link]+"</ixy>" +
            "<ixz>"+this->i_xz[index_link]+"</ixz>" +
            "<iyy>"+this->i_yy[index_link]+"</iyy>" +
            "<iyz>"+this->i_yz[index_link]+"</iyz>" +
            "<izz>"+this->i_zz[index_link]+"</izz>\n" +
            "</inertia>\n" +
            "</inertial>\n";
}

string SdfBuilder::getGeometry(int index_geometry=0)
{
    return  open_geometry + 
            open_geometry_model +
            open_dimension + object_dimension[index_geometry] + close_dimension + 
            close_geometry_model +
            close_geometry;
}

string SdfBuilder::getFriction(int index_collision)
{
    
    string tortional;
    if (stof(tortional_friction[index_collision])>0){
        tortional = string("<torsional>\n")+
                            "<coefficient>"+ this->tortional_friction[index_collision] + "</coefficient>" +
                            "<surface_radius>" + this->object_dimension[index_collision] + "</surface_radius>" +
                            "<use_patch_radius>false</use_patch_radius>\n"+
                            "</torsional>\n";
    }
    else
        tortional = "";

    return string("<surface>") +
            "<friction>" +
            tortional +
            "<ode>\n" +
            "<mu>"      + this->mu1[index_collision]    + "</mu>" +
            "<mu2>"     + this->mu2[index_collision]    + "</mu2>" +
            "<fdir1>"   + this->fdir1[index_collision]  + "</fdir1>" +
            "<slip1>"   +this->slip1[index_collision]   + "</slip1>" +
            "<slip2>"   +this->slip2[index_collision]   + "</slip2>" +
            "</ode>\n"+
            "</friction>" +
            "</surface>\n";

}

string SdfBuilder::getCollision(int index_link=0)
{
    return  open_collision + 
            getFriction(index_link) +
            getGeometry(index_link) +
            close_collision;

}
string SdfBuilder::getVisual(int index_link=0)
{
    return  open_visual +
            getGeometry(index_link) +
            close_visual;        
}

string SdfBuilder::getModelPose()   {return open_pose + model_pose + close_pose;}
string SdfBuilder::getLinkPose(int index_link=0)    {return open_pose + link_pose[index_link] + close_pose;};

const bool SdfBuilder::modelHasJoint() {return !(joint.empty());}

const int SdfBuilder::getNumOfJoint() {return (int)joint.size();}

string SdfBuilder::getJoints()
{
    string joint_list;
    for (int i=0; i<joint.size(); i++)
        joint_list += joint[i] + "\n";
    
    return joint_list;
}

string SdfBuilder::getLinkName(int index_link)
{
    return link_name_list[index_link];
}


string SdfBuilder::getSelfCollision(int index_link)
{
    return "<self_collide>" + this->self_collision[index_link] + "</self_collide>\n";
}

string SdfBuilder::getGravity(int i){return this->gravity[i];}

std::string SdfBuilder::getPlugin()
{
    string plugin_string;
    for (int i=0; i<plugin.size(); i++)
        plugin_string += plugin[i] + "\n";
    return plugin_string;
}