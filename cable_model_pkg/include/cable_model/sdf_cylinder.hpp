#ifndef SDF_CYLINDER_HPP
#define SDF_CYLINDER_HPP

#include "sdf_builder.hpp"

namespace sdf_cylinder{

    class CylinderSdf : public sdf_builder::SdfBuilder
    {
        private:

        public:
            CylinderSdf(/* args */);
            ~CylinderSdf();

            void addLink(std::string link_name, float mass, float radius, float length, std::vector<float> pose = std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            void setSelfCollide(bool self_collide);
            
            void setMu1(float mu1);
            void setMu2(float mu2);
            void setFdir1(std::vector<float> fdir1);
            void setSlip1(float slip1);
            void setSlip2(float slip2);
            void setTortionalFriction(float t_f);
            void setGravity(bool gravity);

            void addPLugin(std::string plugin_name, std::string plugin_filename);

            std::string getSDF();
    };

}

#endif
