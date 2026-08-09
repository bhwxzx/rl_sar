#include <mujoco/mujoco.h>

#include <array>
#include <iostream>
#include <string>

namespace
{
bool LoadAndValidate(const std::string& scene_name, bool expect_heightfield)
{
    const std::string path = std::string(LW_DESCRIPTION_MJCF_DIR)
        + "/" + scene_name;
    std::array<char, 2048> error{};
    mjModel* model = mj_loadXML(path.c_str(), nullptr, error.data(), error.size());
    if (model == nullptr)
    {
        std::cerr << "failed to load " << path << ": " << error.data()
                  << std::endl;
        return false;
    }

    bool valid = true;
    if (model->nu != 10)
    {
        std::cerr << scene_name << " has " << model->nu
                  << " actuators; expected 10" << std::endl;
        valid = false;
    }
    if (model->nsensor < 37)
    {
        std::cerr << scene_name << " has only " << model->nsensor
                  << " sensors; expected at least 37" << std::endl;
        valid = false;
    }
    if ((model->nhfield > 0) != expect_heightfield)
    {
        std::cerr << scene_name << " heightfield presence is incorrect"
                  << std::endl;
        valid = false;
    }
    for (const char* sensor_name : {
             "left_foot_force_sensor", "right_foot_force_sensor"})
    {
        if (mj_name2id(model, mjOBJ_SENSOR, sensor_name) < 0)
        {
            std::cerr << scene_name << " is missing sensor " << sensor_name
                      << std::endl;
            valid = false;
        }
    }

    mjData* data = mj_makeData(model);
    if (data == nullptr)
    {
        std::cerr << "failed to allocate MuJoCo data for " << scene_name
                  << std::endl;
        valid = false;
    }
    else
    {
        mj_forward(model, data);
        mj_deleteData(data);
    }
    mj_deleteModel(model);
    return valid;
}
} // namespace

int main()
{
    const bool flat_valid = LoadAndValidate("scene.xml", false);
    const bool terrain_valid = LoadAndValidate("scene_terrain.xml", true);
    if (!flat_valid || !terrain_valid)
    {
        return 1;
    }
    std::cout << "LW MuJoCo descriptions loaded successfully" << std::endl;
    return 0;
}
