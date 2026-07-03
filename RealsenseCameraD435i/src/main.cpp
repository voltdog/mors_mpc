#include "RealsenseCameraD435i/RealsenseCameraD435i.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string ResolveConfigPath(int argc, char** argv)
{
    if (argc > 1)
    {
        return argv[1];
    }

    std::vector<std::string> candidates;

    const char* config_dir = std::getenv("CONFIGPATH");
    if (config_dir != nullptr && config_dir[0] != '\0')
    {
        candidates.emplace_back(std::string(config_dir) + "/realsense_camera_d435i.yaml");
    }
    candidates.emplace_back("config/realsense_camera_d435i.yaml");
    candidates.emplace_back("../config/realsense_camera_d435i.yaml");
    candidates.emplace_back("../../config/realsense_camera_d435i.yaml");

    for (const auto& path : candidates)
    {
        if (std::filesystem::exists(path))
        {
            return path;
        }
    }

    if (!candidates.empty())
    {
        return candidates.front();
    }
    return "realsense_camera_d435i.yaml";
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        std::cout << std::unitbuf;
        std::cerr << std::unitbuf;
        const std::string config_path = ResolveConfigPath(argc, argv);
        std::cout << "[RealsenseCameraD435i] config: " << config_path << std::endl;
        rscam::RealsenseCameraD435iNode node(config_path);
        return node.Run();
    }
    catch (const rs2::error& e)
    {
        std::cerr << "[RealsenseCameraD435i] RealSense error in "
                  << e.get_failed_function() << "(" << e.get_failed_args()
                  << "): " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[RealsenseCameraD435i] fatal: " << e.what() << std::endl;
        return 1;
    }
}
