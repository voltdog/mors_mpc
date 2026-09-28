#include "StateEstimatorHMB/HeightMapBuilder.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

std::string HeightMapConfigPath()
{
    const char* config_dir = std::getenv("CONFIGPATH");
    if (config_dir == nullptr || config_dir[0] == '\0')
    {
        throw std::runtime_error("[StateEstimatorHMB] CONFIGPATH must be set.");
    }

    return (std::filesystem::path(config_dir) / "heightmap_builder.yaml").string();
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        bool sim_mode = false;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--sim")
            {
                sim_mode = true;
            }
            else if (arg == "-h" || arg == "--help")
            {
                std::cout << "Usage: " << argv[0] << " --sim" << std::endl;
                return 0;
            }
            else
            {
                std::cerr << "Unknown option: " << arg << std::endl;
                return 2;
            }
        }

        if (!sim_mode)
        {
            std::cerr << "[StateEstimatorHMB] This build has no RealSense support; "
                         "only --sim is available."
                      << std::endl;
            return 2;
        }

        std::cout << "[StateEstimatorHMB] simulation mode: LCM heightmap builder only"
                  << std::endl;
        hmb::HeightMapBuilderNode node(HeightMapConfigPath(), true);
        return node.Run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[StateEstimatorHMB] fatal: " << e.what() << std::endl;
        return 1;
    }
}
