#include <cstring>
#include <fstream>
#include <iostream>

#include "path_sync_core/map_loader/cost_map.hpp"

namespace path_sync
{

bool CostMap::load(const std::string &filename)
{
    std::ifstream in(filename, std::ios::binary);

    if (!in.is_open())
    {
        std::cerr << "CostMap: could not open " << filename << std::endl;
        return false;
    }

    in.read(reinterpret_cast<char *>(&height), sizeof(int));
    in.read(reinterpret_cast<char *>(&width), sizeof(int));
    in.read(reinterpret_cast<char *>(&objectives), sizeof(int));

    if (!in || height <= 0 || width <= 0 || objectives <= 0)
    {
        std::cerr << "CostMap: invalid header in " << filename << std::endl;
        return false;
    }

    size_t total = static_cast<size_t>(height) * width * objectives;
    costs.resize(total);

    in.read(reinterpret_cast<char *>(costs.data()), total * sizeof(float));

    if (static_cast<size_t>(in.gcount()) != total * sizeof(float))
    {
        std::cerr << "CostMap: truncated file " << filename << std::endl;
        return false;
    }

    return true;
}

} // namespace path_sync
