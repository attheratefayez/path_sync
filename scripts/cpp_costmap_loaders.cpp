struct CostMap
{
    int height;
    int width;
    int objectives;

    std::vector<float> costs;
};

CostMap loadCostMap(const std::string& filename)
{
    CostMap m;

    std::ifstream in(
        filename,
        std::ios::binary
    );

    in.read(
        reinterpret_cast<char*>(&m.height),
        sizeof(int)
    );

    in.read(
        reinterpret_cast<char*>(&m.width),
        sizeof(int)
    );

    in.read(
        reinterpret_cast<char*>(&m.objectives),
        sizeof(int)
    );

    size_t total =
        static_cast<size_t>(m.height) *
        m.width *
        m.objectives;

    m.costs.resize(total);

    in.read(
        reinterpret_cast<char*>(m.costs.data()),
        total * sizeof(float)
    );

    return m;
}
