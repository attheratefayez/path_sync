#ifndef PATH_SYNC_POTENTIAL_FIELD_SOLVER_HPP
#define PATH_SYNC_POTENTIAL_FIELD_SOLVER_HPP

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solver_interface.hpp"

#include <vector>

namespace path_sync
{
namespace solvers
{
namespace sapf
{

class PotentialFieldSolver : public ISolver
{
public:
    PotentialFieldSolver();
    std::string_view get_solver_name() const override;
    bool is_optimal() const override { return false; }
    void set_weights(const std::vector<float> &w) { weights_ = w; }
    void set_num_objectives(int n) { num_objectives_ = n; }

    std::map<Coordinate, Coordinate> solve(
        const MapData &map_data, Coordinate start, Coordinate goal,
        PerformanceMetrics &performance_met) override;

private:
    std::string solver_name_ = "PotentialField";
    std::vector<float> weights_ = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    int num_objectives_ = 5;
    static constexpr int MAX_ITER = 50000;
};

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#endif // PATH_SYNC_POTENTIAL_FIELD_SOLVER_HPP
