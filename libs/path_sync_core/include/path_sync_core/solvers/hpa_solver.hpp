#ifndef __PATHFINDING_HPA_SOLVER_HPP__
#define __PATHFINDING_HPA_SOLVER_HPP__

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solver_interface.hpp"

namespace path_sync
{
namespace solvers
{
namespace sapf
{

class HPA_Solver : public ISolver
{
  private:
    const std::string solver_name_ = "HPA_Solver";

    int cluster_size_ = 4;

    struct AbstractNode
    {
        Coordinate pos;
        int cluster_id;
    };

    struct AbstractEdge
    {
        int target_id;
        int cost;
    };

    int get_cluster_id(int x, int y, int w, int h) const;
    std::vector<std::vector<int>> build_abstract_graph(
        const MapData &map_data, std::vector<AbstractNode> &nodes) const;
    std::vector<Coordinate> refine_path(
        const MapData &map_data,
        const std::vector<int> &abstract_path,
        const std::vector<AbstractNode> &nodes,
        Coordinate start, Coordinate goal) const;

  public:
    std::string_view get_solver_name() const override;

    std::map<Coordinate, Coordinate> solve(const path_sync::MapData &map_data, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override;
};

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#endif
