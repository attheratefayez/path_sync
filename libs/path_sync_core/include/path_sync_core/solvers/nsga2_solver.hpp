#ifndef PATH_SYNC_NSGA2_SOLVER_HPP
#define PATH_SYNC_NSGA2_SOLVER_HPP

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/mo_types.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solvers/imo_solver.hpp"

namespace path_sync
{
namespace solvers
{
namespace mo
{

class NSGA2Solver : public IMOSolver
{
public:
    std::string_view get_solver_name() const override { return "NSGA2"; }
    bool is_optimal() const override { return false; }
    int get_num_objectives() const override { return num_objectives_; }
    void set_num_objectives(int n) { num_objectives_ = n; }
    void set_pop_size(int n) { pop_size_ = n; }
    void set_max_gen(int n) { max_gen_ = n; }

    std::optional<std::vector<MOSolution>> solve(
        const MapData &map_data,
        const CostMap *cost_map,
        Coordinate start, Coordinate goal,
        int num_objectives,
        PerformanceMetrics &perf,
        MOMetrics &mo_met) override;

private:
    int num_objectives_ = 5;
    int pop_size_ = 50;
    int max_gen_ = 50;

    struct Individual
    {
        std::vector<Coordinate> path;
        std::vector<float> costs;
        int rank = 0;
        float crowding = 0.0f;
    };

    void evaluate(Individual &ind, const MapData &map_data,
                  const CostMap *cost_map, Coordinate start, Coordinate goal,
                  int num_obj);
    void fast_non_dominated_sort(std::vector<Individual> &pop, int num_obj);
    void crowding_distance(std::vector<Individual> &pop, int num_obj);
    std::vector<Coordinate> random_path(const MapData &map_data,
                                         Coordinate start, Coordinate goal);
    std::vector<Coordinate> crossover(const std::vector<Coordinate> &p1,
                                       const std::vector<Coordinate> &p2);
    void mutate(std::vector<Coordinate> &path, const MapData &map_data);
};

} // namespace mo
} // namespace solvers
} // namespace path_sync

#endif // PATH_SYNC_NSGA2_SOLVER_HPP
