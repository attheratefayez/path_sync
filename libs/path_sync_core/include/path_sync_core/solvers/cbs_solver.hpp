#ifndef PATH_SYNC_CBS_SOLVER_HPP
#define PATH_SYNC_CBS_SOLVER_HPP

#include "path_sync_core/solver_interface.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace path_sync::solvers::mapf
{

// ── Shared types ────────────────────────────────────────────────────────

struct CBSConstraint
{
    int agent = -1;
    int x = 0, y = 0;
    int timestep = 0;
    bool is_vertex = true;
    int x2 = 0, y2 = 0;

    bool operator==(const CBSConstraint &o) const
    {
        return agent == o.agent && x == o.x && y == o.y && timestep == o.timestep;
    }
    bool operator<(const CBSConstraint &o) const
    {
        if (agent != o.agent) return agent < o.agent;
        if (timestep != o.timestep) return timestep < o.timestep;
        if (x != o.x) return x < o.x;
        return y < o.y;
    }
};

struct CBSConflict
{
    int agent_a = -1;
    int agent_b = -1;
    int x = 0, y = 0;
    int timestep = 0;
    bool is_vertex = true;
    int x2 = 0, y2 = 0;
};

enum class CBSConflictClass { CARDINAL, SEMI_CARDINAL, NON_CARDINAL };

struct CBSNode
{
    std::vector<std::vector<Coordinate>> paths;
    int cost = 0;
    std::vector<CBSConstraint> constraints;
    int h = 0;
    int depth = 0;
};

// ── CBS Solver (core CBS + ICBS + CBSH) ─────────────────────────────────

class CBS_Solver : public IMASolver
{
public:
    std::string_view get_solver_name() const override { return solver_name_; }
    bool is_optimal() const override { return true; }

    void set_use_icbs(bool v) { use_icbs_ = v; }
    void set_use_cbsh(bool v) { use_cbsh_ = v; }
    void set_solver_name(const std::string &name) { solver_name_ = name; }

    std::optional<std::vector<std::vector<Coordinate>>> solve(
        const MapData &map_data,
        std::vector<Coordinate> starts,
        std::vector<Coordinate> goals,
        PerformanceMetrics &performance_met) override;

private:
    friend class CBS_Solver_ICBS_Test;
    friend class CBS_Solver_CBSH_Test;

    // Core CBS
    std::optional<std::vector<Coordinate>> low_level_search(
        const MapData &map,
        Coordinate start, Coordinate goal,
        const std::vector<CBSConstraint> &constraints,
        int agent_id) const;

    std::optional<CBSConflict> find_first_conflict(
        const std::vector<std::vector<Coordinate>> &paths) const;

    // ICBS
    CBSConflictClass classify_conflict(
        const CBSConflict &conflict,
        const std::vector<std::vector<Coordinate>> &current_paths,
        const MapData &map,
        const std::vector<Coordinate> &starts,
        const std::vector<Coordinate> &goals) const;

    std::optional<std::vector<std::vector<Coordinate>>> try_bypass(
        const MapData &map,
        const std::vector<Coordinate> &starts,
        const std::vector<Coordinate> &goals,
        const CBSNode &node,
        const CBSConflict &conflict) const;

    // CBSH
    int compute_cg_heuristic(
        const std::vector<std::vector<Coordinate>> &paths) const;

    int min_vertex_cover_size(const std::vector<std::vector<bool>> &graph) const;

    // Helpers
    bool validate_paths_against_constraints(
        const std::vector<std::vector<Coordinate>> &paths,
        const std::vector<CBSConstraint> &constraints) const;

    int compute_soc(const std::vector<std::vector<Coordinate>> &paths) const;

    bool use_icbs_ = false;
    bool use_cbsh_ = false;
    std::string solver_name_ = "CBS_Solver";
    static constexpr int MAX_TIMESTEP = 500;
};

} // namespace path_sync::solvers::mapf
#endif
