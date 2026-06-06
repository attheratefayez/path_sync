#include "path_sync_core/solvers/cbs_solver.hpp"

namespace path_sync::solvers::mapf
{

// ── ICBS: classify a conflict as cardinal / semi / non ──────────────────

CBSConflictClass CBS_Solver::classify_conflict(
    const CBSConflict &conflict,
    const std::vector<std::vector<Coordinate>> &current_paths,
    const MapData &map,
    const std::vector<Coordinate> &starts,
    const std::vector<Coordinate> &goals) const
{
    // Baseline costs (current path lengths)
    int cost_a = static_cast<int>(current_paths[conflict.agent_a].size()) - 1;
    int cost_b = static_cast<int>(current_paths[conflict.agent_b].size()) - 1;

    bool a_increased = false;
    bool b_increased = false;

    // Test agent A with a vertex constraint at conflict
    {
        std::vector<CBSConstraint> con;
        CBSConstraint c;
        c.agent = conflict.agent_a;
        c.x = conflict.x; c.y = conflict.y;
        c.timestep = conflict.timestep;
        c.is_vertex = conflict.is_vertex;
        if (!conflict.is_vertex)
        {
            // For edge constraints, agent A must not be at (x2,y2) at timestep
            // Actually, for edge, agent A is at (x,y) → (x2,y2), so constrain (x2,y2)
            // But the constraint for agent A is just to avoid being at (x,y) at timestep-1 and (x2,y2) at timestep
            // For simplicity, constrain the endpoint
            c.x = conflict.is_vertex ? conflict.x : conflict.x2;
            c.y = conflict.is_vertex ? conflict.y : conflict.y2;
        }
        con.push_back(c);

        // Also constrain the other agents' move from the opposite side if edge
        if (!conflict.is_vertex)
        {
            CBSConstraint c2;
            c2.agent = conflict.agent_a;
            c2.x = conflict.x;
            c2.y = conflict.y;
            c2.timestep = conflict.timestep;
            c2.is_vertex = true;
            con.push_back(c2);
        }

        auto new_path = low_level_search(map, starts[conflict.agent_a],
                                          goals[conflict.agent_a], con,
                                          conflict.agent_a);
        if (new_path.has_value())
        {
            int new_cost = static_cast<int>(new_path->size()) - 1;
            if (new_cost > cost_a)
                a_increased = true;
        }
        else
        {
            a_increased = true; // no path found → effectively ∞ cost
        }
    }

    // Test agent B
    {
        std::vector<CBSConstraint> con;
        CBSConstraint c;
        c.agent = conflict.agent_b;
        c.x = conflict.x; c.y = conflict.y;
        c.timestep = conflict.timestep;
        c.is_vertex = conflict.is_vertex;
        if (!conflict.is_vertex)
        {
            c.x = conflict.x2;
            c.y = conflict.y2;
        }
        con.push_back(c);

        if (!conflict.is_vertex)
        {
            CBSConstraint c2;
            c2.agent = conflict.agent_b;
            c2.x = conflict.x;
            c2.y = conflict.y;
            c2.timestep = conflict.timestep;
            c2.is_vertex = true;
            con.push_back(c2);
        }

        auto new_path = low_level_search(map, starts[conflict.agent_b],
                                          goals[conflict.agent_b], con,
                                          conflict.agent_b);
        if (new_path.has_value())
        {
            int new_cost = static_cast<int>(new_path->size()) - 1;
            if (new_cost > cost_b)
                b_increased = true;
        }
        else
        {
            b_increased = true;
        }
    }

    if (a_increased && b_increased) return CBSConflictClass::CARDINAL;
    if (a_increased || b_increased) return CBSConflictClass::SEMI_CARDINAL;
    return CBSConflictClass::NON_CARDINAL;
}

// ── ICBS: try_bypass ────────────────────────────────────────────────────

std::optional<std::vector<std::vector<Coordinate>>> CBS_Solver::try_bypass(
    const MapData &map,
    const std::vector<Coordinate> &starts,
    const std::vector<Coordinate> &goals,
    const CBSNode &node,
    const CBSConflict &conflict) const
{
    // Try to reroute one of the conflicting agents to avoid conflict
    // while keeping the same cost

    int orig_a = static_cast<int>(node.paths[conflict.agent_a].size()) - 1;

    // Re-search agent A with a constraint at the conflict point
    std::vector<CBSConstraint> con = node.constraints;
    CBSConstraint c;
    c.agent = conflict.agent_a;
    c.x = conflict.x; c.y = conflict.y;
    c.timestep = conflict.timestep;
    c.is_vertex = conflict.is_vertex;
    if (!conflict.is_vertex)
    {
        // For edge, constrain the destination at timestep and origin at timestep-1
        CBSConstraint c2;
        c2.agent = conflict.agent_a;
        c2.x = conflict.x;
        c2.y = conflict.y;
        c2.timestep = conflict.timestep;
        c2.is_vertex = true;
        con.push_back(c2);
        c.x = conflict.x2;
        c.y = conflict.y2;
    }
    con.push_back(c);

    auto new_path_a = low_level_search(map, starts[conflict.agent_a],
                                        goals[conflict.agent_a], con,
                                        conflict.agent_a);
    if (!new_path_a.has_value())
        return std::nullopt;

    int new_cost_a = static_cast<int>(new_path_a->size()) - 1;
    if (new_cost_a > orig_a)
        return std::nullopt; // cost increased, not a bypass

    // Check if new paths are conflict-free
    auto new_paths = node.paths;
    new_paths[conflict.agent_a] = std::move(*new_path_a);
    auto remaining = find_first_conflict(new_paths);
    if (!remaining.has_value() || remaining->timestep > conflict.timestep)
        return new_paths; // bypass succeeded (or pushed conflict later in time)

    return std::nullopt;
}

} // namespace path_sync::solvers::mapf
