#ifndef __PATH_SYNC_ASTAR_JOINT_STATE_UTILS_HPP__
#define __PATH_SYNC_ASTAR_JOINT_STATE_UTILS_HPP__

#include <vector>

#include "path_sync/path_sync_types.hpp"
#include "path_sync/visualization_system/grid.hpp"

namespace mapf
{
namespace astar_joint_state
{
struct Utils
{
    // HEURISTIC FUNCTION
    static float heuristic(std::vector<Coordinate> starts, std::vector<Coordinate> goals);
    static float manhattan_distance(Coordinate pos1, Coordinate pos2);

    // CARTESION PRODUCT FUNCTION
    static std::optional<std::vector<std::vector<Coordinate>>> cartesian_product(
        std::vector<std::vector<Coordinate>> input_vec);

    static void cartesian_product_underlying(std::vector<std::vector<Coordinate>> &res,
                                              const std::vector<Coordinate> &in1);

    // POSSIBLE ACTIONS FUNCTION
    static std::optional<std::vector<std::vector<Coordinate>>> possible_actions_with_state(
        const mapf_type::JointState &state, const path_sync::Grid &grid);

    static std::vector<Coordinate> possible_actions_with_agent(const Coordinate &agent, const path_sync::Grid &grid);

    // APPLY ACTIONS FUNCTION
    static mapf_type::JointState apply_actions(const mapf_type::JointState &state,
                                                const std::vector<Coordinate> &actions);

    static Coordinate apply_single_action(Coordinate agent, Coordinate action);

    // CHECK VALIDITY OF STATE: (check for edge collision and vertex collision)
    static bool check_validity_of_state(const mapf_type::JointState &current_state,
                                         const mapf_type::JointState &new_state);

    static std::vector<std::vector<Coordinate>> extract_paths(mapf_type::NodePtr&);
};
} // namespace astar_joint_state
} // namespace mapf

#endif // !__PATH_SYNC_ASTAR_JOINT_STATE_UTILS_HPP__
