#ifndef __PATH_SYNC_ASTAR_JOINT_STATE_UTILS_HPP__
#define __PATH_SYNC_ASTAR_JOINT_STATE_UTILS_HPP__

#include <optional>
#include <vector>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace mapf
{
namespace astar_joint_state
{
struct Utils
{
    // HEURISTIC FUNCTION
    static float heuristic(std::vector<path_sync::Coordinate> starts, std::vector<path_sync::Coordinate> goals);
    static float manhattan_distance(path_sync::Coordinate pos1, path_sync::Coordinate pos2);

    // CARTESION PRODUCT FUNCTION
    static std::optional<std::vector<std::vector<path_sync::Coordinate>>> cartesian_product(
        std::vector<std::vector<path_sync::Coordinate>> input_vec);

    static void cartesian_product_underlying(std::vector<std::vector<path_sync::Coordinate>> &res,
                                             const std::vector<path_sync::Coordinate> &in1);

    // POSSIBLE ACTIONS FUNCTION
    static std::optional<std::vector<std::vector<path_sync::Coordinate>>> possible_actions_with_state(
        const path_sync::mapf_type::JointState &state, const path_sync::MapData &map_data);

    static std::vector<path_sync::Coordinate> possible_actions_with_agent(const path_sync::Coordinate &agent, const path_sync::MapData &map_data);

    // APPLY ACTIONS FUNCTION
    static path_sync::mapf_type::JointState apply_actions(const path_sync::mapf_type::JointState &state,
                                               const std::vector<path_sync::Coordinate> &actions);


    static path_sync::Coordinate apply_single_action(path_sync::Coordinate agent, path_sync::Coordinate action);

    // CHECK VALIDITY OF STATE: (check for edge collision and vertex collision)
    static bool check_validity_of_state(const path_sync::mapf_type::JointState &current_state,
                                        const path_sync::mapf_type::JointState &new_state);

    static std::vector<std::vector<path_sync::Coordinate>> extract_paths(path_sync::mapf_type::NodePtr &current);

};
} // namespace astar_joint_state
} // namespace mapf

#endif // !__PATH_SYNC_ASTAR_JOINT_STATE_UTILS_HPP__
