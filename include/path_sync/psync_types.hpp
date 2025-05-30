#ifndef __PATH_SYNC_PSYNC_TYPES__
#define __PATH_SYNC_PSYNC_TYPES__

#include <SFML/System/Vector2.hpp>
#include <cstddef>
#include <memory>
#include <vector>

#include "path_sync/visualization_system/cell.hpp"

typedef std::vector<std::vector<psync::Cell>> CellGrid;
typedef std::vector<std::vector<std::vector<size_t>>> CostGrid;
typedef std::pair<int, int> Coordinate;

namespace mapf_type
{

struct JointState
{
    std::vector<Coordinate> positions;
    std::size_t time;

    bool operator<(const JointState& right) const
    {
        return time < right.time;
    }
};

struct Node
{
    Node(JointState state, std::size_t g_score, std::size_t h_score, std::shared_ptr<Node> parent = nullptr)
        : _state(state), _g_score(g_score), _h_score(h_score), _parent(parent)
    {
    }

    bool operator>(const Node &other) const
    {
        return _g_score > other._g_score;
    }

    JointState _state;
    std::size_t _g_score;
    std::size_t _h_score;
    std::shared_ptr<Node> _parent;
};

using NodePtr = std::shared_ptr<Node>;

struct CompareGreaterNode
{
    bool operator()(const NodePtr n1, const NodePtr n2) const 
    {
        return *n1 > *n2;
    }
};

} // namespace mapf_type
#endif // !__PATH_SYNC_PSYNC_TYPES__
