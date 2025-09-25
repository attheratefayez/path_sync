#ifndef __PATH_SYNC_PATH_SYNC_TYPES__
#define __PATH_SYNC_PATH_SYNC_TYPES__

#include <memory>
#include <sstream>
#include <vector>

namespace path_sync
{

enum class CellType
{
    DEFAULT,
    WALL,
    FOUND,
    VISITED,
    START,
    END,
    PATH,
};

typedef std::vector<std::vector<path_sync::CellType>> CellGrid;
typedef std::vector<std::vector<std::size_t>> CostGrid;
typedef std::pair<int, int> Coordinate;


struct MapInfo
{
    std::string map_name;
    std::stringstream map;
    std::size_t width;
    std::size_t height;
};

namespace mapf_type
{

struct JointState
{
    std::vector<Coordinate> positions;
    std::size_t time;

    bool operator<(const JointState &right) const
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
} // namespace path_sync

#endif // !__PATH_SYNC_PATH_SYNC_TYPES__
