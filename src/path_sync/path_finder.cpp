#include "path_sync/path_finder.hpp"
#include "path_sync/logger.hpp"

#include <SFML/Window/Keyboard.hpp>
#include <algorithm>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace psync
{

std::optional<std::vector<Coordinate>> PathFinder::find_path(ISolver &solver, psync::Grid &grid)
{
    std::stringstream ss;

    std::vector<Coordinate> start_points = grid.get_start_points();
    std::vector<Coordinate> end_points = grid.get_end_points();

    if (start_points.size() == end_points.size())
    {
        std::map<Coordinate, Coordinate> node_map;
        std::vector<Coordinate> path;
        __performance_met.solver_name = solver.get_solver_name();

        for (int counter = 0; counter < start_points.size(); counter++)
        {
            node_map = solver.solve(grid, start_points[counter], end_points[counter], __performance_met);

            if (node_map.empty())
            {
                ss << "No possible route from: (" << start_points[counter].second << ", " << start_points[counter].first
                   << ") to : (" << end_points[counter].second << ", " << end_points[counter].first << ")."
                   << std::endl;

                psync::Logger::get()->info(ss.str().c_str());
                ss.str(std::string());

                return std::nullopt;
            }

            path = __construct_path(node_map, start_points[counter], end_points[counter]);
            __performance_met.path_length = path.size();
        }
        return path;
    }

    /*else if (start_points.size() == 1 and end_points.size() > 1)*/
    /*{*/
    /*    std::map<Coordinate, Coordinate> node_map;*/
    /*    std::vector<Coordinate>::iterator current_start = start_points.begin();*/
    /**/
    /*    for (std::vector<Coordinate>::iterator it = end_points.begin(); it < end_points.end(); it++)*/
    /*    {*/
    /*        std::vector<Coordinate> path;*/
    /*        if (node_map.find(*current_start) == node_map.end() or node_map.find(*it) == node_map.end())*/
    /*        {*/
    /*            node_map = solver.solve(grid, *current_start, *it);*/
    /*        }*/
    /**/
    /*        path = __construct_path(node_map, *current_start, *it);*/
    /**/
    /*        current_start = it;*/
    /*    }*/
    /*}*/

    else
    {
        throw std::logic_error("Start and End points are not equal.");
    }

    return std::nullopt;
}

std::vector<Coordinate> PathFinder::__construct_path(std::map<Coordinate, Coordinate> &node_map,
                                                     const Coordinate &start, const Coordinate &end)
{
    std::vector<Coordinate> the_path;
    the_path.push_back(end);

    Coordinate current = node_map[end];
    while (current != start)
    {
        the_path.push_back(current);
        current = node_map[current];
    }
    the_path.push_back(start);

    std::reverse(the_path.begin(), the_path.end());
    return std::vector<Coordinate>(the_path.begin(), the_path.end());
}

} // namespace psync
