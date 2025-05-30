/**
 * @file visualization_system.hpp
 * @brief Header file for Visualization System of Path Sync Project. 
 *
 */

#ifndef __MOMAPF_VISUALIZATION_SYSTEM_HPP__
#define __MOMAPF_VISUALIZATION_SYSTEM_HPP__


#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "path_sync/solvers/astar_solver.hpp"
#include "path_sync/solvers/bfs_solver.hpp"
#include "path_sync/visualization_system/grid.hpp"
#include "path_sync/path_finder.hpp"
#include "path_sync/visualization_system/visualization_system_config.hpp"

namespace path_sync {

/**
 * @class VisualizationSystem
 * @brief Singleton class that controls the visualization and view of the app.
 *
 */
class VisualizationSystem 
{
public:

    /**
     * @brief Initialize the VisualizationSystem with given VisualizationSystemConfig.
     *
     * @param system_config VisualizationSystemConfig object that holds the system env_vars.
     *
     */
    static void initialize(VisualizationSystemConfig& system_config);


    /**
     * @brief Get the singleton instance of VisualizationSystem.
     *
     * @return VisualizationSystem*
     */
    static VisualizationSystem* get();


    /**
     * @brief Handles the events.
     */
    void handle_event();


    /**
     * @brief Handles updates in the windows. 
     * Mainly clears the SFML window and redraws everything. 
     * Also handles updates of ImGui.
     */
    void update();


    /**
     * @brief Handles the main_loop of current SFML window.
     *
     * It calls handle_event() and then update(). Upon closing, It shuts down Imgui.
     */
    void run();

private:
    VisualizationSystemConfig __system_config;
    static VisualizationSystem* __instance;
    sf::RenderWindow __main_window;

    sf::Clock __deltaClock;
    path_sync::Grid __grid;

    // FIX: path_finder, solvers shouldn't be here
    path_sync::PathFinder __path_finder;
    std::vector<ISolver*> __solvers;
    path_sync::solvers::sapf::Astar_Solver __astar_solver;
    path_sync::solvers::sapf::BFS_Solver __bfs_solver;
    std::size_t __selected_solver_index;

    std::vector<std::string> __available_maps;
    std::size_t __selected_map_index;
    std::stringstream __help_stream;

    VisualizationSystem(VisualizationSystemConfig& ) ;

    VisualizationSystem(VisualizationSystem const&) = delete;
    VisualizationSystem& operator=(VisualizationSystem const&) = delete;

    bool __is_mouse_inside_window();
    Coordinate __draw_with_cell_type(path_sync::CellType cell_type);

    void __get_available_maps();
    void __change_map();

    ~VisualizationSystem();

};  // end of class VisualizationSystem
} // end of namespace momapf

#endif
