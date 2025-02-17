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

#include "path_sync/visualization_system_config.hpp"

namespace psync {

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
    static VisualizationSystem* __instance;
    sf::RenderWindow __main_window;
    sf::View __main_view;
    float __zoom_factor;
    int __zoom_direction; 
    sf::Clock __deltaClock;
    sf::RectangleShape rect;

    VisualizationSystem(VisualizationSystemConfig& ) ;

    VisualizationSystem(VisualizationSystem const&) = delete;
    VisualizationSystem& operator=(VisualizationSystem const&) = delete;

    /**
     * @brief Performs zoom action upon mouse scroll.
     *
     * @param scroll_event sf::Event::MouseWheelScrolled* sfml mouse scroll event.
     */
    void __set_zoom(const sf::Event::MouseWheelScrolled* scroll_event);

    ~VisualizationSystem();

};  // end of class VisualizationSystem
} // end of namespace momapf

#endif
