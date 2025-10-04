/**
 * @file visualization_system.hpp
 * @brief Header file for Visualization System of Path Sync Project.
 *
 */

#ifndef __MOMAPF_VISUALIZATION_SYSTEM_HPP__
#define __MOMAPF_VISUALIZATION_SYSTEM_HPP__

#include <functional>
#include <map>
#include <memory>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Window/Event.hpp>

#include "PathSyncApp.hpp"
#include "path_sync_ui/grid.hpp"
#include "path_sync_ui/visualization_system_config.hpp"

namespace path_sync
{

/**
 * @class VisualizationSystem
 * @brief Singleton class that controls the visualization and view of the app.
 *
 */
class VisualizationSystem
{
  public:
    explicit VisualizationSystem(PathSyncApp& app, std::unique_ptr<VisualizationSystemConfig> system_config);
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
     * It calls handle_event() and then update(). Upon closing, It shuts down
     * Imgui.
     */
    void run();

  private:
    PathSyncApp& app_;
    std::unique_ptr<VisualizationSystemConfig> system_config_;
    sf::RenderWindow main_window_;

    Grid grid_;

    std::stringstream help_stream_;

    std::map<sf::Keyboard::Key, std::function<void()>> key_bindings_;

    VisualizationSystem(VisualizationSystem const &) = delete;
    VisualizationSystem &operator=(VisualizationSystem const &) = delete;

    void setup_keybindings();
    void handle_key_press(const sf::Event::KeyPressed &key_event);
    void handle_mouse_button_press(const sf::Event::MouseButtonPressed &mouse_event,
                                     const sf::Vector2i &mouse_position);
    void handle_mouse_move(const sf::Event::MouseMoved &mouse_event, const sf::Vector2i &mouse_position);

    bool is_point_inside_window_bounds(const sf::Vector2i &point);
    Coordinate get_grid_cell_from_mouse_position(const sf::Vector2i &mouse_position);

}; // end of class VisualizationSystem
} // namespace path_sync

#endif
