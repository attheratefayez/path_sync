#include <filesystem>
#include <iostream>
#include <optional>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <ostream>
#include <sstream>
#include <yaml-cpp/yaml.h>

#include "SFML/Window/Keyboard.hpp"
#include "path_sync/env_map.hpp"
#include "path_sync/grid.hpp"
#include "path_sync/logger.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/visualization_system.hpp"

namespace psync
{

std::stringstream ss;
// STATIC MEMBERS
VisualizationSystem *VisualizationSystem::__instance = nullptr;

void VisualizationSystem::initialize(VisualizationSystemConfig &system_config)
{
    __instance = new VisualizationSystem(system_config);
}

VisualizationSystem *VisualizationSystem::get()
{
    if (__instance == nullptr)
        psync::Logger::get()->error("VisualizationSystem not initialized.");
    return __instance;
}

// CONSTRUCTOR
VisualizationSystem::VisualizationSystem(VisualizationSystemConfig &system_config)
    : __system_config(system_config), __grid(system_config, GridMode::Map)
{
    __main_window.create(sf::VideoMode({system_config.WIDTH, system_config.HEIGHT}), system_config.TITLE,
                         sf::Style::Titlebar | sf::Style::Close);
    __main_window.setFramerateLimit(system_config.FRAMERATE);

    __main_view.setViewport(
        sf::FloatRect({0, 0}, {static_cast<float>(system_config.WIDTH), static_cast<float>(system_config.HEIGHT)}));

    __zoom_factor = 1.0;
    __zoom_direction = 1;

    /* HACK: setting view messes up with drawings on RenderWindow */
    /*__main_window.setView(__main_view);*/

    /*INFO: ADDING Solvers to __solvers list*/
    __solvers.push_back(&__astar_solver);
    __solvers.push_back(&__bfs_solver);

    /*__selected_solver_index = __solvers.size() ? __solvers.size() - 1 : 0;*/
    __selected_solver_index = 0;

    __help_stream << "\n";
    __help_stream << "\t\tPSYNC CONTROLS\n";
    __help_stream << "\ts    + Mouse-Left  :   Draw Start Point.\n";
    __help_stream << "\te    + Mouse-Left  :   Draw End Point.\n";
    __help_stream << "\tMouse-Left  + Drag :   Draw Wall.\n";
    __help_stream << "\tMouse-Right + Drag :   Erase Wall.\n";
    __help_stream << "\tc                  :   Change Solver.\n";
    __help_stream << "\tSpace              :   Find Solution.\n";
    __help_stream << "\tShift-H            :   Show This Help.\n";
    __help_stream << "\tShift-P            :   Clear Path.\n";
    __help_stream << "\tShift-M            :   Change Map.\n";
    __help_stream << "\tShift-R            :   Clear Grid.\n";

    __get_available_maps();
    __selected_map_index = 0;
}

// MEMBER FUNCTIONS
void VisualizationSystem::handle_event()
{
    while (std::optional<sf::Event> event = __main_window.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
            __main_window.close();

        if (event->is<sf::Event::MouseButtonPressed>())
        {
            /*DRAWING START POSITION*/
            if (event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left &&
                __is_mouse_inside_window() && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
            {
                Coordinate start_point = __draw_with_cell_type(psync::CellType::START);
                __grid.get_start_points().push_back(start_point);
            }

            /*DRAWING END POSITION*/
            else if (event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left &&
                     __is_mouse_inside_window() && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E))
            {
                Coordinate end_point = __draw_with_cell_type(psync::CellType::END);
                __grid.get_end_points().push_back(end_point);
            }

            /*DRAWING WALL*/
            else if (event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left &&
                     __is_mouse_inside_window())
            {
                Coordinate wall_pos = __draw_with_cell_type(psync::CellType::WALL);

                /*for(auto& cell: __grid.find_neighbors_b2(wall_pos))*/
                /*{*/
                /*    __grid.get_grid()[cell.second][cell.first].set_cell_type(psync::CellType::FOUND);*/
                /*}*/
            }

            /*Erasing */
            else if (event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Right)
            {
                __draw_with_cell_type(psync::CellType::DEFAULT);
            }
        }

        if (event->is<sf::Event::MouseMoved>())
        {

            /*DRAWING WALL*/
            if (__is_mouse_inside_window() && sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
            {
                __draw_with_cell_type(psync::CellType::WALL);
            }

            /*ERASING*/
            else if (__is_mouse_inside_window() && sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
            {
                sf::Vector2i current_mouse_pos = sf::Mouse::getPosition();
                __draw_with_cell_type(psync::CellType::DEFAULT);
            }
        }

        if (event->is<sf::Event::MouseWheelScrolled>())
        {
            __set_zoom(event->getIf<sf::Event::MouseWheelScrolled>());
        }

        if (event->is<sf::Event::KeyPressed>())
        {
            /*CHANGE SOLVERS*/
            if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::C)
            {
                psync::Logger::get()->info("Changing Solver...");
                __selected_solver_index++;
                __selected_solver_index %= __solvers.size();
                __selected_solver_index =
                    (__selected_solver_index == __solvers.size()) ? --__selected_solver_index : __selected_solver_index;

                ss << "Selected solver: " << __solvers[__selected_solver_index]->get_solver_name() << std::endl;
                psync::Logger::get()->info(ss.str().c_str());
                ss.str(std::string());
            }

            /*FIND SOLUTION*/
            if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Space)
            {
                psync::Logger::get()->info("FINDING SOLUTION...");
                ss << "Solver: " << __solvers[__selected_solver_index]->get_solver_name() << std::endl;
                psync::Logger::get()->info(ss.str().c_str());
                ss.str(std::string());


                std::optional<std::vector<Coordinate>> returned_path =
                    __path_finder.find_path(*__solvers[__selected_solver_index], __grid);

                if (returned_path != std::nullopt)
                {
                    for (auto &cell : *returned_path)
                    {
                        __grid.get_grid()[cell.second][cell.first].set_cell_type(psync::CellType::PATH);
                    }

                    for (auto &start_point : __grid.get_start_points())
                    {
                        __grid.get_grid()[start_point.second][start_point.first].set_cell_type(psync::CellType::START);
                    }

                    for (auto &end_point : __grid.get_end_points())
                    {
                        __grid.get_grid()[end_point.second][end_point.first].set_cell_type(psync::CellType::END);
                    }

                    std::cout << __path_finder.get_performance_data().str() << std::endl;
                }
            }

            /*SHOW HELP*/
            else if (event->getIf<sf::Event::KeyPressed>()->shift && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::H))
            {
                psync::Logger::get()->info(__help_stream.str().c_str());
            }

            /*CHNAGE MAP*/
            else if (event->getIf<sf::Event::KeyPressed>()->shift && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::M))
            {
                psync::Logger::get()->info("Changing Map...");
                __change_map();
            }

            /*CLEAR PATH*/
            else if (event->getIf<sf::Event::KeyPressed>()->shift && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::P))
            {
                psync::Logger::get()->info("CLEARING PATH...");
                __grid.clear_paths();
            }

            /*RESET GRID*/
            else if (event->getIf<sf::Event::KeyPressed>()->shift && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::R))
            {
                psync::Logger::get()->info("RESETTING GRID...");
                __grid.reset_grid();
            }
        }
    }
}

void VisualizationSystem::update()
{
    __main_window.clear();
    __main_window.draw(__grid);

    /* HACK: setting view messes up with drawings on RenderWindow */
    // __main_window.setView(__main_view);

    __main_window.display();
}

void VisualizationSystem::run()
{
    psync::Logger::get()->info(__help_stream.str().c_str());

    while (__main_window.isOpen())
    {
        handle_event();
        update();
    }
}

void VisualizationSystem::__set_zoom(const sf::Event::MouseWheelScrolled *scroll_event)
{
    if (__zoom_direction * scroll_event->delta == -1 || __zoom_factor > 1.2 || __zoom_factor < 0.8)
        __zoom_factor = 1.0f;
    else
        __zoom_factor -= (scroll_event->delta * 0.05);

    sf::Vector2f before_zoom = __main_window.mapPixelToCoords(sf::Mouse::getPosition(__main_window), __main_view);
    __main_view.zoom(__zoom_factor);
    sf::Vector2f after_zoom = __main_window.mapPixelToCoords(sf::Mouse::getPosition(__main_window), __main_view);

    __main_view.move(before_zoom - after_zoom);
    __zoom_direction = scroll_event->delta;
}

bool VisualizationSystem::__is_mouse_inside_window()
{
    sf::Vector2i current_mouse_pos = sf::Mouse::getPosition(__main_window);

    if ((current_mouse_pos.x < 0 || current_mouse_pos.x > __system_config.WIDTH) ||
        (current_mouse_pos.y < 0 || current_mouse_pos.y > __system_config.HEIGHT))
        return false;

    return true;
}

Coordinate VisualizationSystem::__draw_with_cell_type(psync::CellType cell_type)
{
    sf::Vector2i current_mouse_pos = sf::Mouse::getPosition(__main_window);
    Coordinate grid_cell = {current_mouse_pos.x / psync::VisualizationSystemConfig::CELL_SIZE,
                            current_mouse_pos.y / psync::VisualizationSystemConfig::CELL_SIZE};
    __grid.get_grid()[grid_cell.second][grid_cell.first].set_cell_type(cell_type);

    return grid_cell;
}

void VisualizationSystem::__get_available_maps()
{
    std::string map_directory = std::string(PROJECT_ROOT) + "/maps/";

    for(const auto& entry: std::filesystem::directory_iterator(map_directory))
    {
        if(std::filesystem::is_regular_file(entry))
        {
            __available_maps.push_back(entry.path().filename().stem());
        }
    }
}


void VisualizationSystem::__change_map()
{
    ++__selected_map_index;
    __selected_map_index = __selected_map_index % __available_maps.size();

    __grid.change_current_map(__available_maps[__selected_map_index]);
}

VisualizationSystem::~VisualizationSystem()
{
    delete __instance;
}
} // namespace psync
