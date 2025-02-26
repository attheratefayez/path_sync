#include <iostream>
#include <optional>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <yaml-cpp/yaml.h>

#include "path_sync/logger.hpp"
#include "path_sync/visualization_system.hpp"


namespace psync {

// STATIC MEMBERS
VisualizationSystem* VisualizationSystem::__instance = nullptr;

void VisualizationSystem::initialize(VisualizationSystemConfig& system_config)
{
    __instance = new VisualizationSystem(system_config);
}

VisualizationSystem* VisualizationSystem::get()
{
    if (__instance == nullptr)
        psync::Logger::get()->error("VisualizationSystem not initialized.");
    return __instance;
}

// CONSTRUCTOR
VisualizationSystem::VisualizationSystem(VisualizationSystemConfig& system_config)
    : __system_config(system_config)
    , __grid(system_config)
{
    __main_window.create(
        sf::VideoMode({ system_config.WIDTH, system_config.HEIGHT }),
        system_config.TITLE,
        sf::Style::Titlebar | sf::Style::Close);
    __main_window.setFramerateLimit(system_config.FRAMERATE);

    __main_view.setViewport(sf::FloatRect(
        { 0, 0 },
        { static_cast<float>(system_config.WIDTH), static_cast<float>(system_config.HEIGHT) }));

    __zoom_factor = 1.0;
    __zoom_direction = 1;

    /* HACK: setting view messes up with drawings on RenderWindow */
    /*__main_window.setView(__main_view);*/

}

// MEMBER FUNCTIONS
void VisualizationSystem::handle_event()
{
    while (std::optional<sf::Event> event = __main_window.pollEvent()) {

        if (event->is<sf::Event::Closed>())
            __main_window.close();

        if (event->is<sf::Event::MouseButtonPressed>()) 
        { 
            /*DRAWING START POSITION*/
            if (
                event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left
                && __is_mouse_inside_window() 
                && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)
            ) 
            {
                sf::Vector2i start_point = __draw_with_cell_type(psync::CellType::START);
                __grid.get_start_points().push_back(start_point);
            }

            /*DRAWING END POSITION*/
            else if (
                event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left
                && __is_mouse_inside_window() 
                && sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)
            ) 
            {
                sf::Vector2i end_point = __draw_with_cell_type(psync::CellType::END);
                __grid.get_end_points().push_back(end_point);
            }


            /*DRAWING WALL*/
            else if (
                event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left
                && __is_mouse_inside_window()
            ) 
            {
                __draw_with_cell_type(psync::CellType::WALL);
            }

            /*Erasing */
            else if (event->getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Right) 
            {
                __draw_with_cell_type(psync::CellType::DEFAULT);
            }
        }

        if (event->is<sf::Event::MouseMoved>()) {

            /*DRAWING WALL*/
            if (__is_mouse_inside_window() && sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
                __draw_with_cell_type(psync::CellType::WALL);
            }

            /*ERASING*/
            else if (__is_mouse_inside_window() && sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) {
                sf::Vector2i current_mouse_pos = sf::Mouse::getPosition();
                __draw_with_cell_type(psync::CellType::DEFAULT);
            }
        }

        if (event->is<sf::Event::MouseWheelScrolled>()) {
            __set_zoom(event->getIf<sf::Event::MouseWheelScrolled>());
        }
    }
}

void VisualizationSystem::update()
{
    __main_window.clear();
    /*__main_window.draw(__test_rect);*/
    __main_window.draw(__grid);

    /* HACK: setting view messes up with drawings on RenderWindow */
    /*__main_window.setView(__main_view);*/

    __main_window.display();
}

void VisualizationSystem::run()
{
    while (__main_window.isOpen()) {
        handle_event();
        update();
    }
}

void VisualizationSystem::__set_zoom(const sf::Event::MouseWheelScrolled* scroll_event)
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

    if (
        (current_mouse_pos.x < 0 || current_mouse_pos.x > __system_config.WIDTH) || (current_mouse_pos.y < 0 || current_mouse_pos.y > __system_config.HEIGHT))
        return false;

    return true;
}

sf::Vector2i VisualizationSystem::__draw_with_cell_type(psync::CellType cell_type)
{
    sf::Vector2i current_mouse_pos = sf::Mouse::getPosition(__main_window);
    sf::Vector2i grid_cell = {
        current_mouse_pos.x / psync::VisualizationSystemConfig::CELL_SIZE,
        current_mouse_pos.y / psync::VisualizationSystemConfig::CELL_SIZE
    };
    __grid.get_grid()[grid_cell.y][grid_cell.x].set_cell_type(cell_type);

    return grid_cell;
}

VisualizationSystem::~VisualizationSystem()
{
    delete __instance;
}
} // END OF NAMESPACE PSYNC


