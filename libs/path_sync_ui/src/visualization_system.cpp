#include <yaml-cpp/yaml.h>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <optional>
#include <sstream>

#include "SFML/Window/Keyboard.hpp"
#include "path_sync_core/logger.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_ui/grid.hpp"
#include "path_sync_ui/visualization_system.hpp"

namespace path_sync
{

// CONSTRUCTOR
// TODO: add a way to change the map mode {MAP, FREE}
VisualizationSystem::VisualizationSystem(PathSyncApp &app, std::unique_ptr<VisualizationSystemConfig> system_config)
    : app_(app)
    , system_config_(std::move(system_config))
    , grid_(*app.get_current_map_data(), system_config_->CELL_SIZE)
{
    main_window_.create(sf::VideoMode({system_config_->WIDTH, system_config_->HEIGHT}), system_config_->TITLE,
                        sf::Style::Titlebar | sf::Style::Close);
    main_window_.setFramerateLimit(system_config_->FRAMERATE);

    setup_keybindings();

    help_stream_ << "\n";
    help_stream_ << "\t\tPath Sync Controls\n";
    help_stream_ << "\ts    + Mouse-Left  :   Draw Start Point.\n";
    help_stream_ << "\te    + Mouse-Left  :   Draw End Point.\n";
    help_stream_ << "\tMouse-Left  + Drag :   Draw Wall.\n";
    help_stream_ << "\tMouse-Right + Drag :   Erase Wall.\n";
    help_stream_ << "\tc                  :   Change Solver.\n";
    help_stream_ << "\tSpace              :   Find Solution.\n";
    help_stream_ << "\tShift-H            :   Show This Help.\n";
    help_stream_ << "\tShift-P            :   Clear Path.\n";
    help_stream_ << "\tShift-M            :   Change Map.\n";
    help_stream_ << "\tShift-R            :   Clear Grid.\n";
}

// MEMBER FUNCTIONS
void VisualizationSystem::setup_keybindings()
{
    key_bindings_[sf::Keyboard::Key::C] = [this]() {
        path_sync::Logger::get().info("Changing Solver...");
        // app_.change_solver(); // TODO: Implement in PathSyncApp
    };

    key_bindings_[sf::Keyboard::Key::Space] = [this]() {
        if (app_.solve_current_scene())
        {
        }
        else
        {
            path_sync::Logger::get().warn("No Solution Found.");
        }
    };

    key_bindings_[sf::Keyboard::Key::H] = [this]() {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) ||
            sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift))
        {
            path_sync::Logger::get().info(help_stream_.str().c_str());
        }
    };

    key_bindings_[sf::Keyboard::Key::M] = [this]() {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) ||
            sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift))
        {
            path_sync::Logger::get().info("Changing Map...");
            app_.request_next_map();
        }
    };

    key_bindings_[sf::Keyboard::Key::P] = [this]() {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) ||
            sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift))
        {
            path_sync::Logger::get().info("CLEARING PATH...");
            app_.clear_paths();
        }
    };

    key_bindings_[sf::Keyboard::Key::R] = [this]() {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) ||
            sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift))
        {
            path_sync::Logger::get().info("RESETTING GRID...");
            // app_.reset_grid(); // TODO: Implement in PathSyncApp
        }
    };

    key_bindings_[sf::Keyboard::Key::RBracket] = [this]() {
        if(!app_.request_next_scene())
        {
            path_sync::Logger::get().warn("No next scene.");
            return;
        }
        app_.clear_paths();
    };
}

void VisualizationSystem::handle_event()
{
    const sf::Vector2i mouse_position = sf::Mouse::getPosition(main_window_);

    while (std::optional<sf::Event> event = main_window_.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
        {
            main_window_.close();
        }

        if (auto key_pressed = event->getIf<sf::Event::KeyPressed>())
        {
            handle_key_press(*key_pressed);
        }
        else if (auto mouse_button_pressed = event->getIf<sf::Event::MouseButtonPressed>())
        {
            handle_mouse_button_press(*mouse_button_pressed, mouse_position);
        }
        else if (auto mouse_moved = event->getIf<sf::Event::MouseMoved>())
        {
            handle_mouse_move(*mouse_moved, mouse_position);
        }
    }
}

void VisualizationSystem::handle_key_press(const sf::Event::KeyPressed &key_event)
{
    if (key_bindings_.count(key_event.code))
    {
        key_bindings_[key_event.code]();
    }
}

void VisualizationSystem::handle_mouse_button_press(const sf::Event::MouseButtonPressed &mouse_event,
                                                    const sf::Vector2i &mouse_position)
{
    // if (mouse_event.button == sf::Mouse::Button::Left && is_point_inside_window_bounds(mouse_position) &&
    //     sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
    // {
    //     Coordinate start_point = get_grid_cell_from_mouse_position(mouse_position);
    //     // app_.add_start_point(start_point); // TODO: Implement in PathSyncApp
    //     app_.get_map_data().set_cell_type(start_point.first, start_point.second, CellType::START);
    // }
    // else if (mouse_event.button == sf::Mouse::Button::Left && is_point_inside_window_bounds(mouse_position) &&
    //          sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E))
    // {
    //     Coordinate end_point = get_grid_cell_from_mouse_position(mouse_position);
    //     // app_.add_end_point(end_point); // TODO: Implement in PathSyncApp
    //     app_.get_map_data().set_cell_type(end_point.first, end_point.second, CellType::END);
    // }
    // else if (mouse_event.button == sf::Mouse::Button::Left && is_point_inside_window_bounds(mouse_position))
    // {
    //     Coordinate wall_pos = get_grid_cell_from_mouse_position(mouse_position);
    //     app_.get_map_data().set_cell_type(wall_pos.first, wall_pos.second, CellType::WALL);
    // }
    // else if (mouse_event.button == sf::Mouse::Button::Right)
    // {
    //     Coordinate default_pos = get_grid_cell_from_mouse_position(mouse_position);
    //     app_.get_map_data().set_cell_type(default_pos.first, default_pos.second, CellType::DEFAULT);
    // }
}

void VisualizationSystem::handle_mouse_move(const sf::Event::MouseMoved &mouse_event,
                                            const sf::Vector2i &mouse_position)
{
    // if (is_point_inside_window_bounds(mouse_position) && sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
    // {
    //     Coordinate wall_pos = get_grid_cell_from_mouse_position(mouse_position);
    //     app_.get_map_data().set_cell_type(wall_pos.first, wall_pos.second, CellType::WALL);
    // }
    // else if (is_point_inside_window_bounds(mouse_position) && sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
    // {
    //     Coordinate default_pos = get_grid_cell_from_mouse_position(mouse_position);
    //     app_.get_map_data().set_cell_type(default_pos.first, default_pos.second, CellType::DEFAULT);
    // }
}

void VisualizationSystem::update()
{
    main_window_.clear();
    grid_.sync_with_map_data(*app_.get_current_map_data());
    main_window_.draw(grid_);
    main_window_.display();
}

void VisualizationSystem::run()
{
    path_sync::Logger::get().info(help_stream_.str().c_str());

    while (main_window_.isOpen())
    {
        handle_event();
        update();
    }
}

bool VisualizationSystem::is_point_inside_window_bounds(const sf::Vector2i &point)
{
    if ((point.x < 0 || point.x > system_config_->WIDTH) || (point.y < 0 || point.y > system_config_->HEIGHT))
        return false;

    return true;
}

Coordinate VisualizationSystem::get_grid_cell_from_mouse_position(const sf::Vector2i &mouse_position)
{
    Coordinate grid_cell = {mouse_position.x / system_config_->CELL_SIZE, mouse_position.y / system_config_->CELL_SIZE};
    return grid_cell;
}

} // namespace path_sync
