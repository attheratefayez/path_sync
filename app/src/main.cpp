#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "PathSyncApp.hpp"
// #include "path_sync_ui/visualization_system.hpp"
// #include "path_sync/visualization_system/visualization_system_config.hpp"

void test_loop();
void path_sync_loop();

// int path_sync::VisualizationSystemConfig::CELL_SIZE = 5;

int main()
{
    path_sync_loop();
    /*test_loop();*/
}

void path_sync_loop()
{
    // load path_sync configs
    // initialize the Visualization System
    // run the system
    // path_sync::VisualizationSystemConfig system_config =
        // path_sync::VisualizationSystemConfig("/home/fayez/Bugs/Cpp/path_sync/config/env_vars.yaml");
    
    path_sync::PathSyncApp app{};
    app.solve_current_scene();
}

void test_loop()
{
    // path_sync::VisualizationSystemConfig system_config =
    //     path_sync::VisualizationSystemConfig("/home/fayez/Bugs/Cpp/path_sync/config/env_vars.yaml");
    // // this is a comment
    // sf::RenderWindow window(sf::VideoMode({1800, 900}), "Path Sync", sf::Style::Titlebar | sf::Style::Close);
    // window.setFramerateLimit(60);
    //
    // /* SFML DRAWABLES */
    // /*sf::CircleShape shape(100.f);*/
    // /*shape.setFillColor(sf::Color::Green);*/
    //
    // /*path_sync::Cell new_cell(path_sync::CellType::DEFAULT, {100.0f, 100.0f});*/
    // path_sync::path_sync_core::MapData map_data(system_config.WIDTH / path_sync::VisualizationSystemConfig::CELL_SIZE,
    //                                   system_config.HEIGHT / path_sync::VisualizationSystemConfig::CELL_SIZE);
    // path_sync::Grid new_grid(map_data, path_sync::VisualizationSystemConfig::CELL_SIZE);
    //
    // sf::Clock deltaClock;
    // while (window.isOpen())
    // {
    //     while (const auto event = window.pollEvent())
    //     {
    //         if (event->is<sf::Event::Closed>())
    //         {
    //             window.close();
    //         }
    //     }
    //
    //     window.clear();
    //
    //     /*DRAW STUFFS*/
    //     new_grid.sync_with_map_data(map_data);
    //     window.draw(new_grid);
    //
    //     window.display();
    // }
}
