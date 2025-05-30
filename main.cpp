#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "path_sync/visualization_system/cell.hpp"
#include "path_sync/visualization_system/grid.hpp"
#include "path_sync/visualization_system/visualization_system.hpp"
#include "path_sync/visualization_system/visualization_system_config.hpp"

void test_loop();
void pfsync_loop();

int path_sync::VisualizationSystemConfig::CELL_SIZE = 50;
unsigned int path_sync::VisualizationSystemConfig::NUM_OF_OBJECTIVES = 1;

int main()
{
    pfsync_loop();
    /*test_loop();*/

}

void pfsync_loop()
{
    path_sync::VisualizationSystemConfig system_config =
        path_sync::VisualizationSystemConfig("/home/fayez/Bugs/Cpp/path_sync/config/env_vars.yaml");
    path_sync::VisualizationSystem::initialize(system_config);
    path_sync::VisualizationSystem::get()->run();
}

void test_loop()
{
    path_sync::VisualizationSystemConfig system_config =
        path_sync::VisualizationSystemConfig("/home/fayez/Bugs/Cpp/path_sync/config/env_vars.yaml");
    // this is a comment
    sf::RenderWindow window(sf::VideoMode({1800, 900}), "Path Sync", sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(60);

    /* SFML DRAWABLES */
    /*sf::CircleShape shape(100.f);*/
    /*shape.setFillColor(sf::Color::Green);*/

    /*path_sync::Cell new_cell(path_sync::CellType::DEFAULT, {100.0f, 100.0f});*/
    path_sync::Grid new_grid(system_config);

    sf::Clock deltaClock;
    while (window.isOpen())
    {
        while (const auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        window.clear();

        /*DRAW STUFFS*/
        window.draw(new_grid);

        window.display();
    }
}
