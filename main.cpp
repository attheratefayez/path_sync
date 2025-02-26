#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "imgui-SFML.h"
#include "imgui.h"

#include "path_sync/logger.hpp"
#include "path_sync/cell.hpp"
#include "path_sync/grid.hpp"
#include "path_sync/visualization_system.hpp"
#include "path_sync/visualization_system_config.hpp"

void test_loop();
void pfsync_loop();

int psync::VisualizationSystemConfig::CELL_SIZE = 4;

int main()
{
    pfsync_loop();
    /*test_loop();*/
}

void pfsync_loop()
{
    psync::VisualizationSystemConfig system_config = psync::VisualizationSystemConfig("/home/fayez/Bugs/Cpp/path_sync/config/env_vars.yaml");
    psync::VisualizationSystem::initialize(system_config);
    psync::VisualizationSystem::get()->run();

}

void test_loop()
{
    psync::VisualizationSystemConfig system_config = psync::VisualizationSystemConfig("/home/fayez/Bugs/Cpp/path_sync/config/env_vars.yaml");
    // this is a comment
    sf::RenderWindow window(sf::VideoMode({1800, 900}), "PathSync", sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(60);


    if(!ImGui::SFML::Init(window))
    {
        psync::Logger::get()->info("ImGui initialization failed.");
    };

    /* SFML DRAWABLES */
    /*sf::CircleShape shape(100.f);*/
    /*shape.setFillColor(sf::Color::Green);*/

    /*psync::Cell new_cell(psync::CellType::DEFAULT, {100.0f, 100.0f});*/
    psync::Grid new_grid(system_config);


    sf::Clock deltaClock;
    while (window.isOpen()) {
        while (const auto event = window.pollEvent()) {
            ImGui::SFML::ProcessEvent(window, *event);

            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::ShowDemoWindow();

        ImGui::Begin("Control Panel");
        ImGui::Button("Look at this pretty button");
        ImGui::End();

        window.clear();


        /*DRAW STUFFS*/
        window.draw(new_grid);
        
        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();

}


