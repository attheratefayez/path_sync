#include <optional>
#include <iostream>

#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <imgui-SFML.h>
#include <imgui.h>
#include <yaml-cpp/yaml.h>

#include "path_sync/visualization_system.hpp"
#include "path_sync/logger.hpp"

void show_control_panel_imgui();


namespace psync{

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
{
    __main_window.create(
        sf::VideoMode({ system_config.WIDTH, system_config.HEIGHT }),
        system_config.TITLE,
        sf::Style::Titlebar | sf::Style::Close
    );
    __main_window.setVerticalSyncEnabled(true);

    __main_view.setViewport(sf::FloatRect(
        {0, 0}, 
        {static_cast<float>(system_config.WIDTH), static_cast<float>(system_config.HEIGHT)}
    ));

    __zoom_factor = 1.0;
    __zoom_direction = 1;
    __main_window.setView(__main_view);

    if(!ImGui::SFML::Init(__main_window))
    {
        psync::Logger::get()->error("ImGui initialization failed.");
    }

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
}


// MEMBER FUNCTIONS

void VisualizationSystem::handle_event()
{
    while(std::optional<sf::Event> event = __main_window.pollEvent())
    {
        ImGui::SFML::ProcessEvent(__main_window, *event);

        if (event->is<sf::Event::Closed>())
            __main_window.close();

        if (event->is<sf::Event::MouseWheelScrolled>()) {
            __set_zoom(event->getIf<sf::Event::MouseWheelScrolled>());
        }
    }
}

void VisualizationSystem::update()
{
    ImGui::SFML::Update(__main_window, __deltaClock.restart());
    ImGui::ShowDemoWindow();

    show_control_panel_imgui();

    __main_window.clear();
    __main_window.setView(__main_view);
    ImGui::SFML::Render(__main_window);
    __main_window.display();

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

void VisualizationSystem::run()
{
    while (__main_window.isOpen()) 
    {
        handle_event();
        update();
    }
    ImGui::SFML::Shutdown();
}




VisualizationSystem::~VisualizationSystem()
{
    delete __instance;
}
} // END OF NAMESPACE PSYNC

void show_control_panel_imgui()
{
    bool* p_open = nullptr;
    ImGuiWindowFlags control_panel_window_flags = 0;
    control_panel_window_flags |= ImGuiWindowFlags_NoScrollbar;
    control_panel_window_flags |= ImGuiWindowFlags_AlwaysAutoResize;
    control_panel_window_flags |= ImGuiWindowFlags_NoBackground;

    if(!ImGui::Begin("Control Panel", p_open, control_panel_window_flags))
    {
        ImGui::End();
        return;
    }

    const char* items[] = {"A*", "Dijkstra", "BFS", "DFS"};
    static int selected_item_idx = 0;
    const char* combo_preview_value = items[selected_item_idx];

    if(ImGui::BeginCombo("Algorithms", combo_preview_value))
    {
        for(int counter = 0; counter < IM_ARRAYSIZE(items); counter++)
        {
            const bool is_selected = (selected_item_idx == counter);
            if(ImGui::Selectable(items[counter], is_selected))
            {
                selected_item_idx = counter;
            }
            
            if(is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::End();
}
