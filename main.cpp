#include <QApplication>

#include "PathSyncApp.hpp"
#include "path_sync_ui/visualization_system.hpp"
#include "path_sync_ui/visualization_system_config.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    path_sync::PathSyncApp app_logic;
    path_sync::VisualizationSystem widget(app_logic);
    widget.show();

    return app.exec();
}
