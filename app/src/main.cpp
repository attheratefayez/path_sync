#include "PathSyncApp.hpp"

void path_sync_loop();

int main()
{
    path_sync_loop();
}

void path_sync_loop()
{
    path_sync::PathSyncApp app{};
    app.solve_current_map();
}
