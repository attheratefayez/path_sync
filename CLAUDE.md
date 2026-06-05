# Project context

## Build environment
- Compiled inside a **Docker container** via `docker-compose.yml`
- Container name: `cpp-dev`
- To build: `docker exec cpp-dev cmake --build /workspace/build -j$(nproc)`
- To configure: `docker exec cpp-dev cmake -S /workspace -B /workspace/build -DCMAKE_BUILD_TYPE=Release`
- To test: `docker exec cpp-dev /workspace/build/test/path_sync_test`
- Source is at `/workspace` inside the container (mounted from host)
- Build artifacts are at `/workspace/build` inside the container

## Project structure
- `libs/path_sync_core/` — core algorithms (solvers, map loader, path finder)
- `libs/path_sync_ui/` — Qt6 UI (visualization, grid, toolbar)
- `app/` — application entry point (PathSyncApp, main)
- `test/` — GTest test suite
