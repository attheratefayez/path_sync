#ifndef __PATH_SYNC_PLUGIN_LOADER_HPP__
#define __PATH_SYNC_PLUGIN_LOADER_HPP__

#include <memory>
#include <string>
#include <vector>

#include "path_sync_core/solver_interface.hpp"
#include "path_sync_core/solvers/imo_solver.hpp"

namespace path_sync
{

class PluginLoader
{
  public:
    PluginLoader() = default;
    ~PluginLoader();

    void register_sa_solver(std::string name, bool optimal,
                            std::unique_ptr<ISolver> (*factory)());

    void register_ma_solver(std::string name, bool optimal,
                            std::unique_ptr<IMASolver> (*factory)());

    void register_mo_solver(std::string name, bool optimal,
                            std::unique_ptr<IMOSolver> (*factory)());

    bool load_plugins(const std::string &directory);

    const std::vector<ISolver *> &get_sa_solvers() const { return sa_solvers_; }
    const std::vector<IMASolver *> &get_ma_solvers() const { return ma_solvers_; }
    const std::vector<IMOSolver *> &get_mo_solvers() const { return mo_solvers_; }
    const std::vector<std::string> &get_errors() const { return errors_; }

  private:
    struct PluginHandle
    {
        void *dl_handle = nullptr;
        bool is_ma = false;
        bool is_mo = false;
        bool optimal = false;
        std::string name;
        void *instance = nullptr;
        void (*destroy_fn)(void *) = nullptr;

        explicit operator bool() const noexcept { return instance != nullptr; }
    };

    bool load_single(const std::string &so_path);

    std::vector<std::unique_ptr<PluginHandle>> handles_;
    std::vector<ISolver *> sa_solvers_;
    std::vector<IMASolver *> ma_solvers_;
    std::vector<IMOSolver *> mo_solvers_;
    std::vector<std::string> errors_;
};

} // namespace path_sync

#endif // __PATH_SYNC_PLUGIN_LOADER_HPP__
