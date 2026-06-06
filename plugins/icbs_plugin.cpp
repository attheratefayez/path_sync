#include "path_sync_core/solvers/cbs_solver.hpp"

extern "C"
{

const char *plugin_name() { return "ICBS_Solver"; }
bool plugin_is_optimal() { return true; }
bool plugin_is_multi_agent() { return true; }
void *plugin_create()
{
    auto *solver = new path_sync::solvers::mapf::CBS_Solver();
    solver->set_use_icbs(true);
    solver->set_use_cbsh(false);
    return solver;
}
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::mapf::CBS_Solver *>(p); }

}
