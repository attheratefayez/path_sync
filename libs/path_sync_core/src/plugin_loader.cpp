#ifdef _WIN32
    #include <windows.h>
#else
    #include <dlfcn.h>
#endif

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "path_sync_core/plugin_loader.hpp"

namespace fs = std::filesystem;

namespace path_sync
{

namespace
{

#ifdef _WIN32
using dl_handle = HMODULE;
dl_handle dl_open(const char *path) { return LoadLibraryA(path); }
void *dl_sym(dl_handle h, const char *name) { return (void *)GetProcAddress(h, name); }
bool dl_close(dl_handle h) { return FreeLibrary(h) != 0; }
const char *dl_error()
{
    static char buf[256];
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, nullptr, GetLastError(), 0, buf, sizeof buf, nullptr);
    return buf;
}
#else
using dl_handle = void *;
dl_handle dl_open(const char *path)
{
    auto *h = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    return h;
}
void *dl_sym(dl_handle h, const char *name) { return dlsym(h, name); }
bool dl_close(dl_handle h) { return dlclose(h) == 0; }
const char *dl_error() { return dlerror(); }
#endif

template <typename T>
T sym_or(dl_handle h, const char *name, T fallback)
{
    auto *p = dl_sym(h, name);
    if (!p)
        return fallback;
    return reinterpret_cast<T>(p);
}

using name_fn_t = const char *(*)();
using bool_fn_t = bool (*)();
using create_fn_t = void *(*)();
using destroy_fn_t = void (*)(void *);

} // anonymous namespace

// ── PluginLoader ─────────────────────────────────────────────────────

PluginLoader::~PluginLoader()
{
    for (auto &h : handles_)
    {
        if (h->instance && h->destroy_fn)
            h->destroy_fn(h->instance);
        if (h->dl_handle)
            dl_close(h->dl_handle);
    }
}

void PluginLoader::register_sa_solver(std::string name, bool optimal,
                                      std::unique_ptr<ISolver> (*factory)())
{
    auto *instance = factory().release();
    auto handle = std::make_unique<PluginHandle>();
    handle->name = std::move(name);
    handle->optimal = optimal;
    handle->is_ma = false;
    handle->instance = instance;
    handle->destroy_fn = [](void *p) { delete static_cast<ISolver *>(p); };
    sa_solvers_.push_back(static_cast<ISolver *>(instance));
    handles_.push_back(std::move(handle));
}

void PluginLoader::register_ma_solver(std::string name, bool optimal,
                                      std::unique_ptr<IMASolver> (*factory)())
{
    auto *instance = factory().release();
    auto handle = std::make_unique<PluginHandle>();
    handle->name = std::move(name);
    handle->optimal = optimal;
    handle->is_ma = true;
    handle->instance = instance;
    handle->destroy_fn = [](void *p) { delete static_cast<IMASolver *>(p); };
    ma_solvers_.push_back(static_cast<IMASolver *>(instance));
    handles_.push_back(std::move(handle));
}

bool PluginLoader::load_plugins(const std::string &directory)
{
    if (!fs::is_directory(directory))
        return false;

    bool any_ok = false;
    for (auto &entry : fs::directory_iterator(directory))
    {
        if (!entry.is_regular_file())
            continue;
        auto path = entry.path();
        auto ext = path.extension().string();
#ifdef _WIN32
        if (ext != ".dll")
            continue;
#else
        if (ext != ".so")
            continue;
#endif
        if (load_single(path.string()))
            any_ok = true;
    }
    return any_ok;
}

bool PluginLoader::load_single(const std::string &so_path)
{
    auto *handle = dl_open(so_path.c_str());
    if (!handle)
    {
        std::ostringstream ss;
        ss << "Plugin load failed [" << so_path << "]: " << dl_error();
        errors_.push_back(ss.str());
        return false;
    }

    auto *name = sym_or<name_fn_t>(handle, "plugin_name", nullptr);
    auto is_optimal = sym_or<bool_fn_t>(handle, "plugin_is_optimal", nullptr);
    auto is_ma = sym_or<bool_fn_t>(handle, "plugin_is_multi_agent", nullptr);
    auto create = sym_or<create_fn_t>(handle, "plugin_create", nullptr);
    auto destroy = sym_or<destroy_fn_t>(handle, "plugin_destroy", nullptr);

    if (!name || !create || !destroy)
    {
        std::ostringstream ss;
        ss << "Plugin missing required symbols [" << so_path << "]: "
           << "plugin_name=" << (name ? "ok" : "missing") << " "
           << "plugin_create=" << (create ? "ok" : "missing") << " "
           << "plugin_destroy=" << (destroy ? "ok" : "missing");
        errors_.push_back(ss.str());
        dl_close(handle);
        return false;
    }

    auto ph = std::make_unique<PluginHandle>();
    ph->dl_handle = handle;
    ph->name = name();
    ph->optimal = is_optimal ? is_optimal() : false;
    ph->is_ma = is_ma ? is_ma() : false;
    ph->destroy_fn = destroy;

    try
    {
        ph->instance = create();
    }
    catch (...)
    {
        std::ostringstream ss;
        ss << "Plugin constructor threw [" << so_path << "]";
        errors_.push_back(ss.str());
        dl_close(handle);
        return false;
    }

    if (!ph->instance)
    {
        errors_.push_back("Plugin create returned null [" + so_path + "]");
        dl_close(handle);
        return false;
    }

    if (ph->is_ma)
        ma_solvers_.push_back(static_cast<IMASolver *>(ph->instance));
    else
        sa_solvers_.push_back(static_cast<ISolver *>(ph->instance));

    handles_.push_back(std::move(ph));
    return true;
}

} // namespace path_sync
