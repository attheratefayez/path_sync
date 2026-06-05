#ifndef __MOMAPF_VISUALIZATION_SYSTEM_HPP__
#define __MOMAPF_VISUALIZATION_SYSTEM_HPP__

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <functional>
#include <map>
#include <memory>
#include <sstream>

#include "PathSyncApp.hpp"
#include "path_sync_ui/grid.hpp"
#include "path_sync_ui/visualization_system_config.hpp"

namespace path_sync
{

class GridWidget;

class VisualizationSystem : public QWidget
{
    Q_OBJECT

public:
    explicit VisualizationSystem(PathSyncApp& app, QWidget *parent = nullptr);
    ~VisualizationSystem() override;

private slots:
    void on_solver_combo_changed(int index);
    void on_solve_clicked();
    void on_clear_clicked();
    void on_reset_clicked();
    bool on_prev_scene();
    bool on_next_scene();
    void on_prev_map();
    void on_next_map();
    void on_toggle_agent_mode();
    void update_status();
    void focus_grid();
    void solve_async();

private:
    PathSyncApp& app_;
    GridWidget *grid_widget_;
    QPushButton *solve_btn_;
    QPushButton *prev_btn_;
    QPushButton *next_btn_;
    QComboBox *solver_combo_;
    QLabel *status_label_;
    QTimer *scene_timer_;
    bool scene_dir_forward_ = true;
    bool solving_ = false;
    std::stringstream help_stream_;

    VisualizationSystem(VisualizationSystem const &) = delete;
    VisualizationSystem &operator=(VisualizationSystem const &) = delete;

    void setup_ui();
    void populate_solver_combo();
    void show_performance_dialog();
};

} // namespace path_sync
#endif
