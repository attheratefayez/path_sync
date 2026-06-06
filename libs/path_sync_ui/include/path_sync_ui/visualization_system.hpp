#ifndef __MOMAPF_VISUALIZATION_SYSTEM_HPP__
#define __MOMAPF_VISUALIZATION_SYSTEM_HPP__

#include <QComboBox>
#include <QLabel>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <deque>
#include <memory>

#include "PathSyncApp.hpp"
#include "path_sync_core/performance_mat.hpp"
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
    void update_status();
    void solve_async();

private:
    PathSyncApp& app_;
    GridWidget *grid_widget_;
    QPushButton *solve_btn_;
    QPushButton *cancel_btn_;
    QComboBox *solver_combo_;
    QSpinBox *scene_spin_;
    QSpinBox *map_spin_;
    QSpinBox *agent_spin_;
    QSpinBox *timeout_spin_;
    QLabel *status_label_;
    QLabel *solve_status_label_;
    QLabel *timeout_remaining_label_;
    QTimer *timeout_timer_;
    std::chrono::steady_clock::time_point solve_deadline_;
    QWidget *sidebar_;
    QPlainTextEdit *perf_text_;
    bool solving_ = false;
    struct PerfEntry
    {
        PerformanceMetrics pm;
        std::optional<MAPFMetrics> ma_met;
    };
    std::deque<PerfEntry> perf_buffer_;

    VisualizationSystem(VisualizationSystem const &) = delete;
    VisualizationSystem &operator=(VisualizationSystem const &) = delete;

    void setup_ui();
    void populate_solver_combo();
    void update_perf_sidebar();
    void reset_perf_buffer();
};

} // namespace path_sync
#endif
