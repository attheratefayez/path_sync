#ifndef __MOMAPF_VISUALIZATION_SYSTEM_HPP__
#define __MOMAPF_VISUALIZATION_SYSTEM_HPP__

#include <QColor>
#include <QComboBox>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPushButton>
#include <QResizeEvent>
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

class VisualizationSystem : public QWidget
{
    Q_OBJECT

public:
    explicit VisualizationSystem(PathSyncApp& app, QWidget *parent = nullptr);
    ~VisualizationSystem() override;

protected:
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void on_solver_combo_changed(int index);
    void on_solve_clicked() { app_.solve_current_scene(); }
    void on_clear_clicked() { app_.clear_paths(); }
    void on_reset_clicked() { app_.reset_grid(); }
    void on_prev_scene() { app_.request_previous_scene(); app_.clear_paths(); update_status(); }
    void on_next_scene() { app_.request_next_scene(); app_.clear_paths(); update_status(); }
    void on_next_map() { app_.request_next_map(); update_status(); }
    void update_status();

private:
    PathSyncApp& app_;
    std::unique_ptr<VisualizationSystemConfig> system_config_;
    Grid grid_;
    QTimer *timer_;
    std::stringstream help_stream_;
    std::map<Qt::Key, std::function<void()>> key_bindings_;

    QComboBox *solver_combo_;
    QLabel *status_label_;
    int toolbar_height_;

    VisualizationSystem(VisualizationSystem const &) = delete;
    VisualizationSystem &operator=(VisualizationSystem const &) = delete;

    void setup_keybindings();
    void setup_ui();
    void populate_solver_combo();
    Coordinate get_grid_cell_from_mouse_position(const QPoint &pos) const;
    bool is_point_inside_grid(const QPoint &point) const;
    QRect grid_area_rect() const;
};

} // namespace path_sync
#endif
