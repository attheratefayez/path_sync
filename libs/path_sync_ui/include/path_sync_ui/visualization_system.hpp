#ifndef __MOMAPF_VISUALIZATION_SYSTEM_HPP__
#define __MOMAPF_VISUALIZATION_SYSTEM_HPP__

#include <QColor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
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
    ~VisualizationSystem() override = default;

protected:
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    PathSyncApp& app_;
    std::unique_ptr<VisualizationSystemConfig> system_config_;
    Grid grid_;
    QTimer *timer_;
    std::stringstream help_stream_;
    std::map<Qt::Key, std::function<void()>> key_bindings_;

    VisualizationSystem(VisualizationSystem const &) = delete;
    VisualizationSystem &operator=(VisualizationSystem const &) = delete;

    void setup_keybindings();
    Coordinate get_grid_cell_from_mouse_position(const QPoint &pos) const;
    bool is_point_inside_grid(const QPoint &point) const;
};

} // namespace path_sync
#endif
