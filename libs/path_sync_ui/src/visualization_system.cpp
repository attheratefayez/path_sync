#include "path_sync_ui/visualization_system.hpp"

#include <QPainter>
#include <QFont>

#include "path_sync_core/logger.hpp"

namespace path_sync
{

VisualizationSystem::~VisualizationSystem() = default;

VisualizationSystem::VisualizationSystem(PathSyncApp& app, QWidget *parent)
    : QWidget(parent)
    , app_(app)
    , grid_()
    , timer_(new QTimer(this))
{
    setWindowTitle("Path Sync");
    resize(1850, 950);

    connect(timer_, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
    timer_->start(16);

    setup_keybindings();

    help_stream_ << "Available Key Bindings:" << "\n"
                 << "C    - Change solver" << "\n"
                 << "A    - Toggle agent mode" << "\n"
                 << "Space - Solve current scene" << "\n"
                 << "Shift+H - Help (this message)" << "\n"
                 << "Shift+M - Next map" << "\n"
                 << "Shift+P - Clear paths" << "\n"
                 << "Shift+R - Reset grid" << "\n"
                 << "[    - Previous scene" << "\n"
                 << "]    - Next scene" << "\n"
                 << "Mouse click - Toggle wall";

    Logger::get().info(help_stream_.str().c_str());
}

void VisualizationSystem::setup_keybindings()
{
    key_bindings_[Qt::Key_C] = [this]() { app_.change_solver(); Logger::get().info((std::string("Solver: ") + std::string(app_.get_current_solver_name())).c_str()); };
    key_bindings_[Qt::Key_A] = [this]() { app_.toggle_agent_mode(); };
    key_bindings_[Qt::Key_Space] = [this]() { app_.solve_current_scene(); };
    key_bindings_[Qt::Key_H] = [this]() { Logger::get().info(help_stream_.str().c_str()); };
    key_bindings_[Qt::Key_M] = [this]() { app_.request_next_map(); };
    key_bindings_[Qt::Key_P] = [this]() { app_.clear_paths(); };
    key_bindings_[Qt::Key_R] = [this]() { app_.reset_grid(); };
    key_bindings_[Qt::Key_BracketLeft]  = [this]() { app_.request_previous_scene(); app_.clear_paths(); };
    key_bindings_[Qt::Key_BracketRight] = [this]() { app_.request_next_scene(); app_.clear_paths(); };
}

void VisualizationSystem::paintEvent(QPaintEvent * /*event*/)
{
    QPainter painter(this);

    painter.fillRect(rect(), QColor(240, 240, 240));

    auto map_data = app_.get_current_map_data();
    if (!map_data)
        return;

    grid_.sync_with_map_data(*map_data);

    int cell_size = grid_.get_cell_size();
    for (int y = 0; y < grid_.get_height(); ++y)
    {
        for (int x = 0; x < grid_.get_width(); ++x)
        {
            const Cell &cell = grid_.get_cell(x, y);
            QColor color = cell_type_to_color(cell.type);
            painter.fillRect(x * cell_size, y * cell_size, cell_size, cell_size, color);
        }
    }

    // Draw thin grid lines
    painter.setPen(QPen(QColor(200, 200, 200), 1));
    for (int x = 0; x <= grid_.get_width(); ++x)
        painter.drawLine(x * cell_size, 0, x * cell_size, grid_.get_height() * cell_size);
    for (int y = 0; y <= grid_.get_height(); ++y)
        painter.drawLine(0, y * cell_size, grid_.get_width() * cell_size, y * cell_size);

    // Draw solver name
    painter.setPen(QColor(0, 0, 0));
    QFont font("monospace", 12);
    painter.setFont(font);
    QString solver_text = QString::fromStdString(
        std::string(app_.get_current_solver_name()));
    painter.drawText(10, 20, solver_text);
}

void VisualizationSystem::keyPressEvent(QKeyEvent *event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());

    if (event->modifiers() & Qt::ShiftModifier)
    {
        // Shift+key mappings
        auto it = key_bindings_.find(key);
        if (it != key_bindings_.end())
            it->second();
        return;
    }

    auto it = key_bindings_.find(key);
    if (it != key_bindings_.end())
        it->second();
}

void VisualizationSystem::mousePressEvent(QMouseEvent *event)
{
    if (event->button() != Qt::LeftButton)
        return;

    auto map_data = app_.get_current_map_data();
    if (!map_data)
        return;

    QPoint pos = event->pos();
    if (!is_point_inside_grid(pos))
        return;

    Coordinate coord = get_grid_cell_from_mouse_position(pos);

    CellType current = map_data->get_cell_type(coord);
    CellType new_type = (current == CellType::WALL) ? CellType::DEFAULT : CellType::WALL;
    map_data->set_cell_type(coord, new_type);
}

void VisualizationSystem::mouseMoveEvent(QMouseEvent *event)
{
    if (!(event->buttons() & Qt::LeftButton))
        return;

    auto map_data = app_.get_current_map_data();
    if (!map_data)
        return;

    QPoint pos = event->pos();
    if (!is_point_inside_grid(pos))
        return;

    Coordinate coord = get_grid_cell_from_mouse_position(pos);
    map_data->set_cell_type(coord, CellType::WALL);
}

Coordinate VisualizationSystem::get_grid_cell_from_mouse_position(const QPoint &pos) const
{
    int cell_size = grid_.get_cell_size();
    return Coordinate(pos.x() / cell_size, pos.y() / cell_size);
}

bool VisualizationSystem::is_point_inside_grid(const QPoint &point) const
{
    int cell_size = grid_.get_cell_size();
    int max_x = grid_.get_width() * cell_size;
    int max_y = grid_.get_height() * cell_size;
    return point.x() >= 0 && point.x() < max_x &&
           point.y() >= 0 && point.y() < max_y;
}

} // namespace path_sync
