#include "path_sync_ui/visualization_system.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
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
    , toolbar_height_(0)
{
    setWindowTitle("Path Sync");
    resize(1850, 1000);

    connect(timer_, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
    timer_->start(16);

    setup_ui();
    setup_keybindings();
    populate_solver_combo();

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
    update_status();
}

void VisualizationSystem::setup_ui()
{
    auto *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);

    auto *toolbar = new QWidget(this);
    auto *tb_layout = new QHBoxLayout(toolbar);
    tb_layout->setContentsMargins(6, 4, 6, 4);

    auto *solve_btn = new QPushButton("Solve");
    auto *clear_btn = new QPushButton("Clear");
    auto *reset_btn = new QPushButton("Reset");
    auto *prev_btn = new QPushButton("< Scene");
    auto *next_btn = new QPushButton("Scene >");
    auto *map_btn = new QPushButton("Next Map");

    solver_combo_ = new QComboBox;

    connect(solve_btn, &QPushButton::clicked, this, &VisualizationSystem::on_solve_clicked);
    connect(clear_btn, &QPushButton::clicked, this, &VisualizationSystem::on_clear_clicked);
    connect(reset_btn, &QPushButton::clicked, this, &VisualizationSystem::on_reset_clicked);
    connect(prev_btn,  &QPushButton::clicked, this, &VisualizationSystem::on_prev_scene);
    connect(next_btn,  &QPushButton::clicked, this, &VisualizationSystem::on_next_scene);
    connect(map_btn,   &QPushButton::clicked, this, &VisualizationSystem::on_next_map);
    connect(solver_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VisualizationSystem::on_solver_combo_changed);

    tb_layout->addWidget(solve_btn);
    tb_layout->addWidget(clear_btn);
    tb_layout->addWidget(reset_btn);
    tb_layout->addWidget(prev_btn);
    tb_layout->addWidget(next_btn);
    tb_layout->addWidget(map_btn);
    tb_layout->addWidget(new QLabel(" Solver:"));
    tb_layout->addWidget(solver_combo_);
    tb_layout->addStretch();

    main_layout->addWidget(toolbar);

    status_label_ = new QLabel(this);
    status_label_->setContentsMargins(6, 2, 6, 2);
    main_layout->addWidget(status_label_);
}

void VisualizationSystem::populate_solver_combo()
{
    solver_combo_->blockSignals(true);
    for (const auto &name : app_.get_solver_names())
        solver_combo_->addItem(QString::fromStdString(name));
    solver_combo_->blockSignals(false);
}

void VisualizationSystem::on_solver_combo_changed(int index)
{
    if (index < 0)
        return;
    app_.select_solver_by_index(static_cast<std::size_t>(index));
    Logger::get().info((std::string("Solver: ") + std::string(app_.get_current_solver_name())).c_str());
    update_status();
}

void VisualizationSystem::update_status()
{
    auto map_data = app_.get_current_map_data();
    if (!map_data)
        return;
    std::string text = "Solver: ";
    text += app_.get_current_solver_name();
    text += "  |  Map: ";
    text += app_.get_current_map_name();
    text += "  |  Scene: ";
    text += std::to_string(app_.get_scene_index() + 1);
    text += " / ";
    text += std::to_string(app_.get_total_scenes());
    status_label_->setText(QString::fromStdString(text));
}

void VisualizationSystem::setup_keybindings()
{
    key_bindings_[Qt::Key_C] = [this]() {
        app_.change_solver();
        solver_combo_->setCurrentIndex((solver_combo_->currentIndex() + 1) % solver_combo_->count());
        Logger::get().info((std::string("Solver: ") + std::string(app_.get_current_solver_name())).c_str());
        update_status();
    };
    key_bindings_[Qt::Key_A] = [this]() { app_.toggle_agent_mode(); update_status(); };
    key_bindings_[Qt::Key_Space] = [this]() { app_.solve_current_scene(); };
    key_bindings_[Qt::Key_H] = [this]() { Logger::get().info(help_stream_.str().c_str()); };
    key_bindings_[Qt::Key_M] = [this]() { app_.request_next_map(); update_status(); };
    key_bindings_[Qt::Key_P] = [this]() { app_.clear_paths(); };
    key_bindings_[Qt::Key_R] = [this]() { app_.reset_grid(); };
    key_bindings_[Qt::Key_BracketLeft]  = [this]() { app_.request_previous_scene(); app_.clear_paths(); update_status(); };
    key_bindings_[Qt::Key_BracketRight] = [this]() { app_.request_next_scene(); app_.clear_paths(); update_status(); };
}

QRect VisualizationSystem::grid_area_rect() const
{
    return QRect(0, toolbar_height_, width(), height() - toolbar_height_ - status_label_->height());
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
    QRect area = grid_area_rect();
    int ox = area.x();
    int oy = area.y();

    for (int y = 0; y < grid_.get_height(); ++y)
    {
        for (int x = 0; x < grid_.get_width(); ++x)
        {
            const Cell &cell = grid_.get_cell(x, y);
            QColor color = cell_type_to_color(cell.type);
            painter.fillRect(ox + x * cell_size, oy + y * cell_size, cell_size, cell_size, color);
        }
    }

    painter.setPen(QPen(QColor(200, 200, 200), 1));
    for (int x = 0; x <= grid_.get_width(); ++x)
        painter.drawLine(ox + x * cell_size, oy, ox + x * cell_size, oy + grid_.get_height() * cell_size);
    for (int y = 0; y <= grid_.get_height(); ++y)
        painter.drawLine(ox, oy + y * cell_size, ox + grid_.get_width() * cell_size, oy + y * cell_size);
}

void VisualizationSystem::keyPressEvent(QKeyEvent *event)
{
    Qt::Key key = static_cast<Qt::Key>(event->key());

    if (event->modifiers() & Qt::ShiftModifier)
    {
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

void VisualizationSystem::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    toolbar_height_ = 0;
    for (QObject *child : children())
    {
        if (auto *tb = qobject_cast<QWidget *>(child))
        {
            if (tb != status_label_)
                toolbar_height_ = std::max(toolbar_height_, tb->y() + tb->height());
        }
    }
}

Coordinate VisualizationSystem::get_grid_cell_from_mouse_position(const QPoint &pos) const
{
    QRect area = grid_area_rect();
    int cell_size = grid_.get_cell_size();
    return Coordinate((pos.x() - area.x()) / cell_size, (pos.y() - area.y()) / cell_size);
}

bool VisualizationSystem::is_point_inside_grid(const QPoint &point) const
{
    QRect area = grid_area_rect();
    int cell_size = grid_.get_cell_size();
    int max_x = area.x() + grid_.get_width() * cell_size;
    int max_y = area.y() + grid_.get_height() * cell_size;
    return point.x() >= area.x() && point.x() < max_x &&
           point.y() >= area.y() && point.y() < max_y;
}

} // namespace path_sync
