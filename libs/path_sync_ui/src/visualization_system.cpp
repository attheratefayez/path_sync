#include "path_sync_ui/visualization_system.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QPainter>
#include <QFont>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <algorithm>
#include <functional>

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

#include "path_sync_core/logger.hpp"

namespace path_sync
{

// ── GridWidget: fixed-size viewport with pan + zoom ──────────────────────

class GridWidget : public QWidget
{
    Q_OBJECT

public:
    GridWidget(PathSyncApp &app, QWidget *parent = nullptr)
        : QWidget(parent), app_(app), grid_()
    {
        setFocusPolicy(Qt::StrongFocus);
        setup_keybindings();
    }

    void sync_and_update()
    {
        auto map_data = app_.get_current_map_data();
        if (!map_data)
            return;
        grid_.sync_with_map_data(*map_data);
        update();
    }

    void reset_view()
    {
        zoom_ = 1.0;
        center_view();
    }

    void center_view()
    {
        auto map_data = app_.get_current_map_data();
        if (!map_data)
            return;
        grid_.sync_with_map_data(*map_data);
        double cell = static_cast<double>(grid_.get_cell_size()) * zoom_;
        double mw = grid_.get_width() * cell;
        double mh = grid_.get_height() * cell;
        pan_x_ = (width() - mw) / 2.0;
        pan_y_ = (height() - mh) / 2.0;
        update();
    }

    void set_solve_callback(std::function<void()> cb) { solve_callback_ = std::move(cb); }

    void setup_keybindings()
    {
        key_bindings_[Qt::Key_Space] = [this]() {
            if (solve_callback_) solve_callback_();
        };
        key_bindings_[Qt::Key_H]     = [this]() {
            Logger::get().info(
                "Key Bindings:\n"
                "C       - Cycle solver\n"
                "A       - Toggle agent mode\n"
                "Space   - Solve\n"
                "H       - This help\n"
                "M       - Next map\n"
                "P       - Clear paths\n"
                "R       - Reset grid\n"
                "[ / ]   - Prev / Next scene\n"
                "Click   - Toggle wall\n"
                "Drag    - Draw walls\n"
                "MClick  - Pan\n"
                "Wheel   - Zoom");
        };
    }

    Qt::Key last_key() const { return last_key_; }
    void set_last_key(Qt::Key k) { last_key_ = k; }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.fillRect(rect(), QColor(48, 48, 48));

        auto map_data = app_.get_current_map_data();
        if (!map_data)
            return;

        grid_.sync_with_map_data(*map_data);

        double cell = static_cast<double>(grid_.get_cell_size()) * zoom_;
        int gw = static_cast<int>(grid_.get_width() * cell);
        int gh = static_cast<int>(grid_.get_height() * cell);

        p.setClipRect(rect());

        for (int y = 0; y < grid_.get_height(); ++y)
        {
            for (int x = 0; x < grid_.get_width(); ++x)
            {
                const Cell &c = grid_.get_cell(x, y);
                p.fillRect(QRectF(pan_x_ + x * cell, pan_y_ + y * cell, cell, cell),
                           cell_type_to_color(c.type));
            }
        }

        p.setPen(QPen(QColor(100, 100, 100), 1));
        for (int x = 0; x <= grid_.get_width(); ++x)
        {
            double lx = pan_x_ + x * cell;
            p.drawLine(QPointF(lx, pan_y_), QPointF(lx, pan_y_ + gh));
        }
        for (int y = 0; y <= grid_.get_height(); ++y)
        {
            double ly = pan_y_ + y * cell;
            p.drawLine(QPointF(pan_x_, ly), QPointF(pan_x_ + gw, ly));
        }

        // viewport border
        p.setPen(QPen(QColor(102, 102, 102), 2));
        p.drawRect(rect().adjusted(1, 1, -1, -1));
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        bool shifted = event->modifiers() & Qt::ShiftModifier;
        Qt::Key key = static_cast<Qt::Key>(event->key());

        if (!shifted && (key == Qt::Key_C || key == Qt::Key_A ||
                         key == Qt::Key_M || key == Qt::Key_BracketLeft ||
                         key == Qt::Key_BracketRight))
        {
            last_key_ = key;
            emit key_forwarded(key);
            return;
        }
        if (shifted && (key == Qt::Key_M || key == Qt::Key_P || key == Qt::Key_R))
        {
            last_key_ = key;
            emit key_forwarded(static_cast<Qt::Key>(key | 0x1000));
            return;
        }

        handle_key(key, shifted);
        update();
    }

    void wheelEvent(QWheelEvent *event) override
    {
        if (!(event->modifiers() & Qt::ControlModifier))
            return;

        double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
        double mx = event->position().x();
        double my = event->position().y();
        double wx = (mx - pan_x_) / zoom_;
        double wy = (my - pan_y_) / zoom_;
        zoom_ = std::clamp(zoom_ * factor, 0.1, 10.0);
        pan_x_ = mx - wx * zoom_;
        pan_y_ = my - wy * zoom_;
        update();
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::MiddleButton)
        {
            dragging_ = true;
            drag_start_ = event->pos();
            drag_origin_pan_x_ = pan_x_;
            drag_origin_pan_y_ = pan_y_;
            return;
        }
        if (event->button() == Qt::LeftButton)
            toggle_cell_at(event->pos());
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (dragging_ && (event->buttons() & Qt::MiddleButton))
        {
            QPoint delta = event->pos() - drag_start_;
            pan_x_ = drag_origin_pan_x_ + delta.x();
            pan_y_ = drag_origin_pan_y_ + delta.y();
            update();
            return;
        }
        if (event->buttons() & Qt::LeftButton)
            set_cell_at(event->pos(), CellType::WALL);
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::MiddleButton)
            dragging_ = false;
    }

signals:
    void key_forwarded(Qt::Key key);

private:
    PathSyncApp &app_;
    Grid grid_;
    std::map<Qt::Key, std::function<void()>> key_bindings_;
    Qt::Key last_key_ = Qt::Key_unknown;

    std::function<void()> solve_callback_;

    double zoom_ = 1.0;
    double pan_x_ = 0.0;
    double pan_y_ = 0.0;
    bool dragging_ = false;
    QPoint drag_start_;
    double drag_origin_pan_x_ = 0.0;
    double drag_origin_pan_y_ = 0.0;

    void handle_key(Qt::Key key, bool shifted)
    {
        if (shifted)
        {
            switch (key)
            {
            case Qt::Key_H: key_bindings_[Qt::Key_H](); return;
            case Qt::Key_M: app_.request_next_map(); return;
            case Qt::Key_P: app_.clear_paths();        return;
            case Qt::Key_R: app_.reset_grid();         return;
            default: break;
            }
            return;
        }

        auto it = key_bindings_.find(key);
        if (it != key_bindings_.end())
            it->second();
    }

    Coordinate cell_at(const QPoint &pos) const
    {
        auto map_data = app_.get_current_map_data();
        if (!map_data)
            return {0, 0};
        double cell = static_cast<double>(grid_.get_cell_size()) * zoom_;
        int x = static_cast<int>((pos.x() - pan_x_) / cell);
        int y = static_cast<int>((pos.y() - pan_y_) / cell);
        return {x, y};
    }

    bool inside_grid(const QPoint &pos) const
    {
        auto map_data = app_.get_current_map_data();
        if (!map_data)
            return false;
        double cell = static_cast<double>(grid_.get_cell_size()) * zoom_;
        Coordinate c = cell_at(pos);
        return c.first >= 0 && c.first < grid_.get_width() &&
               c.second >= 0 && c.second < grid_.get_height() &&
               pos.x() >= pan_x_ && pos.y() >= pan_y_ &&
               pos.x() < pan_x_ + grid_.get_width() * cell &&
               pos.y() < pan_y_ + grid_.get_height() * cell;
    }

    void toggle_cell_at(const QPoint &pos)
    {
        if (!inside_grid(pos))
            return;
        auto md = app_.get_current_map_data();
        if (!md)
            return;
        Coordinate c = cell_at(pos);
        CellType cur = md->get_cell_type(c);
        md->set_cell_type(c, cur == CellType::WALL ? CellType::DEFAULT : CellType::WALL);
        update();
    }

    void set_cell_at(const QPoint &pos, CellType type)
    {
        if (!inside_grid(pos))
            return;
        auto md = app_.get_current_map_data();
        if (!md)
            return;
        md->set_cell_type(cell_at(pos), type);
        update();
    }
};

// ── VisualizationSystem: container ───────────────────────────────────────

VisualizationSystem::~VisualizationSystem() = default;

VisualizationSystem::VisualizationSystem(PathSyncApp& app, QWidget *parent)
    : QWidget(parent)
    , app_(app)
    , grid_widget_(nullptr)
    , solver_combo_(nullptr)
    , status_label_(nullptr)
    , solve_status_label_(nullptr)
    , sidebar_(nullptr)
    , perf_text_(nullptr)
    , scene_timer_(new QTimer(this))
{
    setWindowTitle("Path Sync");
    resize(1200, 900);

    setup_ui();
    populate_solver_combo();
    update_status();

    help_stream_ << "Available Key Bindings:"
                 << "\nC    - Cycle solver"
                 << "\nA    - Toggle agent mode"
                 << "\nSpace - Solve current scene"
                 << "\nShift+H - Help overlay"
                 << "\nShift+M - Next map"
                 << "\nShift+P - Clear paths"
                 << "\nShift+R - Reset grid"
                 << "\n[    - Previous scene"
                 << "\n]    - Next scene"
                 << "\nCtrl+Wheel - Zoom"
                 << "\nMClick+Drag - Pan"
                 << "\nMouse click - Toggle wall";

    Logger::get().info(help_stream_.str().c_str());
}

void VisualizationSystem::setup_ui()
{
    auto *root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // ── fixed-size viewport, centred ───────────────────────────────────
    auto *vp_container = new QFrame;
    vp_container->setFrameShape(QFrame::NoFrame);
    vp_container->setStyleSheet("QFrame { background: #303030; }");
    auto *vp_lay = new QHBoxLayout(vp_container);
    vp_lay->setContentsMargins(0, 0, 0, 0);

    grid_widget_ = new GridWidget(app_, this);
    grid_widget_->setFixedSize(1400, 720);
    grid_widget_->set_solve_callback([this]() { solve_async(); });

    auto *vp_center = new QHBoxLayout;
    vp_center->addStretch();
    vp_center->addWidget(grid_widget_);
    vp_center->addStretch();
    vp_lay->addLayout(vp_center);

    // ── performance sidebar ────────────────────────────────────────────
    sidebar_ = new QWidget;
    sidebar_->setFixedWidth(320);
    sidebar_->setStyleSheet("QWidget { background: #252525; }");
    auto *sb_side_lay = new QVBoxLayout(sidebar_);
    sb_side_lay->setContentsMargins(6, 6, 6, 6);

    auto *perf_title = new QLabel("Performance data");
    perf_title->setStyleSheet("QLabel { color: #0a0; font-weight: bold;"
                              "  font-size: 13px; padding-bottom: 4px; }");
    sb_side_lay->addWidget(perf_title);

    perf_text_ = new QPlainTextEdit;
    perf_text_->setReadOnly(true);
    perf_text_->setPlainText("No data available right now.");
    perf_text_->setStyleSheet("QPlainTextEdit { background: #1e1e1e; color: #ccc;"
                              "  border: 1px solid #444; padding: 6px;"
                              "  font-family: monospace; font-size: 12px; }");
    sb_side_lay->addWidget(perf_text_, 1);

    vp_lay->addWidget(sidebar_);

    root->addWidget(vp_container, 1);

    connect(grid_widget_, &GridWidget::key_forwarded, this, [this](Qt::Key key) {
        int raw = static_cast<int>(key);
        bool shifted = raw & 0x1000;
        Qt::Key actual = shifted ? static_cast<Qt::Key>(raw & ~0x1000) : key;

        switch (actual)
        {
        case Qt::Key_C:
            app_.change_solver();
            {
                QString name = QString::fromStdString(
                    std::string(app_.get_current_solver_name()));
                int idx = solver_combo_->findText(name);
                if (idx >= 0)
                    solver_combo_->setCurrentIndex(idx);
            }
            break;
        case Qt::Key_A:
            app_.toggle_agent_mode();
            populate_solver_combo();
            break;
        case Qt::Key_M:
            app_.request_next_map();
            grid_widget_->reset_view();
            break;
        case Qt::Key_BracketLeft:
            app_.request_previous_scene();
            app_.clear_paths();
            grid_widget_->center_view();
            break;
        case Qt::Key_BracketRight:
            app_.request_next_scene();
            app_.clear_paths();
            grid_widget_->center_view();
            break;
        default: break;
        }

        if (shifted)
        {
            if (actual == Qt::Key_M) { app_.request_previous_map(); grid_widget_->reset_view(); }
            if (actual == Qt::Key_P) app_.clear_paths();
            if (actual == Qt::Key_R) app_.reset_grid();
        }

        update_status();
        grid_widget_->update();
    });

    // ── toolbar (at bottom, above status bar) ──────────────────────────
    auto *tb = new QWidget;
    tb->setStyleSheet("QWidget { background: #2b2b2b; }"
                      "QPushButton { background: #3c3c3c; color: #eee; border: 1px solid #555;"
                      "  padding: 4px 12px; border-radius: 3px; min-height: 24px; }"
                      "QPushButton:hover { background: #4a4a4a; }"
                      "QLabel { color: #ccc; }"
                      "QComboBox { background: #3c3c3c; color: #eee; border: 1px solid #555;"
                      "  padding: 3px 6px; border-radius: 3px; min-height: 24px; }");

    auto *tb_lay = new QHBoxLayout(tb);
    tb_lay->setContentsMargins(8, 4, 8, 4);
    tb_lay->setSpacing(6);

    solve_btn_ = new QPushButton("Solve");
    auto *solve_btn = solve_btn_;
    cancel_btn_ = new QPushButton("Cancel");
    auto *cancel_btn = cancel_btn_;
    cancel_btn->setEnabled(false);
    auto *clear_btn = new QPushButton("Clear");
    auto *reset_btn = new QPushButton("Reset");
    prev_btn_ = new QPushButton("◀ Scene");
    next_btn_ = new QPushButton("Scene ▶");
    auto *prev_btn  = prev_btn_;
    auto *next_btn  = next_btn_;
    auto *prev_map_btn = new QPushButton("◀ Map");
    auto *map_btn      = new QPushButton("Map ▶");
    auto *agent_btn    = new QPushButton("Agent");

    solver_combo_ = new QComboBox;
    solver_combo_->setMinimumWidth(160);

    connect(solve_btn, &QPushButton::clicked, this, &VisualizationSystem::on_solve_clicked);
    connect(cancel_btn, &QPushButton::clicked, this, [this]() {
        app_.cancel_solve();
        cancel_btn_->setEnabled(false);
        cancel_btn_->setText("Cancelling...");
    });
    connect(clear_btn, &QPushButton::clicked, this, &VisualizationSystem::on_clear_clicked);
    connect(reset_btn, &QPushButton::clicked, this, &VisualizationSystem::on_reset_clicked);
    connect(prev_btn,  &QPushButton::pressed, this, [this]() {
        scene_dir_forward_ = false;
        scene_timer_interval_ = 350.0;
        scene_timer_->setInterval(static_cast<int>(scene_timer_interval_));
        if (on_prev_scene()) scene_timer_->start();
    });
    connect(next_btn,  &QPushButton::pressed, this, [this]() {
        scene_dir_forward_ = true;
        scene_timer_interval_ = 350.0;
        scene_timer_->setInterval(static_cast<int>(scene_timer_interval_));
        if (on_next_scene()) scene_timer_->start();
    });
    connect(prev_btn,  &QPushButton::released, this, [this]() { scene_timer_->stop(); });
    connect(next_btn,  &QPushButton::released, this, [this]() { scene_timer_->stop(); });
    connect(scene_timer_, &QTimer::timeout, this, [this]() {
        scene_timer_interval_ = std::max(scene_timer_interval_ * 0.85, 70.0);
        scene_timer_->setInterval(static_cast<int>(scene_timer_interval_));
        bool ok = scene_dir_forward_ ? on_next_scene() : on_prev_scene();
        if (!ok) scene_timer_->stop();
    });
    connect(prev_map_btn, &QPushButton::clicked, this, &VisualizationSystem::on_prev_map);
    connect(map_btn,      &QPushButton::clicked, this, &VisualizationSystem::on_next_map);
    connect(agent_btn, &QPushButton::clicked, this, &VisualizationSystem::on_toggle_agent_mode);
    connect(solver_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VisualizationSystem::on_solver_combo_changed);

    tb_lay->addWidget(solve_btn);
    tb_lay->addWidget(cancel_btn);
    tb_lay->addWidget(clear_btn);
    tb_lay->addWidget(reset_btn);
    tb_lay->addWidget(prev_btn);
    tb_lay->addWidget(next_btn);
    tb_lay->addWidget(prev_map_btn);
    tb_lay->addWidget(map_btn);
    tb_lay->addWidget(agent_btn);
    tb_lay->addSpacing(12);
    tb_lay->addWidget(new QLabel("Solver:"));
    tb_lay->addWidget(solver_combo_);
    tb_lay->addStretch();

    root->addWidget(tb);

    // ── status bar ──────────────────────────────────────────────────────
    auto *status_bar = new QWidget;
    status_bar->setStyleSheet("background: #1e1e1e;");
    status_bar->setFixedHeight(24);
    auto *sb_lay = new QHBoxLayout(status_bar);
    sb_lay->setContentsMargins(8, 0, 8, 0);

    status_label_ = new QLabel;
    status_label_->setStyleSheet("QLabel { color: #aaa; font-family: monospace; }");
    sb_lay->addWidget(status_label_);

    sb_lay->addStretch();

    solve_status_label_ = new QLabel;
    solve_status_label_->setStyleSheet("QLabel { color: #aaa; font-family: monospace; }");
    sb_lay->addWidget(solve_status_label_);

    root->addWidget(status_bar);

    auto *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, grid_widget_, [this]() { grid_widget_->update(); });
    timer->start(16);
}

void VisualizationSystem::populate_solver_combo()
{
    solver_combo_->blockSignals(true);
    solver_combo_->clear();
    bool multi = app_.get_is_multi_agent();
    for (const auto &name : app_.get_solver_names(multi))
        solver_combo_->addItem(QString::fromStdString(name));
    solver_combo_->blockSignals(false);
    if (solver_combo_->count() > 0)
        solver_combo_->setCurrentIndex(0);
}

void VisualizationSystem::on_solver_combo_changed(int index)
{
    if (index < 0)
        return;
    app_.select_solver_by_index(static_cast<std::size_t>(index), app_.get_is_multi_agent());
    grid_widget_->update();
    update_status();
}

void VisualizationSystem::solve_async()
{
    if (solving_)
        return;

    app_.reset_cancel();
    solving_ = true;
    solve_btn_->setEnabled(false);
    cancel_btn_->setEnabled(true);
    cancel_btn_->setText("Cancel");
    solve_btn_->setText("Solving...");
    solve_status_label_->setText("Solving...");

    auto starts = app_.get_current_scene().first;
    auto ends   = app_.get_current_scene().second;

    auto *watcher = new QFutureWatcher<std::shared_ptr<MapData>>(this);
    connect(watcher, &QFutureWatcher<std::shared_ptr<MapData>>::finished, this, [this, watcher]() {
        auto result = watcher->result();
        if (result) {
            app_.set_map_data(std::move(result));
            grid_widget_->sync_and_update();
            solve_status_label_->setText("Solved");
            update_perf_sidebar();
        } else {
            solve_status_label_->setText("No path found");
        }
        QTimer::singleShot(3000, this, [this]() { solve_status_label_->clear(); });
        solving_ = false;
        solve_btn_->setEnabled(true);
        solve_btn_->setText("Solve");
        cancel_btn_->setEnabled(false);
        cancel_btn_->setText("Cancel");
        watcher->deleteLater();
    });

    watcher->setFuture(QtConcurrent::run([this, starts, ends]() {
        return app_.solve_async_on_copy(starts, ends);
    }));
}

void VisualizationSystem::on_solve_clicked()      { solve_async();                           }
void VisualizationSystem::on_clear_clicked()      { app_.clear_paths();                      }
void VisualizationSystem::on_reset_clicked()      { app_.reset_grid();                       }
bool VisualizationSystem::on_prev_scene()
{
    if (!app_.request_previous_scene()) return false;
    app_.clear_paths();
    grid_widget_->center_view();
    update_status();
    return true;
}
bool VisualizationSystem::on_next_scene()
{
    if (!app_.request_next_scene()) return false;
    app_.clear_paths();
    grid_widget_->center_view();
    update_status();
    return true;
}
void VisualizationSystem::on_prev_map()           { app_.request_previous_map();  grid_widget_->reset_view();  update_status(); }
void VisualizationSystem::on_next_map()           { app_.request_next_map();       grid_widget_->reset_view();  update_status(); }
void VisualizationSystem::on_toggle_agent_mode()  { app_.toggle_agent_mode(); populate_solver_combo(); update_status(); }

void VisualizationSystem::update_status()
{
    auto map_data = app_.get_current_map_data();
    if (!map_data)
        return;
    std::string text;
    text += "Agent: ";
    text += std::to_string(app_.get_num_agents());
    text += "  │  Solver: ";
    text += app_.get_current_solver_name();
    text += "  │  Map: ";
    text += app_.get_current_map_name();
    text += "  │  Scene: ";
    text += std::to_string(app_.get_scene_index() + 1);
    text += " / ";
    text += std::to_string(app_.get_total_scenes());
    status_label_->setText(QString::fromStdString(text));
    grid_widget_->update();
}

void VisualizationSystem::focus_grid()
{
    grid_widget_->setFocus();
}

void VisualizationSystem::update_perf_sidebar()
{
    auto pm = app_.get_performance_metrics();

    std::string text;
    text += "Solver:  " + pm.solver_name + "\n";
    text += "Map:     " + pm.map_name + "\n";
    text += "Status:  " + std::string(pm.success ? "OK" : "FAIL") + "\n";
    text += "Runtime: " + std::to_string(pm.runtime.count()) + " us\n";

    auto sep = [&]() { text += "──────────────────────────\n"; };

    sep();
    text += "Search Effort\n";
    text += "  Explored:  " + std::to_string(pm.num_of_nodes_explored) + "\n";
    text += "  Expanded:  " + std::to_string(pm.num_of_nodes_expanded) + "\n";
    text += "  Reopened:  " + std::to_string(pm.num_of_nodes_reopened) + "\n";
    text += "  Peak Open: " + std::to_string(pm.peak_open_size) + "\n";

    sep();
    text += "Path Quality\n";
    text += "  Length:    " + std::to_string(pm.path_length) + "\n";
    if (pm.optimal_path_length > 0)
        text += "  Optimal:   " + std::to_string(pm.optimal_path_length) + "\n";

    if (pm.sum_of_costs > 0)
    {
        sep();
        text += "Multi-Agent\n";
        text += "  Sum of Costs: " + std::to_string(pm.sum_of_costs) + "\n";
        text += "  Makespan:     " + std::to_string(pm.makespan) + "\n";
    }

    perf_text_->setPlainText(QString::fromStdString(text));
}

#include "visualization_system.moc"
} // namespace path_sync
