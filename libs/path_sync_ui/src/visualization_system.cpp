#include "path_sync_ui/visualization_system.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QPainter>
#include <QFont>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QStackedWidget>
#include <algorithm>
#include <filesystem>
#include <fstream>

#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>

#include "path_sync_core/logger.hpp"
#include "path_sync_ui/cost_map_viewer.hpp"

namespace path_sync
{

// ── GridWidget: fixed-size viewport with pan + zoom ──────────────────────

class GridWidget : public QWidget
{
    Q_OBJECT

signals:
    void points_changed();

public:
    GridWidget(PathSyncApp &app, QWidget *parent = nullptr)
        : QWidget(parent), app_(app), grid_()
    {
        setFocusPolicy(Qt::StrongFocus);
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

        // ── multi-agent path overlay ──────────────────────────────────
        static const QColor kAgentColors[] = {
            QColor(230,  25,  75, 180),  // red
            QColor( 60, 180,  75, 180),  // green
            QColor( 67,  99, 216, 180),  // blue
            QColor(245, 130,  49, 180),  // orange
            QColor(145,  30, 180, 180),  // purple
            QColor( 66, 212, 244, 180),  // cyan
            QColor(240,  50, 230, 180),  // magenta
            QColor(191, 239,  69, 180),  // lime
            QColor(250, 190, 212, 180),  // pink
            QColor( 70, 153, 144, 180),  // teal
        };

        const auto &ma = app_.get_current_ma_solution();
        if (!ma.empty())
        {
            double inset = cell * 0.12;
            for (std::size_t ai = 0; ai < ma.size(); ++ai)
            {
                const auto &color = kAgentColors[ai % 10];
                for (const auto &pt : ma[ai])
                {
                    double px = pan_x_ + static_cast<int>(pt.first) * cell + inset;
                    double py = pan_y_ + static_cast<int>(pt.second) * cell + inset;
                    double sz = cell - 2.0 * inset;
                    p.fillRect(QRectF(px, py, sz, sz), color);
                }
            }
        }

        // viewport border
        p.setPen(QPen(QColor(102, 102, 102), 2));
        p.drawRect(rect().adjusted(1, 1, -1, -1));
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        QWidget::keyPressEvent(event);
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
        {
            toggle_cell_at(event->pos());
            return;
        }
        if (event->button() == Qt::RightButton)
        {
            if (!inside_grid(event->pos()))
                return;
            Coordinate c = cell_at(event->pos());
            if (event->modifiers() & Qt::ShiftModifier)
                app_.set_goal_point(c);
            else
                app_.set_start_point(c);
            emit points_changed();
            update();
        }
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

private:
    PathSyncApp &app_;
    Grid grid_;

    double zoom_ = 1.0;
    double pan_x_ = 0.0;
    double pan_y_ = 0.0;
    bool dragging_ = false;
    QPoint drag_start_;
    double drag_origin_pan_x_ = 0.0;
    double drag_origin_pan_y_ = 0.0;

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

void VisualizationSystem::closeEvent(QCloseEvent *event)
{
    app_.save_custom_map();
    QWidget::closeEvent(event);
}

VisualizationSystem::VisualizationSystem(PathSyncApp& app, QWidget *parent)
    : QWidget(parent)
    , app_(app)
    , grid_widget_(nullptr)
    , solver_combo_(nullptr)
    , status_label_(nullptr)
    , solve_status_label_(nullptr)
    , sidebar_(nullptr)
    , sidebar_stack_(nullptr)
    , perf_text_(nullptr)
    , pareto_panel_(nullptr)
{
    setWindowTitle("Path Sync");
    resize(1200, 900);

    setup_ui();
    populate_solver_combo();
    update_status();
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
    connect(grid_widget_, &GridWidget::points_changed,
            this, &VisualizationSystem::update_status);

    auto *vp_center = new QHBoxLayout;
    vp_center->addStretch();
    vp_center->addWidget(grid_widget_);
    vp_center->addStretch();
    vp_lay->addLayout(vp_center);

    // ── sidebar with stacked widget (perf / pareto) ──────────────────
    sidebar_ = new QWidget;
    sidebar_->setFixedWidth(320);
    sidebar_->setStyleSheet("QWidget { background: #252525; }");
    auto *sb_side_lay = new QVBoxLayout(sidebar_);
    sb_side_lay->setContentsMargins(6, 6, 6, 6);

    sidebar_stack_ = new QStackedWidget;
    sb_side_lay->addWidget(sidebar_stack_);

    // page 0: performance
    auto *perf_page = new QWidget;
    auto *perf_lay = new QVBoxLayout(perf_page);
    perf_lay->setContentsMargins(0, 0, 0, 0);

    auto *perf_title = new QLabel("Performance data");
    perf_title->setStyleSheet("QLabel { color: #0a0; font-weight: bold;"
                              "  font-size: 13px; padding-bottom: 4px; }");
    perf_lay->addWidget(perf_title);

    perf_text_ = new QPlainTextEdit;
    perf_text_->setReadOnly(true);
    perf_text_->setPlainText("No data available right now.");
    perf_text_->setStyleSheet("QPlainTextEdit { background: #1e1e1e; color: #ccc;"
                              "  border: 1px solid #444; padding: 6px;"
                              "  font-family: monospace; font-size: 12px; }");
    perf_lay->addWidget(perf_text_, 1);

    sidebar_stack_->addWidget(perf_page);

    // page 1: Pareto front (MO)
    pareto_panel_ = new ParetoFrontPanel;
    pareto_panel_->setStyleSheet(
        "ParetoFrontPanel { background: #1e1e1e; }"
        "QLabel       { color: #ddd; }"
        "QGroupBox    { color: #ddd; border: 1px solid #444; border-radius: 4px;"
        "               margin-top: 10px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
        "QPushButton  { background: #3a5a8c; color: #fff; border: 1px solid #4a7abc;"
        "               padding: 5px 14px; border-radius: 3px; min-height: 24px; }"
        "QPushButton:hover { background: #4a7abc; }"
        "QSpinBox     { background: #2d2d2d; color: #ddd; border: 1px solid #555;"
        "               padding: 3px; border-radius: 3px; min-height: 22px; }"
        "QSpinBox::up-button, QSpinBox::down-button {"
        "  background: #3a3a3a; border: 1px solid #555;"
        "  border-radius: 2px; margin: 1px; width: 18px; }"
        "QSlider::groove:horizontal { border: 1px solid #555; height: 6px;"
        "  background: #2d2d2d; border-radius: 3px; }"
        "QSlider::handle:horizontal { background: #4a7abc; border: 1px solid #5a8acc;"
        "  width: 14px; margin: -4px 0; border-radius: 7px; }"
        "QSlider::sub-page:horizontal { background: #4a7abc; border-radius: 3px; }"
        "QHeaderView::section { background: #2d2d2d; color: #ddd;"
        "  border: 1px solid #444; padding: 3px; }"
        "QTableWidget { background: #1e1e1e; color: #ddd;"
        "  border: 1px solid #444; gridline-color: #333; }"
        "QTableWidget::item:selected { background: #3a5a8c; color: #fff; }");
    sidebar_stack_->addWidget(pareto_panel_);

    sidebar_stack_->setCurrentIndex(0);

    connect(pareto_panel_, &ParetoFrontPanel::solve_requested,
            this, &VisualizationSystem::on_mo_solve_requested);
    connect(pareto_panel_, &ParetoFrontPanel::solution_selected,
            this, &VisualizationSystem::on_mo_solution_selected);

    vp_lay->addWidget(sidebar_);

    root->addWidget(vp_container, 1);

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

    auto *scene_label = new QLabel("Current Scene: ");
    int max_block = (app_.get_total_scenes() + app_.get_num_agents() - 1) / app_.get_num_agents();
    scene_spin_ = new QSpinBox;
    scene_spin_->setMinimum(1);
    scene_spin_->setMaximum(max_block);
    scene_spin_->setMaximumWidth(80);
    scene_spin_->setAlignment(Qt::AlignCenter);
    scene_spin_->setSuffix(" / " + QString::number(max_block));
    scene_spin_->setStyleSheet(
        "QSpinBox { background: #3c3c3c; color: #eee; border: 1px solid #555;"
        "  padding: 4px 4px; border-radius: 3px; min-height: 24px; }"
        "QSpinBox::up-button, QSpinBox::down-button {"
        "  background: #4a4a4a; border: 1px solid #555;"
        "  border-radius: 2px; margin: 1px; }");

    QObject::connect(scene_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
                     this, [this](int value) {
                         int idx = (value - 1) * app_.get_num_agents();
                         if (!app_.request_scene(idx))
                         {
                             update_status();
                             return;
                         }
                         app_.clear_paths();
                         grid_widget_->center_view();
                         reset_perf_buffer();
                         update_status();
                     });

    auto *map_label = new QLabel("  Map: ");
    int total_maps = app_.get_total_maps();
    map_spin_ = new QSpinBox;
    map_spin_->setMinimum(1);
    map_spin_->setMaximum(total_maps);
    map_spin_->setMaximumWidth(80);
    map_spin_->setAlignment(Qt::AlignCenter);
    map_spin_->setSuffix(" / " + QString::number(total_maps));
    map_spin_->setStyleSheet(
        "QSpinBox { background: #3c3c3c; color: #eee; border: 1px solid #555;"
        "  padding: 4px 4px; border-radius: 3px; min-height: 24px; }"
        "QSpinBox::up-button, QSpinBox::down-button {"
        "  background: #4a4a4a; border: 1px solid #555;"
        "  border-radius: 2px; margin: 1px; }");

    QObject::connect(map_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
                     this, [this](int value) {
                         if (!app_.request_map(value - 1))
                         {
                             update_status();
                             return;
                         }
                         app_.clear_paths();
                         grid_widget_->center_view();
                         reset_perf_buffer();
                         update_status();
                     });

    auto *agent_label = new QLabel("  Agents:");
    auto *agent_spin = new QSpinBox;
    agent_spin->setMinimum(1);
    agent_spin->setMaximum(std::min(10, app_.get_total_scenes()));
    agent_spin->setValue(app_.get_num_agents());
    agent_spin->setMaximumWidth(60);
    agent_spin->setAlignment(Qt::AlignCenter);
    agent_spin->setStyleSheet(
        "QSpinBox { background: #3c3c3c; color: #eee; border: 1px solid #555;"
        "  padding: 4px 4px; border-radius: 3px; min-height: 24px; }"
        "QSpinBox::up-button, QSpinBox::down-button {"
        "  background: #4a4a4a; border: 1px solid #555;"
        "  border-radius: 2px; margin: 1px; }");

    QObject::connect(agent_spin, QOverload<int>::of(&QSpinBox::valueChanged),
                     this, [this, agent_spin](int value) {
                         app_.set_num_agents(value);
                         app_.clear_paths();
                         grid_widget_->center_view();
                         reset_perf_buffer();
                         populate_solver_combo();

                         int max_block = (app_.get_total_scenes() + value - 1) / value;
                         scene_spin_->blockSignals(true);
                         scene_spin_->setMaximum(max_block);
                         scene_spin_->setSuffix(" / " + QString::number(max_block));
                         scene_spin_->setValue(1);
                         scene_spin_->blockSignals(false);

                         update_status();
                     });

    auto *timeout_label = new QLabel("  Time (s):");
    timeout_spin_ = new QSpinBox;
    timeout_spin_->setMinimum(5);
    timeout_spin_->setMaximum(300);
    timeout_spin_->setValue(5);
    timeout_spin_->setSingleStep(5);
    timeout_spin_->setMaximumWidth(70);
    timeout_spin_->setAlignment(Qt::AlignCenter);
    timeout_spin_->setStyleSheet(
        "QSpinBox { background: #3c3c3c; color: #eee; border: 1px solid #555;"
        "  padding: 4px 4px; border-radius: 3px; min-height: 24px; }"
        "QSpinBox::up-button, QSpinBox::down-button {"
        "  background: #4a4a4a; border: 1px solid #555;"
        "  border-radius: 2px; margin: 1px; }");

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
    connect(solver_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VisualizationSystem::on_solver_combo_changed);

    tb_lay->addWidget(solve_btn);
    tb_lay->addWidget(cancel_btn);
    tb_lay->addWidget(clear_btn);
    tb_lay->addWidget(reset_btn);

    auto *new_map_btn = new QPushButton("New Map");
    connect(new_map_btn, &QPushButton::clicked, this, [this]() {
        bool ok = false;
        QString size = QInputDialog::getText(this, "New Map",
            "Dimensions (WxH, e.g. 200x200):",
            QLineEdit::Normal, "200x200", &ok);
        if (!ok || size.isEmpty()) return;
        auto parts = size.split('x');
        if (parts.size() != 2) return;
        int w = parts[0].toInt();
        int h = parts[1].toInt();
        if (w < 10 || h < 10 || w > 1000 || h > 1000) return;
        app_.create_blank_map(w, h);
        grid_widget_->sync_and_update();
        populate_solver_combo();
        update_status();
    });
    tb_lay->addWidget(new_map_btn);

    auto *cost_map_btn = new QPushButton("Cost Map");
    connect(cost_map_btn, &QPushButton::clicked, this, [this]() {
        auto cm = app_.load_cost_map();
        if (!cm)
        {
            solve_status_label_->setText("Failed to load cost map");
            QTimer::singleShot(3000, this, [this]() { solve_status_label_->clear(); });
            return;
        }
        int n_obj = mo_mode_ ? mo_num_objectives_ : cm->objectives;
        CostMapViewer viewer(*cm, n_obj, this);
        viewer.exec();
    });
    tb_lay->addWidget(cost_map_btn);

    tb_lay->addWidget(scene_label);
    tb_lay->addWidget(scene_spin_);
    tb_lay->addWidget(map_label);
    tb_lay->addWidget(map_spin_);
    tb_lay->addWidget(agent_label);
    tb_lay->addWidget(agent_spin);
    tb_lay->addWidget(timeout_label);
    tb_lay->addWidget(timeout_spin_);
    tb_lay->addSpacing(12);
    tb_lay->addWidget(new QLabel("Solver:"));
    tb_lay->addWidget(solver_combo_);
    tb_lay->addStretch();
    timeout_remaining_label_ = new QLabel;
    timeout_remaining_label_->setStyleSheet("color: #ffa500; font-weight: bold; padding-right: 8px;");
    timeout_remaining_label_->hide();
    tb_lay->addWidget(timeout_remaining_label_);

    timeout_timer_ = new QTimer(this);
    timeout_timer_->setInterval(200);
    connect(timeout_timer_, &QTimer::timeout, this, [this]() {
        if (!solving_) {
            timeout_timer_->stop();
            timeout_remaining_label_->hide();
            return;
        }
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            solve_deadline_ - std::chrono::steady_clock::now()).count();
        if (remaining <= 0)
            timeout_remaining_label_->setText("Timeout: 0.0s");
        else
            timeout_remaining_label_->setText(
                QString("Timeout: %1s").arg(remaining / 1000.0, 0, 'f', 1));
    });

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
    combo_sections_.clear();

    auto add_header = [&](const QString &text) {
        solver_combo_->addItem(text);
        int idx = solver_combo_->count() - 1;
        solver_combo_->setItemData(idx, QColor(140, 200, 140), Qt::ForegroundRole);
        solver_combo_->setItemData(idx, QFont("sans", 10, QFont::Bold), Qt::FontRole);
        // Disable so it cannot be selected
        auto *model = qobject_cast<QStandardItemModel *>(solver_combo_->model());
        if (model)
            model->item(idx)->setEnabled(false);
    };

    // ── Single-Agent ──
    const auto &sa_names = app_.get_solver_names(false);
    if (!sa_names.empty())
    {
        add_header("── Single-Agent ──");
        int first = solver_combo_->count();
        for (std::size_t i = 0; i < sa_names.size(); i++)
        {
            QString display = QString::fromStdString(sa_names[i])
                + (app_.is_sa_solver_optimal(i) ? "  [optimal]" : "  [suboptimal]");
            solver_combo_->addItem(display, QString::fromStdString(sa_names[i]));
        }
        combo_sections_.push_back({first, static_cast<int>(sa_names.size()), SolverCategory::SA});
        solver_combo_->insertSeparator(solver_combo_->count());
    }

    // ── Multi-Agent ──
    const auto &ma_names = app_.get_solver_names(true);
    if (!ma_names.empty())
    {
        add_header("── Multi-Agent ──");
        int first = solver_combo_->count();
        for (std::size_t i = 0; i < ma_names.size(); i++)
        {
            QString display = QString::fromStdString(ma_names[i])
                + (app_.is_ma_solver_optimal(i) ? "  [optimal]" : "  [suboptimal]");
            solver_combo_->addItem(display, QString::fromStdString(ma_names[i]));
        }
        combo_sections_.push_back({first, static_cast<int>(ma_names.size()), SolverCategory::MA});
        solver_combo_->insertSeparator(solver_combo_->count());
    }

    // ── Multi-Objective ──
    const auto &mo_names = app_.get_mo_solver_names();
    if (!mo_names.empty())
    {
        add_header("── Multi-Objective ──");
        int first = solver_combo_->count();
        for (std::size_t i = 0; i < mo_names.size(); i++)
        {
            QString display = QString::fromStdString(mo_names[i])
                + (app_.is_mo_solver_optimal(i) ? "  [optimal]" : "  [suboptimal]");
            solver_combo_->addItem(display, QString::fromStdString(mo_names[i]));
        }
        combo_sections_.push_back({first, static_cast<int>(mo_names.size()), SolverCategory::MO});
        solver_combo_->insertSeparator(solver_combo_->count());
    }

    solver_combo_->blockSignals(false);
    if (!combo_sections_.empty())
        solver_combo_->setCurrentIndex(combo_sections_[0].first_index);
}

std::optional<std::size_t> VisualizationSystem::combo_to_solver_index(
    int combo_index, SolverCategory &cat) const
{
    for (const auto &sec : combo_sections_)
    {
        if (combo_index >= sec.first_index && combo_index < sec.first_index + sec.count)
        {
            cat = sec.category;
            return static_cast<std::size_t>(combo_index - sec.first_index);
        }
    }
    return std::nullopt;
}

void VisualizationSystem::on_solver_combo_changed(int index)
{
    if (index < 0)
        return;

    SolverCategory cat;
    auto opt_idx = combo_to_solver_index(index, cat);
    if (!opt_idx.has_value())
        return;

    switch (cat)
    {
    case SolverCategory::SA:
        app_.select_sa_solver_by_index(*opt_idx);
        exit_mo_mode();
        break;
    case SolverCategory::MA:
        app_.select_ma_solver_by_index(*opt_idx);
        exit_mo_mode();
        break;
    case SolverCategory::MO:
        app_.select_mo_solver_by_index(*opt_idx);
        enter_mo_mode();
        break;
    }

    grid_widget_->update();
    update_status();
}

// ── CSV logging ─────────────────────────────────────────────────────

namespace {

void write_csv_log(const PerformanceMetrics &pm, const std::optional<MAPFMetrics> &ma_met)
{
    std::string dir = std::string(PROJECT_ROOT) + "/log";
    std::filesystem::create_directories(dir);

    if (pm.num_agents > 1 && ma_met.has_value())
    {
        std::string path = dir + "/results_ma.csv";
        bool exists = std::filesystem::exists(path);
        std::ofstream ofs(path, std::ios::app);
        if (!ofs) return;
        if (!exists)
            ofs << MAPFMetrics::csv_header() << "\n";
        ofs << ma_met->csv_line(pm) << "\n";
    }
    else
    {
        std::string path = dir + "/results_sa.csv";
        bool exists = std::filesystem::exists(path);
        std::ofstream ofs(path, std::ios::app);
        if (!ofs) return;
        if (!exists)
            ofs << PerformanceMetrics::csv_header() << "\n";
        ofs << pm.csv_line() << "\n";
    }
}

} // anonymous namespace

void VisualizationSystem::enter_mo_mode()
{
    if (mo_mode_) return;
    mo_mode_ = true;
    sidebar_stack_->setCurrentIndex(1);
}

void VisualizationSystem::exit_mo_mode()
{
    if (!mo_mode_) return;
    mo_mode_ = false;
    sidebar_stack_->setCurrentIndex(0);
}

void VisualizationSystem::solve_mo_async(int num_objectives)
{
    if (solving_) return;

    app_.reset_cancel();
    solving_ = true;
    solve_btn_->setEnabled(false);
    cancel_btn_->setEnabled(true);
    cancel_btn_->setText("Cancel");
    solve_btn_->setText("MO Solving...");
    solve_status_label_->setText("MO Solving...");

    auto starts = app_.get_current_scene().first;
    auto ends   = app_.get_current_scene().second;
    if (starts.empty() || ends.empty())
    {
        solve_status_label_->setText("No start/goal set");
        solving_ = false;
        solve_btn_->setEnabled(true);
        solve_btn_->setText("Solve");
        cancel_btn_->setEnabled(false);
        return;
    }

    app_.set_timeout_ms(timeout_spin_->value() * 1000);
    solve_deadline_ = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_spin_->value() * 1000);
    timeout_remaining_label_->setText(
        QString("Timeout: %1s").arg(timeout_spin_->value()));
    timeout_remaining_label_->show();
    timeout_timer_->start();

    auto *watcher = new QFutureWatcher<std::shared_ptr<MapData>>(this);
    connect(watcher, &QFutureWatcher<std::shared_ptr<MapData>>::finished, this,
            [this, watcher, num_objectives]() {
        auto result = watcher->result();
        if (result) {
            app_.set_map_data(std::move(result));
            grid_widget_->sync_and_update();
            solve_status_label_->setText("MO Solved");

            // Show Pareto front in panel
            const auto &front = app_.get_current_mo_front();
            std::vector<std::vector<float>> costs;
            std::vector<float> path_lengths;
            const char *default_names[] = {"Dist", "Risk", "Energy", "Vis", "Terrain"};
            std::vector<std::string> obj_names;
            for (int i = 0; i < num_objectives && i < 5; i++)
                obj_names.push_back(default_names[i]);

            for (const auto &sol : front)
            {
                costs.push_back(sol.costs);
                path_lengths.push_back(static_cast<float>(sol.path.size()));
            }

            pareto_panel_->show_front(num_objectives, obj_names, costs, path_lengths);
            if (!front.empty())
                app_.select_mo_solution(0);
        } else {
            solve_status_label_->setText("MO: No path found");
        }

        solving_ = false;
        timeout_timer_->stop();
        timeout_remaining_label_->hide();
        solve_btn_->setEnabled(true);
        solve_btn_->setText("Solve");
        cancel_btn_->setEnabled(false);
        cancel_btn_->setText("Cancel");
        watcher->deleteLater();
    });

    Coordinate start = starts[0];
    Coordinate goal  = ends[0];

    watcher->setFuture(QtConcurrent::run([this, start, goal, num_objectives]() {
        return app_.solve_mo_async_on_copy(start, goal, num_objectives);
    }));
}

void VisualizationSystem::on_mo_solve_requested(int num_objectives)
{
    mo_num_objectives_ = num_objectives;
    solve_mo_async(num_objectives);
}

void VisualizationSystem::on_mo_solution_selected(int index)
{
    app_.select_mo_solution(index);
    grid_widget_->sync_and_update();
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

    app_.set_timeout_ms(timeout_spin_->value() * 1000);
    solve_deadline_ = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_spin_->value() * 1000);
    timeout_remaining_label_->setText(
        QString("Timeout: %1s").arg(timeout_spin_->value()));
    timeout_remaining_label_->show();
    timeout_timer_->start();

    auto *watcher = new QFutureWatcher<std::shared_ptr<MapData>>(this);
    connect(watcher, &QFutureWatcher<std::shared_ptr<MapData>>::finished, this, [this, watcher]() {
        auto result = watcher->result();
        if (result) {
            app_.set_map_data(std::move(result));
            grid_widget_->sync_and_update();
            solve_status_label_->setText("Solved");
        } else {
            solve_status_label_->setText("No path found");
        }

        {
            PerfEntry entry;
            entry.pm = app_.get_performance_metrics();
            entry.ma_met = app_.get_last_ma_metrics();
            perf_buffer_.push_front(std::move(entry));
        }
        write_csv_log(perf_buffer_.front().pm, perf_buffer_.front().ma_met);
        while (perf_buffer_.size() > 5)
            perf_buffer_.pop_back();
        update_perf_sidebar();

        QTimer::singleShot(3000, this, [this]() { solve_status_label_->clear(); });
        solving_ = false;
        timeout_timer_->stop();
        timeout_remaining_label_->hide();
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

void VisualizationSystem::on_solve_clicked()
{
    if (mo_mode_)
        solve_mo_async(mo_num_objectives_);
    else
        solve_async();
}
void VisualizationSystem::on_clear_clicked()      { app_.clear_paths();                      }
void VisualizationSystem::on_reset_clicked()      { app_.reset_grid();                       }
bool VisualizationSystem::on_prev_scene()
{
    if (!app_.request_previous_scene()) return false;
    app_.clear_paths();
    grid_widget_->center_view();
    reset_perf_buffer();
    update_status();
    return true;
}
bool VisualizationSystem::on_next_scene()
{
    if (!app_.request_next_scene()) return false;
    app_.clear_paths();
    grid_widget_->center_view();
    reset_perf_buffer();
    update_status();
    return true;
}
void VisualizationSystem::on_prev_map()
{
    if (!app_.request_previous_map())
    {
        solve_status_label_->setText("No previous map");
        QTimer::singleShot(3000, this, [this]() { solve_status_label_->clear(); });
        return;
    }
    grid_widget_->reset_view();
    reset_perf_buffer();
    update_status();
}
void VisualizationSystem::on_next_map()
{
    if (!app_.request_next_map())
    {
        solve_status_label_->setText("No next map");
        QTimer::singleShot(3000, this, [this]() { solve_status_label_->clear(); });
        return;
    }
    grid_widget_->reset_view();
    reset_perf_buffer();
    update_status();
}

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
    int idx = app_.get_scene_index();
    int n_agent = app_.get_num_agents();
    int scene_id = std::max(0, idx - n_agent);
    int scene_num = scene_id / n_agent + 1;
    int total_scenes = app_.get_total_scenes();
    text += std::to_string(scene_num);
    text += " / ";
    text += std::to_string(total_scenes);
    status_label_->setText(QString::fromStdString(text));

    int max_block = (total_scenes + n_agent - 1) / n_agent;
    scene_spin_->blockSignals(true);
    scene_spin_->setMaximum(max_block);
    scene_spin_->setSuffix(" / " + QString::number(max_block));
    scene_spin_->setValue(scene_num);
    scene_spin_->blockSignals(false);

    map_spin_->blockSignals(true);
    map_spin_->setValue(app_.get_map_index() + 1);
    map_spin_->blockSignals(false);

    grid_widget_->update();
}

void VisualizationSystem::reset_perf_buffer()
{
    perf_buffer_.clear();
    perf_text_->setPlainText("No data available right now.");
}

void VisualizationSystem::update_perf_sidebar()
{
    if (perf_buffer_.empty())
    {
        perf_text_->setPlainText("No data available right now.");
        return;
    }

    auto fmt_entry = [](const PerfEntry &entry, int idx, int total) -> std::string
    {
        const auto &pm = entry.pm;
        std::string text;
        std::string label = idx == 0 ? " (most recent)" : "";
        text += "\n═══ Run #" + std::to_string(total - idx) + label + " ═══\n";
        text += "Solver:  " + pm.solver_name + "\n";
        text += "Map:     " + pm.map_name + "\n";
        text += "Scene:   " + std::to_string(pm.scene_id) + "\n";
        text += "Agents:  " + std::to_string(pm.num_agents) + "\n";
        text += "Time:    " + PerformanceMetrics::fmt_timestamp(pm.timestamp) + "\n";
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

        if (entry.ma_met.has_value())
        {
            const auto &m = *entry.ma_met;
            sep();
            text += "Multi-Agent\n";
            text += "  Sum of Costs:  " + std::to_string(pm.sum_of_costs) + "\n";
            text += "  Makespan:      " + std::to_string(pm.makespan) + "\n";
            text += "  Flow Time:     " + std::to_string(m.flow_time) + "\n";
            text += "  Mean Path:     " + std::to_string(m.mean_path_length) + "\n";
            text += "  Min Path:      " + std::to_string(m.min_path_length) + "\n";
            text += "  Max Path:      " + std::to_string(m.max_path_length) + "\n";

            sep();
            text += "Joint Search\n";
            text += "  Joint Exp'd:   " + std::to_string(m.joint_states_expanded) + "\n";
            text += "  Joint Expl'd:  " + std::to_string(m.joint_states_explored) + "\n";
            text += "  Solution Depth:" + std::to_string(m.solution_depth) + "\n";
            text += "  Max Timestep:  " + std::to_string(m.max_timestep_reached) + "\n";

            if (m.num_replan_iterations > 0)
            {
                sep();
                text += "Replanning\n";
                text += "  Iterations:    " + std::to_string(m.num_replan_iterations) + "\n";
                text += "  Conflicts:     " + std::to_string(m.conflicts_detected) + "\n";
                text += "  Joint Src:     " + std::to_string(m.num_joint_searches) + "\n";
                text += "  Src Failures:  " + std::to_string(m.num_joint_search_failures) + "\n";
                text += "  Final Set:     " + std::to_string(m.final_joint_set_size) + "\n";
            }
        }

        return text;
    };

    std::string text;
    int total = static_cast<int>(perf_buffer_.size());
    for (int i = 0; i < total; i++)
    {
        if (i > 0)
            text += "\n════════════════════════════════\n\n";
        text += fmt_entry(perf_buffer_[i], i, total);
    }
    text += "\n";

    perf_text_->setPlainText(QString::fromStdString(text));
}

#include "visualization_system.moc"
} // namespace path_sync
