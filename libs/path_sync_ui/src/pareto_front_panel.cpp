#include "path_sync_ui/pareto_front_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QGroupBox>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace path_sync
{

ParetoFrontPanel::ParetoFrontPanel(QWidget *parent)
    : QWidget(parent)
{
    auto *main_layout = new QVBoxLayout(this);

    // ── Objective count selector ──
    auto *count_layout = new QHBoxLayout();
    count_layout->addWidget(new QLabel("Objectives:"));
    obj_count_spin_ = new QSpinBox();
    obj_count_spin_->setRange(2, 5);
    obj_count_spin_->setValue(5);
    count_layout->addWidget(obj_count_spin_);
    solve_btn_ = new QPushButton("Solve MO");
    count_layout->addWidget(solve_btn_);
    main_layout->addLayout(count_layout);

    // ── Weight sliders ──
    auto *w_group = new QGroupBox("Weights");
    weights_container_ = new QWidget();
    auto *w_layout = new QVBoxLayout(weights_container_);
    w_layout->setContentsMargins(0, 0, 0, 0);
    w_group->setLayout(w_layout);
    // outer layout wraps the group
    auto *w_outer = new QVBoxLayout();
    w_outer->addWidget(w_group);
    main_layout->addLayout(w_outer);

    // ── Radar chart ──
    radar_ = new RadarChartWidget();
    main_layout->addWidget(radar_);

    // ── Pareto front table ──
    table_ = new QTableWidget();
    table_->setColumnCount(3);
    table_->setHorizontalHeaderLabels({"#", "Costs", "Length"});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setMaximumHeight(150);
    main_layout->addWidget(table_);

    // ── Hypervolume label ──
    hypervolume_label_ = new QLabel("HV: —");
    main_layout->addWidget(hypervolume_label_);

    // ── Connections ──
    connect(solve_btn_, &QPushButton::clicked, this, &ParetoFrontPanel::on_solve_clicked);
    connect(obj_count_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ParetoFrontPanel::on_objective_count_changed);
    connect(table_, &QTableWidget::itemSelectionChanged,
            this, &ParetoFrontPanel::on_row_selected);

    rebuild_weight_sliders(5);
}

void ParetoFrontPanel::rebuild_weight_sliders(int n)
{
    for (auto *s : weight_sliders_) delete s;
    for (auto *l : weight_labels_) delete l;
    for (auto *v : weight_value_labels_) delete v;
    weight_sliders_.clear();
    weight_labels_.clear();
    weight_value_labels_.clear();

    auto *w_layout = qobject_cast<QVBoxLayout *>(weights_container_->layout());
    if (!w_layout) return;

    num_objectives_ = n;

    const char *defaults[] = {"Dist", "Risk", "Energy", "Vis", "Terrain"};
    for (int i = 0; i < n; i++)
    {
        auto *row = new QHBoxLayout();

        auto *label = new QLabel(i < 5 ? defaults[i] : "?");
        weight_labels_.push_back(label);
        row->addWidget(label);

        auto *slider = new QSlider(Qt::Horizontal);
        slider->setRange(1, 100);
        slider->setValue(20);
        weight_sliders_.push_back(slider);
        row->addWidget(slider);

        auto *val = new QLabel("0.20");
        weight_value_labels_.push_back(val);
        row->addWidget(val);

        auto *w_item = new QWidget();
        w_item->setLayout(row);
        w_layout->addWidget(w_item);

        connect(slider, &QSlider::valueChanged, this, &ParetoFrontPanel::on_weight_changed);
    }

    w_layout->addStretch();
    on_weight_changed();
}

void ParetoFrontPanel::on_weight_changed()
{
    std::vector<float> w;
    float sum = 0.0f;
    for (auto *s : weight_sliders_)
        sum += s->value();

    if (sum < 0.001f) sum = 1.0f;

    for (int i = 0; i < static_cast<int>(weight_sliders_.size()); i++)
    {
        float v = weight_sliders_[i]->value() / sum;
        w.push_back(v);
        if (i < static_cast<int>(weight_value_labels_.size()))
            weight_value_labels_[i]->setText(QString::number(v, 'f', 2));
    }

    emit weights_changed(w);
}

void ParetoFrontPanel::show_front(int num_objectives,
                                   const std::vector<std::string> &obj_names,
                                   const std::vector<std::vector<float>> &costs,
                                   const std::vector<float> &path_lengths)
{
    front_costs_ = costs;
    radar_->set_front(costs);

    table_->setRowCount(static_cast<int>(costs.size()));
    for (int r = 0; r < static_cast<int>(costs.size()); r++)
    {
        auto *num_item = new QTableWidgetItem(QString::number(r));
        num_item->setFlags(num_item->flags() & ~Qt::ItemIsEditable);
        table_->setItem(r, 0, num_item);

        QString cost_str;
        for (int c = 0; c < static_cast<int>(costs[r].size()); c++)
        {
            if (c > 0) cost_str += ", ";
            cost_str += QString::number(costs[r][c], 'f', 2);
        }
        auto *cost_item = new QTableWidgetItem(cost_str);
        cost_item->setFlags(cost_item->flags() & ~Qt::ItemIsEditable);
        table_->setItem(r, 1, cost_item);

        auto *len_item = new QTableWidgetItem(
            r < static_cast<int>(path_lengths.size()) ? QString::number(path_lengths[r], 'f', 1) : "—");
        len_item->setFlags(len_item->flags() & ~Qt::ItemIsEditable);
        table_->setItem(r, 2, len_item);
    }
    table_->resizeColumnsToContents();

    // Hypervolume (reference = max of each objective + 10%)
    if (costs.empty()) return;
    int n = static_cast<int>(costs[0].size());
    std::vector<float> ref(n, 0.0f);
    for (auto &c : costs)
        for (int i = 0; i < n; i++)
            ref[i] = std::max(ref[i], c[i]);
    for (auto &r : ref) r *= 1.1f;
    if (n == 0) return;

    // Simple hypervolume by grid sampling
    float hv = 1.0f;
    // Use monte carlo for higher dimensions
    int samples = n <= 3 ? 50000 : 200000;
    int dominated = 0;
    for (int s = 0; s < samples; s++)
    {
        std::vector<float> pt(n);
        for (int i = 0; i < n; i++)
            pt[i] = static_cast<float>(rand()) / RAND_MAX * ref[i];

        for (auto &c : costs)
        {
            bool all_leq = true;
            for (int i = 0; i < n; i++)
                if (pt[i] > c[i]) { all_leq = false; break; }
            if (all_leq) { dominated++; break; }
        }
    }
    float ref_vol = 1.0f;
    for (int i = 0; i < n; i++) ref_vol *= ref[i];

    hv = ref_vol * dominated / static_cast<float>(samples);

    hypervolume_label_->setText(
        QString("HV: %1 (ref: %2 obj)").arg(hv, 0, 'e', 3).arg(n));
}

void ParetoFrontPanel::select_solution(int index)
{
    if (index < 0 || index >= static_cast<int>(front_costs_.size())) return;
    table_->selectRow(index);
    if (index < static_cast<int>(front_costs_.size()))
        radar_->set_selected(front_costs_[index]);
}

void ParetoFrontPanel::on_row_selected()
{
    auto rows = table_->selectionModel()->selectedRows();
    if (rows.isEmpty()) return;
    int idx = rows[0].row();
    select_solution(idx);
    emit solution_selected(idx);
}

void ParetoFrontPanel::on_solve_clicked()
{
    emit solve_requested(obj_count_spin_->value());
}

void ParetoFrontPanel::on_objective_count_changed(int n)
{
    rebuild_weight_sliders(n);
    radar_->set_num_objectives(n);
}

} // namespace path_sync
