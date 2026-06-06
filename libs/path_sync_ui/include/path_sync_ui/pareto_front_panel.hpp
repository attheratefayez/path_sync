#ifndef PATH_SYNC_PARETO_FRONT_PANEL_HPP
#define PATH_SYNC_PARETO_FRONT_PANEL_HPP

#include <QWidget>
#include <QSlider>
#include <QTableWidget>
#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <vector>
#include <string>

#include "path_sync_ui/radar_chart_widget.hpp"

namespace path_sync
{

class ParetoFrontPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ParetoFrontPanel(QWidget *parent = nullptr);

    void show_front(int num_objectives,
                    const std::vector<std::string> &obj_names,
                    const std::vector<std::vector<float>> &costs,
                    const std::vector<float> &path_lengths);
    void select_solution(int index);

signals:
    void solution_selected(int index);
    void solve_requested(int num_objectives);
    void weights_changed(const std::vector<float> &weights);

private slots:
    void on_row_selected();
    void on_solve_clicked();
    void on_objective_count_changed(int n);
    void on_weight_changed();

private:
    void rebuild_weight_sliders(int n);

    RadarChartWidget *radar_;
    QTableWidget *table_;
    QSpinBox *obj_count_spin_;
    QPushButton *solve_btn_;
    QWidget *weights_container_;
    QLabel *hypervolume_label_;

    std::vector<QSlider *> weight_sliders_;
    std::vector<QLabel *> weight_labels_;
    std::vector<QLabel *> weight_value_labels_;

    int num_objectives_ = 5;
    std::vector<std::string> obj_names_;
    std::vector<std::vector<float>> front_costs_;
};

} // namespace path_sync

#endif
