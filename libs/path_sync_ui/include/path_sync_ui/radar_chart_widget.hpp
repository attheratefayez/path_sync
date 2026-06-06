#ifndef PATH_SYNC_RADAR_CHART_WIDGET_HPP
#define PATH_SYNC_RADAR_CHART_WIDGET_HPP

#include <QWidget>
#include <QPainter>
#include <vector>
#include <string>

namespace path_sync
{

class RadarChartWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RadarChartWidget(QWidget *parent = nullptr);

    void set_objectives(const std::vector<std::string> &names);
    void set_num_objectives(int n);
    void set_selected(const std::vector<float> &costs);
    void set_front(const std::vector<std::vector<float>> &all_costs);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    int num_objectives_ = 5;
    std::vector<std::string> obj_names_;
    std::vector<float> selected_costs_;
    std::vector<std::vector<float>> front_costs_;

    QPointF polar_to_cart(float angle_rad, float radius, float cx, float cy, float r) const;
};

} // namespace path_sync

#endif
