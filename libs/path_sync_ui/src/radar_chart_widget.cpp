#include "path_sync_ui/radar_chart_widget.hpp"

#include <QPainterPath>
#include <cmath>
#include <algorithm>

namespace path_sync
{

RadarChartWidget::RadarChartWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(180, 180);
    setMaximumSize(300, 300);
    obj_names_ = {"Dist", "Risk", "Energy", "Vis", "Terrain"};
}

void RadarChartWidget::set_objectives(const std::vector<std::string> &names)
{
    obj_names_ = names;
    num_objectives_ = static_cast<int>(names.size());
    update();
}

void RadarChartWidget::set_num_objectives(int n)
{
    num_objectives_ = n;
    obj_names_.resize(n);
    const char *defaults[] = {"Dist", "Risk", "Energy", "Vis", "Terrain"};
    for (int i = 0; i < n && i < 5; i++)
        obj_names_[i] = defaults[i];
    update();
}

void RadarChartWidget::set_selected(const std::vector<float> &costs)
{
    selected_costs_ = costs;
    update();
}

void RadarChartWidget::set_front(const std::vector<std::vector<float>> &all_costs)
{
    front_costs_ = all_costs;
    update();
}

QPointF RadarChartWidget::polar_to_cart(float angle_rad, float radius,
                                         float cx, float cy, float r) const
{
    return {cx + radius * std::sin(angle_rad) * r,
            cy - radius * std::cos(angle_rad) * r};
}

void RadarChartWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    int n = num_objectives_;
    if (n < 2) return;

    float cx = width() / 2.0f;
    float cy = height() / 2.0f;
    float r = std::min(cx, cy) - 20.0f;

    p.fillRect(rect(), QColor(30, 30, 30));

    // Draw grid rings
    for (int ring = 1; ring <= 4; ring++)
    {
        float rr = r * ring / 4.0f;
        p.setPen(QPen(QColor(60, 60, 60), 1));
        QPainterPath path;
        for (int i = 0; i <= n; i++)
        {
            float ang = 2.0f * 3.14159f * i / n;
            QPointF pt = polar_to_cart(ang, 1.0f, cx, cy, rr);
            if (i == 0) path.moveTo(pt);
            else path.lineTo(pt);
        }
        path.closeSubpath();
        p.drawPath(path);
    }

    // Draw axes
    for (int i = 0; i < n; i++)
    {
        float ang = 2.0f * 3.14159f * i / n;
        QPointF pt = polar_to_cart(ang, 1.0f, cx, cy, r);
        p.setPen(QPen(QColor(80, 80, 80), 1));
        p.drawLine(QPointF(cx, cy), pt);

        // Label
        QPointF lp = polar_to_cart(ang, 1.15f, cx, cy, r);
        p.setPen(QColor(180, 180, 180));
        p.drawText(QRectF(lp.x() - 25, lp.y() - 10, 50, 20),
                   Qt::AlignCenter,
                   QString::fromStdString(
                       i < static_cast<int>(obj_names_.size()) ? obj_names_[i] : "?"));
    }

    // Draw front solutions (faint lines)
    for (auto &fc : front_costs_)
    {
        if (static_cast<int>(fc.size()) < n) continue;
        float max_v = *std::max_element(fc.begin(), fc.end());
        if (max_v < 0.01f) max_v = 0.01f;

        p.setPen(QPen(QColor(100, 100, 100, 60), 1));
        QPainterPath fpath;
        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            float radius = std::clamp(fc[idx] / max_v, 0.0f, 1.0f);
            float ang = 2.0f * 3.14159f * idx / n;
            QPointF pt = polar_to_cart(ang, radius, cx, cy, r);
            if (i == 0) fpath.moveTo(pt);
            else fpath.lineTo(pt);
        }
        fpath.closeSubpath();
        p.drawPath(fpath);
    }

    // Draw selected solution (bold)
    if (static_cast<int>(selected_costs_.size()) >= n)
    {
        float max_v = *std::max_element(selected_costs_.begin(), selected_costs_.end());
        if (max_v < 0.01f) max_v = 0.01f;

        QPainterPath spath;
        for (int i = 0; i <= n; i++)
        {
            int idx = i % n;
            float radius = std::clamp(selected_costs_[idx] / max_v, 0.0f, 1.0f);
            float ang = 2.0f * 3.14159f * idx / n;
            QPointF pt = polar_to_cart(ang, radius, cx, cy, r);
            if (i == 0) spath.moveTo(pt);
            else spath.lineTo(pt);
        }
        spath.closeSubpath();

        p.setPen(QPen(QColor(0, 200, 255), 2));
        p.setBrush(QColor(0, 200, 255, 40));
        p.drawPath(spath);
    }
}

} // namespace path_sync
