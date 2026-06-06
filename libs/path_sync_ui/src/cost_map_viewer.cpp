#include "path_sync_ui/cost_map_viewer.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QImage>
#include <algorithm>
#include <cmath>

namespace path_sync
{

static const char *OBJ_NAMES[] = {"Distance", "Risk", "Energy", "Visibility", "Terrain"};

CostMapViewer::CostMapViewer(const CostMap &cm, int num_objectives, QWidget *parent)
    : QDialog(parent), cm_(cm), num_objectives_(num_objectives)
{
    setWindowTitle("Cost Map Layers");
    setMinimumSize(600, 400);

    auto *root = new QVBoxLayout(this);

    auto *info = new QLabel(
        QString("%1 × %2  |  %3 objectives  |  viewing %4")
            .arg(cm.width).arg(cm.height).arg(cm.objectives).arg(num_objectives));
    info->setStyleSheet("color: #ccc; font-size: 13px; padding: 4px;");
    root->addWidget(info);

    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    auto *container = new QWidget;
    auto *lay = new QVBoxLayout(container);

    int show = std::min(num_objectives_, cm_.objectives);

    for (int o = 0; o < show; o++)
    {
        auto *row = new QHBoxLayout;

        const char *name = o < 5 ? OBJ_NAMES[o] : "?";
        auto *label = new QLabel(QString("%1").arg(name));
        label->setStyleSheet("color: #aaa; font-weight: bold; min-width: 70px;");
        row->addWidget(label);

        QPixmap px = render_layer(o);
        auto *lbl = new QLabel;
        lbl->setPixmap(px);
        row->addWidget(lbl);
        row->addStretch();

        lay->addLayout(row);
    }

    // Combined false-color overlay
    auto *sep = new QFrame;
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("color: #444;");
    lay->addWidget(sep);

    QString comb_label_text;
    if (show == 1)
        comb_label_text = "Combined (grayscale)";
    else if (show == 2)
        comb_label_text = "Combined (R,G = Distance, Risk)";
    else
    {
        QStringList ch;
        for (int c = 0; c < std::min(show, 3); c++)
            ch << OBJ_NAMES[c];
        comb_label_text = QString("Combined (R,G,B = %1)").arg(ch.join(", "));
    }
    auto *comb_label = new QLabel(comb_label_text);
    comb_label->setStyleSheet("color: #aaa; font-weight: bold;");
    lay->addWidget(comb_label);

    QPixmap comb = render_combined();
    auto *comb_lbl = new QLabel;
    comb_lbl->setPixmap(comb);
    lay->addWidget(comb_lbl);

    lay->addStretch();
    scroll->setWidget(container);
    root->addWidget(scroll);
}

static QColor heat_color(float v, float min_v, float max_v, bool blocked)
{
    if (blocked) return QColor(20, 20, 20);

    float t = (max_v > min_v + 1e-8f)
        ? std::clamp((v - min_v) / (max_v - min_v), 0.0f, 1.0f)
        : 0.0f;

    // Blue → Cyan → Green → Yellow → Red
    int r = static_cast<int>(std::min(t * 2.0f, 1.0f) * 255);
    int g = static_cast<int>((t < 0.5f ? t * 2.0f : 2.0f - t * 2.0f) * 255);
    int b = static_cast<int>(std::max(1.0f - t * 2.0f, 0.0f) * 255);
    return QColor(r, g, b);
}

QPixmap CostMapViewer::render_layer(int obj) const
{
    int scale = std::max(1, 600 / std::max(cm_.width, cm_.height));
    int pw = cm_.width * scale;
    int ph = cm_.height * scale;
    if (pw < 1 || ph < 1) return QPixmap(1, 1);

    QImage img(pw, ph, QImage::Format_ARGB32);

    float min_v = 1e9f, max_v = -1e9f;
    for (int y = 0; y < cm_.height; y++)
        for (int x = 0; x < cm_.width; x++)
        {
            float v = cm_.at(obj, x, y);
            if (v >= 0) { min_v = std::min(min_v, v); max_v = std::max(max_v, v); }
        }

    for (int py = 0; py < ph; py++)
        for (int px = 0; px < pw; px++)
        {
            int mx = px / scale;
            int my = py / scale;
            float v = cm_.at(obj, mx, my);
            bool blocked = v < 0;
            QColor c = heat_color(v, min_v, max_v, blocked);
            img.setPixelColor(px, py, c);
        }

    return QPixmap::fromImage(img);
}

QPixmap CostMapViewer::render_combined() const
{
    int scale = std::max(1, 600 / std::max(cm_.width, cm_.height));
    int pw = cm_.width * scale;
    int ph = cm_.height * scale;
    if (pw < 1 || ph < 1) return QPixmap(1, 1);

    int n = std::min(num_objectives_, cm_.objectives);
    if (n < 2)
        return render_layer(0);

    QImage img(pw, ph, QImage::Format_ARGB32);

    int ch_count = std::min(n, 3);
    float min_v[3] = {1e9f, 1e9f, 1e9f};
    float max_v[3] = {-1e9f, -1e9f, -1e9f};
    for (int o = 0; o < ch_count; o++)
        for (int y = 0; y < cm_.height; y++)
            for (int x = 0; x < cm_.width; x++)
            {
                float v = cm_.at(o, x, y);
                if (v >= 0) { min_v[o] = std::min(min_v[o], v); max_v[o] = std::max(max_v[o], v); }
            }

    for (int py = 0; py < ph; py++)
        for (int px = 0; px < pw; px++)
        {
            int mx = px / scale;
            int my = py / scale;
            bool blocked = false;
            float ch[3] = {0, 0, 0};
            for (int o = 0; o < ch_count; o++)
            {
                float v = cm_.at(o, mx, my);
                if (v < 0) { blocked = true; break; }
                float t = (max_v[o] > min_v[o] + 1e-8f)
                    ? std::clamp((v - min_v[o]) / (max_v[o] - min_v[o]), 0.0f, 1.0f)
                    : 0.0f;
                ch[o] = t;
            }
            if (blocked)
                img.setPixelColor(px, py, QColor(20, 20, 20));
            else if (n == 2)
                img.setPixelColor(px, py, QColor(
                    static_cast<int>(ch[0] * 255),
                    static_cast<int>(ch[1] * 255),
                    0));
            else
                img.setPixelColor(px, py, QColor(
                    static_cast<int>(ch[0] * 255),
                    static_cast<int>(ch[1] * 255),
                    static_cast<int>(ch[2] * 255)));
        }

    return QPixmap::fromImage(img);
}

} // namespace path_sync
