#ifndef PATH_SYNC_COST_MAP_VIEWER_HPP
#define PATH_SYNC_COST_MAP_VIEWER_HPP

#include <QDialog>
#include <QLabel>
#include <QScrollArea>
#include <QWidget>
#include <vector>

#include "path_sync_core/map_loader/cost_map.hpp"

namespace path_sync
{

class CostMapViewer : public QDialog
{
public:
    explicit CostMapViewer(const CostMap &cm, int num_objectives = 5,
                           QWidget *parent = nullptr);

private:
    QPixmap render_layer(int obj) const;
    QPixmap render_combined() const;

    const CostMap &cm_;
    int num_objectives_;
};

} // namespace path_sync

#endif
