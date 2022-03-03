#ifndef ROBOTSCREENWIDGET_H
#define ROBOTSCREENWIDGET_H


#include <QWidget>
#include <QPushButton>
#include "rplidar.h"
#include "mainwindow.h"
class QVBoxLayout;
class QHBoxLayout;
class QPushButton;

class robotScreenWidget : public QWidget
{
    Q_OBJECT

public:
    robotScreenWidget(QWidget *parent = 0);
    ~robotScreenWidget();
    void draw(QPainter *painter);
    void paintCamera(QImage imgIn);
    void paintLidar(procesedLidarData paintLaserDat);
    void paintSkeleton(skeleton skeletonIn);
protected:
    void paintEvent(QPaintEvent *event);

private:
    QImage ImgIn;
    QPushButton *btn1, *btn2;
    procesedLidarData paintLaserDataWidget;
    skeleton paintSkeletonDataWidget;


};



#endif // ROBOTSCREENWIDGET_H
