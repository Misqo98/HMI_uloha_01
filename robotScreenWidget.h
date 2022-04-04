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
    void paintCamera(cv::Mat robotPic);
    void paintLidar(procesedLidarData paintLaserDat);
    void paintSkeleton(skeleton skeletonIn);
    int findBorderPoints(double array);
    void paintWarnings(int position, int borderPoints, QPainter &painter);
    double scaleAngle(double angle);
    void getGesture(GestureComand command);
    const QString getGestureString(int command);
    //cv::Mat fusionToCam(cv::Mat camPicture);

protected:
    void paintEvent(QPaintEvent *event);

private:
    //QImage ImgIn;
    cv::Mat RobotPic;
    double robotAngle = 0.0;
    QPushButton *btn1, *btn2;
    GestureComand gestureCommandW;
    procesedLidarData paintLaserDataWidget;
    skeleton paintSkeletonDataWidget;


};



#endif // ROBOTSCREENWIDGET_H
