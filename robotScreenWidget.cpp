#include <QtGui>

#include "robotScreenWidget.h"
#include "QPainter"

robotScreenWidget::robotScreenWidget(QWidget *parent)
    : QWidget(parent)
{


  }

robotScreenWidget::~robotScreenWidget()
{

}

void robotScreenWidget::paintEvent(QPaintEvent * /* event */)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    //painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);

    painter.drawImage(0, 0, ImgIn);


    for(int k=0;k<paintLaserDataWidget.numberOfScans;k++)
    {
        int dist=paintLaserDataWidget.Data[k].scanDistance/15;
        int xp=720-(360+dist*sin((360.0-paintLaserDataWidget.Data[k].scanAngle)*3.14159/180.0));
        int yp=620-(310+dist*cos((360.0-paintLaserDataWidget.Data[k].scanAngle)*3.14159/180.0));
        if(xp<721 && xp>19 && yp<621 && yp>121)
            painter.drawEllipse(QPoint(xp, yp),2,2);
    }
    painter.setPen(Qt::red);
    for(int i=0;i<75;i++)
    {
        int xp=720-720 * paintSkeletonDataWidget.joints[i].x;
        int yp=120+ 500 *paintSkeletonDataWidget.joints[i].y;

        painter.drawEllipse(QPoint(xp, yp),2,2);
    }

    draw(&painter);
}



void robotScreenWidget::draw(QPainter *painter)
{


}

void robotScreenWidget::paintCamera(QImage imgIn){
    ImgIn =imgIn;
}
void robotScreenWidget::paintLidar(LaserMeasurement paintLaserData){
    paintLaserDataWidget = paintLaserData;

}
void robotScreenWidget::paintSkeleton(skeleton paintSkeleton){
   paintSkeletonDataWidget = paintSkeleton;

}
