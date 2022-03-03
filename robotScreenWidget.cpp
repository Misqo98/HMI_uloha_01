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

    QPen warningPen;
    warningPen.setStyle(Qt::SolidLine);
    warningPen.setWidth(4);
    warningPen.setColor(Qt::red);

    painter.drawImage(0, 0, ImgIn);


    painter.setPen(Qt::green);
    /*painter.drawRect(QRect(0, 0, 741, 10));
    painter.drawRect(QRect(0, 500, 741, 10));

    painter.drawRect(QRect(0, 0, 10, 506));
    painter.drawRect(QRect(731, 0, 10, 506));*/

    for(int k=0;k<paintLaserDataWidget.length;k++)
    {
        if(paintLaserDataWidget.xObstacles[k]<721 && paintLaserDataWidget.xObstacles[k]>19 && paintLaserDataWidget.yObstacles[k]<621 && paintLaserDataWidget.yObstacles[k]>121){

            if(paintLaserDataWidget.realDistanceD[k]< 30){
                painter.setPen(Qt::red);
            }
            else{
                painter.setPen(Qt::green);
            }
            painter.drawEllipse(QPoint(paintLaserDataWidget.xObstacles[k], paintLaserDataWidget.yObstacles[k]),2,2);
        }
        if(paintLaserDataWidget.realDistanceD[k]< 30){
            if((paintLaserDataWidget.scanAngleRight[k] >= 315 && paintLaserDataWidget.scanAngleRight[k] <= 360) || (paintLaserDataWidget.scanAngleRight[k] >= 0 && paintLaserDataWidget.scanAngleRight[k] <= 45)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
                painter.drawRect(QRect(0, 0, 741, 10));
            }
            if((paintLaserDataWidget.scanAngleRight[k] >= 135 && paintLaserDataWidget.scanAngleRight[k] <= 225)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
                painter.drawRect(QRect(0, 500, 741, 10));// zadok
            }
            if((paintLaserDataWidget.scanAngleRight[k] > 45 && paintLaserDataWidget.scanAngleRight[k] < 135)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
               painter.drawRect(QRect(0, 0, 10, 506));// Lavobok
            }
            if((paintLaserDataWidget.scanAngleRight[k] > 225 && paintLaserDataWidget.scanAngleRight[k] < 315)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
               painter.drawRect(QRect(731, 0, 10, 506));// Pravobok
            }
        }
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
void robotScreenWidget::paintLidar(procesedLidarData paintLaserData){
    paintLaserDataWidget = paintLaserData;

}
void robotScreenWidget::paintSkeleton(skeleton paintSkeleton){
   paintSkeletonDataWidget = paintSkeleton;

}
