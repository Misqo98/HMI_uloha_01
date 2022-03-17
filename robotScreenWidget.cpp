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

int robotScreenWidget::findBorderPoints(double array){
    int result = 0;
    if(array < 10){
        result = 3;
    }else if(array < 20){
        result = 2;
    }else if(array < 30){
        result = 1;
    }else
        result = 0;
    return result;
}

void robotScreenWidget::paintWarnings(int position, int borderPoints){
    // position
    // position = 0 - Front
    // position = 1 - Back
    // position = 2 - Right
    // position = 3 - Left


    /*for(int i = 0; i < 3; i++){
        switch (position) {
          case 0:
            painter.drawRect(QRect(i*10, 0, this->width(), 10));//Front
            break;
          case 1:
            painter.drawRect(QRect(0, this->height(), this->width(), 10));// Back
            break;
          case 2:
            painter.drawRect(QRect(this->width(), 0, 10, this->height()));// Right
            break;
          case 3:
            painter.drawRect(QRect(0, 0, 10, this->height()));// Left
            break;
        }

        }*/

  }

double robotScreenWidget::scaleAngle(double angle){
    if(angle < -PI){
        angle += 2*PI;
    }else if (angle>PI){
        angle-=2*PI;
    }
    return angle;
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

    painter.drawImage(0, 0, ImgIn.scaled(this->width(), this->height()));
    printf("Vyska: %d \n", this->height());
    printf("Sirka: %d \n", this->width());

    painter.setPen(Qt::green);
    /*painter.drawRect(QRect(0, 0, 741, 10));
    painter.drawRect(QRect(0, 500, 741, 10));

    painter.drawRect(QRect(0, 0, 10, 506));
    painter.drawRect(QRect(731, 0, 10, 506));*/
    painter.setBrush(Qt::red);
    for(int k=0;k<paintLaserDataWidget.length;k++)
    {
                double X = paintLaserDataWidget.realDistanceD[k] * sin(paintLaserDataWidget.scanAngleRight[k]*PI/180);
                double Z =  paintLaserDataWidget.realDistanceD[k] * cos(paintLaserDataWidget.scanAngleRight[k]*PI/180);
                double xPicture = this->width()/2 - (F_PIXL * X/Z);
                double yPicture = this->height()/2 + (F_PIXL * 0.06/Z);
                double lidarAngle = paintLaserDataWidget.scanAngleRight[k];
                if(lidarAngle > 180.0){
                    lidarAngle -=360;
                }
                double anglDiff1 = robotAngle-27;
                double anglDiff2 = robotAngle+27;
                anglDiff1 = scaleAngle(anglDiff1);
                anglDiff2 = scaleAngle(anglDiff2);

         if(paintLaserDataWidget.scanAngleRight[k] < anglDiff2 && paintLaserDataWidget.scanAngleRight[k]> anglDiff1){
           if(paintLaserDataWidget.realDistanceD[k]< 30){
                painter.setPen(Qt::red);
                painter.setBrush(Qt::red);
                //printf("Kresli x= %f, y = %f \n", xPicture, yPicture);
            }
           else if(paintLaserDataWidget.realDistanceD[k]< 50){
                painter.setPen(Qt::yellow);
                painter.setBrush(Qt::yellow);
           }
           else{
               painter.setPen(Qt::green);
               painter.setBrush(Qt::green);
           }
           if(xPicture>=0 && yPicture>=0 && xPicture <= this->width() && yPicture <= this->height())
               painter.drawEllipse(QPoint(xPicture, yPicture),4,4);
        }
            //painter.drawEllipse(QPoint(paintLaserDataWidget.xObstacles[k], paintLaserDataWidget.yObstacles[k]),2,2);


        if(findBorderPoints(paintLaserDataWidget.realDistanceD[k])){
            if((paintLaserDataWidget.scanAngleRight[k] >= 315 && paintLaserDataWidget.scanAngleRight[k] <= 360) || (paintLaserDataWidget.scanAngleRight[k] >= 0 && paintLaserDataWidget.scanAngleRight[k] <= 45)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
                painter.drawRect(QRect(0, 0, this->width(), 10));
                //paintWarnings(findBorderPoints(paintLaserDataWidget.realDistanceD[k]), 0);
            }
            if((paintLaserDataWidget.scanAngleRight[k] >= 135 && paintLaserDataWidget.scanAngleRight[k] <= 225)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
                painter.drawRect(QRect(0, this->height(), this->width(), 10));// zadok
            }
            if((paintLaserDataWidget.scanAngleRight[k] > 45 && paintLaserDataWidget.scanAngleRight[k] < 135)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
               painter.drawRect(QRect(0, 0, 10, this->height()));// Lavobok
            }
            if((paintLaserDataWidget.scanAngleRight[k] > 225 && paintLaserDataWidget.scanAngleRight[k] < 315)){
                //printf("Hit angle = %f", paintLaserDataWidget.scanAngleRight[k]);
               painter.drawRect(QRect(this->width(), 0, 10, this->height()));// Pravobok
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

void robotScreenWidget::paintCamera(QImage imgIn, double angle){
    ImgIn =imgIn;
    robotAngle = angle;

}
void robotScreenWidget::paintLidar(procesedLidarData paintLaserData){
    paintLaserDataWidget = paintLaserData;

}
void robotScreenWidget::paintSkeleton(skeleton paintSkeleton){
   paintSkeletonDataWidget = paintSkeleton;

}
