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
    if(array < 300){
        result = 3;
    }else if(array < 400){
        result = 2;
    }else if(array < 500){
        result = 1;
    }else
        result = 0;
    return result;
}

void robotScreenWidget::paintWarnings(int position, int borderPoints, QPainter &painter){
    // position
    // position = 0 - Front
    // position = 1 - Back
    // position = 2 - Right
    // position = 3 - Left
    QColor colorDistance;
    Qt::BrushStyle styleDistance = Qt::SolidPattern;
    colorDistance = QColor(255, 0, 0);
    QBrush distanceBrush(colorDistance, styleDistance);
    painter.setBrush(distanceBrush);
    for(int i = 0; i < borderPoints; i++){
        switch (position) {
          case 0:
            painter.drawRect(QRect(0, i*15, this->width(), 10));//Front
            break;
          case 1:
            painter.drawRect(QRect(0, this->height()-10 - (i*15), this->width(), 10));// Back
            break;
          case 2:
            painter.drawRect(QRect(this->width()-10 -(i*15), 0, 10, this->height()));// Right
            break;
          case 3:
            painter.drawRect(QRect(i*15, 0, 10, this->height()));// Left
            break;
        }

        }

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
    QLabel label(this);

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
    //printf("Vyska: %d \n", this->height());
    //printf("Sirka: %d \n", this->width());


    /*painter.drawRect(QRect(0, 0, 741, 10));
    painter.drawRect(QRect(0, 500, 741, 10));

    painter.drawRect(QRect(0, 0, 10, 506));
    painter.drawRect(QRect(731, 0, 10, 506));*/
    QColor colorDistance;
    Qt::BrushStyle styleDistance = Qt::SolidPattern;



    for(int k=0;k<paintLaserDataWidget.length;k++)
    {
              /* double X = paintLaserDataWidget.realDistanceD[k] * sin(paintLaserDataWidget.scanAngleRight[k]*PI/180);
                double Z =  paintLaserDataWidget.realDistanceD[k] * cos(paintLaserDataWidget.scanAngleRight[k]*PI/180);
                double xPicture = this->width()/2 - (F_PIXL * X/Z);
                double yPicture = this->height()/2 + (F_PIXL * 0.06/Z);
                double lidarAngle = paintLaserDataWidget.scanAngleRight[k];*/
                /*if(lidarAngle > 180.0){
                    lidarAngle -=360;
                }
                double anglDiff1 = robotAngle-54;
                double anglDiff2 = robotAngle+54;
                anglDiff1 = scaleAngle(anglDiff1);
                anglDiff2 = scaleAngle(anglDiff2);*/

       /*  if((lidarAngle <= 24 && lidarAngle >=0) || (lidarAngle <= 360 && lidarAngle >=360-24)){
           if(paintLaserDataWidget.realDistanceD[k]<= 150){
                    double distColorCoef = paintLaserDataWidget.realDistanceD[k]/150;

                    colorDistance = QColor(distColorCoef*255, distColorCoef*255, 255);
                    QBrush distanceBrush(colorDistance, styleDistance);
                    painter.setBrush(distanceBrush);
                    if(xPicture>=0 && yPicture>=0 && xPicture <= this->width() && yPicture <= this->height())
                                   painter.drawEllipse(QPoint(xPicture, yPicture),4,4);

            }
        }*/

        if(findBorderPoints(paintLaserDataWidget.realDistanceD[k])){
            if((paintLaserDataWidget.scanAngleRight[k] >= 315 && paintLaserDataWidget.scanAngleRight[k] <= 360) || (paintLaserDataWidget.scanAngleRight[k] >= 0 && paintLaserDataWidget.scanAngleRight[k] <= 45)){
                paintWarnings(0,findBorderPoints(paintLaserDataWidget.realDistanceD[k]), painter);
            }
            if((paintLaserDataWidget.scanAngleRight[k] >= 135 && paintLaserDataWidget.scanAngleRight[k] <= 225)){
                paintWarnings(1,findBorderPoints(paintLaserDataWidget.realDistanceD[k]), painter);
            }
            if((paintLaserDataWidget.scanAngleRight[k] > 45 && paintLaserDataWidget.scanAngleRight[k] < 135)){
               paintWarnings(3,findBorderPoints(paintLaserDataWidget.realDistanceD[k]), painter);
            }
            if((paintLaserDataWidget.scanAngleRight[k] > 225 && paintLaserDataWidget.scanAngleRight[k] < 315)){
               paintWarnings(2,findBorderPoints(paintLaserDataWidget.realDistanceD[k]), painter);
            }
        }

              if(findBorderPoints(paintLaserDataWidget.realDistanceD[k])==3){
                  QFont font=painter.font();
                  font.setPointSize(18);
                  painter.setFont(font);
                  painter.setPen(Qt::red);
                  painter.drawText(QPoint(this->width()/2, this->height()/2), "Collision");
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
