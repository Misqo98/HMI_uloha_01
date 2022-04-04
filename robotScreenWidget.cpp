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

/*cv::Mat robotScreenWidget::fusionToCam(cv::Mat camPicture){


    for(int k=0;k<paintLaserDataWidget.length;k++)
    {
        //dist [cm]
        double dist = paintLaserDataWidget.realDistanceD[k]/10.0;// mm to cm
         double X = dist * sin(paintLaserDataWidget.scanAngleRight[k]*PI/180);
         double Z =  dist * cos(paintLaserDataWidget.scanAngleRight[k]*PI/180);
         int xPicture = (camPicture.cols/2) - ((F_PIXL * X)/Z);
         int yPicture = (camPicture.rows/2) + ((F_PIXL * 6.0)/Z);

         cv::Scalar distColor;
         cv::Rect rect;


         if((paintLaserDataWidget.scanAngleRight[k] < 27 && paintLaserDataWidget.scanAngleRight[k] >=0) || (paintLaserDataWidget.scanAngleRight[k] < 360 && paintLaserDataWidget.scanAngleRight[k] >333)){
            if(paintLaserDataWidget.realDistanceD[k]< 1000 && paintLaserDataWidget.realDistanceD[k] > 130){
                double distColorCoef = paintLaserDataWidget.realDistanceD[k]/1000;
                distColor = cv::Scalar(distColorCoef*255, 255, distColorCoef*255);

                if(xPicture>=0 && yPicture>=0 && xPicture < camPicture.cols && yPicture < camPicture.rows){
                   /* int blue = camPicture.at<cv::Vec3b>(yPicture,xPicture)[0];
                    int green = camPicture.at<cv::Vec3b>(yPicture,xPicture)[1];
                    int red = camPicture.at<cv::Vec3b>(yPicture,xPicture)[2];*/
                    /*cv::Point point(xPicture,yPicture);
                    cv::circle(camPicture, point, 5, distColor, -1);*/
                    /*cv::floodFill(camPicture, cv::Point(xPicture,yPicture), distColor, 0, cv::Scalar(25, 25, 25), cv::Scalar(25, 25, 25));
                    //printf("X: %d, Y: %d \n", xPicture, yPicture);

                }
            }
        }
 }
        return camPicture;
}*/

int robotScreenWidget::findBorderPoints(double array){
    int result = 0;
    if(array > 130){
        if(array < 300){
            result = 3;
        }else if(array < 400){
            result = 2;
        }else if(array < 500){
            result = 1;
        }else
            result = 0;
    }else
        result = 0;
    return result;
}
const QString robotScreenWidget::getGestureString(int command){
    switch (command) {
      case 0:
            return "Stop";
        break;
      case 1:
            return "Forward";
        break;
      case 2:
            return "Left";
        break;
      case 3:
               return "Right";
        break;
    case 4:
             return "Backward";
      break;
    case 5:
             return "Backward - Left";
      break;
    case 6:
             return "Backward - Right";
      break;

    }
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
        switch (i) {
          case 0:
            colorDistance = QColor(250, 253, 15, 100);
            break;
          case 1:
            colorDistance = QColor(250, 253, 15, 100);
            break;
          case 2:
            colorDistance = QColor(255, 0, 0,100);
            break;
          case 3:
            colorDistance = QColor(255, 0, 0,255);
            break;
        }
        QBrush distanceBrush(colorDistance, styleDistance);
        painter.setBrush(distanceBrush);
        switch (position) {
          case 0:
            painter.drawRect(QRect(0+10, i*15, this->width()-10, 10));//Front
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

    QPixmap handPixMap;
    handPixMap.load("D:\\gitHub\\HMI\\HMI_uloha_01\\hand.png");



    QPen warningPen;
    warningPen.setStyle(Qt::SolidLine);
    warningPen.setWidth(4);
    warningPen.setColor(Qt::red);
    //RobotPic=fusionToCam(RobotPic);
    QImage imgIn= QImage((uchar*) RobotPic.data, RobotPic.cols, RobotPic.rows, RobotPic.step, QImage::Format_BGR888);
    painter.drawImage(0, 0, imgIn.scaled(this->width(), this->height()));
    //printf("Vyska: %d \n", this->height());
    //printf("Sirka: %d \n", this->width());
    double lidarFrameDimension = this->height()*0.33;
    QRect rectLidar(0, 0, lidarFrameDimension, lidarFrameDimension);
    QRect gestureRect(this->width()-lidarFrameDimension, 0, lidarFrameDimension, lidarFrameDimension);
    handPixMap = handPixMap.scaled(lidarFrameDimension/5,lidarFrameDimension/5);

    //Lidar map Rectangle
    painter.setBrush(QColor(220, 220, 220));
    painter.setPen(QColor(220, 220, 220));
    painter.setOpacity(0.20);
    painter.drawRect(rectLidar);
    painter.drawRect(gestureRect);
    painter.setOpacity(1);
    // Robot in map
    painter.setBrush(Qt::white);
    painter.setPen(Qt::white);
    painter.drawEllipse(QPoint(lidarFrameDimension/2, lidarFrameDimension/2),3,3);

    painter.setBrush(Qt::black);
    painter.setPen(Qt::black);
    painter.drawRect(QRect(lidarFrameDimension/2, lidarFrameDimension/2,1, -2));

    QFont font=painter.font();
    font.setPointSize(18);
    painter.setFont(font);
    painter.setPen(Qt::white);


    if(gestureCommandW.on){
       painter.drawPixmap(QPoint(gestureRect.center().x()-20,gestureRect.y()+50), handPixMap);
       painter.drawText(QPoint(gestureRect.x(), gestureRect.y()+30), "Gesture control ON");

    }else{
        painter.drawText(QPoint(gestureRect.x(), gestureRect.y()+30), "Gesture control OFF");
    }
    painter.drawText(gestureRect, Qt::AlignCenter, getGestureString(gestureCommandW.command));

    for(int k=0;k<paintLaserDataWidget.length;k++)
    {
       // Clollision direction
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
        // Clollision warning text
         if(findBorderPoints(paintLaserDataWidget.realDistanceD[k])==3){
                  QFont font=painter.font();
                  font.setPointSize(18);
                  painter.setFont(font);
                  painter.setPen(Qt::red);
                  painter.drawText(QPoint(this->width()/2, this->height()/2), "Collision");
          }
         // Lidar map
          double dist =paintLaserDataWidget.realDistanceD[k]/30.0;
          if(paintLaserDataWidget.realDistanceD[k]< 500 && paintLaserDataWidget.realDistanceD[k] > 130){
                      painter.setBrush(Qt::blue);
                      painter.setPen(Qt::blue);
           }
           else{
              painter.setBrush(Qt::white);
              painter.setPen(Qt::white);
            }
            int xPaint = rectLidar.width() -(rectLidar.width() /2 + dist * sin(paintLaserDataWidget.scanAngleRight[k] *PI/180.0)) + rectLidar.topLeft().x();
            int yPaint = lidarFrameDimension -(lidarFrameDimension /2 + dist * cos(paintLaserDataWidget.scanAngleRight[k]*PI/180)) + rectLidar.topLeft().y();
            if(rectLidar.contains(xPaint, yPaint)){
                    painter.drawEllipse(QPoint(xPaint, yPaint),3,3);
            }

    }

}



void robotScreenWidget::draw(QPainter *painter)
{


}

void robotScreenWidget::paintCamera(cv::Mat robotPic){
    RobotPic = robotPic;


}
void robotScreenWidget::paintLidar(procesedLidarData paintLaserData){
    paintLaserDataWidget = paintLaserData;

}
void robotScreenWidget::paintSkeleton(skeleton paintSkeleton){
   paintSkeletonDataWidget = paintSkeleton;

}
void robotScreenWidget::getGesture(GestureComand command){
    gestureCommandW = command;

}
