#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QTimer"
#include "QPainter"
#include "math.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>


//funkcia local robot je na priame riadenie robota, ktory je vo vasej blizskoti, viete si z dat ratat polohu atd (zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
void MainWindow::localrobot(TKobukiData &sens)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    ///
    ///
    ///
    ///
    ///
    ///



    if(prvyStart)
    {

        GyroUholOld=sens.GyroAngle;
        PomEncoderL= sens.EncoderLeft;
        PomEncoderR= sens.EncoderRight;


        deltaUhol=0.0;
        prvyStart=false;
    }



    PomEncoderL=sens.EncoderLeft;
    if(dl%10==0)
    {
        ///toto je skaredy kod. rozumne je to posielat do ui cez signal slot..
        ui->lineEdit->setText(QString::number(robotX));
        ui->lineEdit_2->setText(QString::number(robotY));
        ui->lineEdit_3->setText(QString::number(robotFi));
    }
    dl++;








}

// funkcia local laser je naspracovanie dat z lasera(zapnutie dopravneho oneskorenia sposobi opozdenie dat oproti aktualnemu stavu robota)
int MainWindow::locallaser(LaserMeasurement &laserData)
{

    paintThisLidar(laserData);


    //priklad ako zastavit robot ak je nieco prilis blizko
    if(laserData.Data[0].scanDistance<500)
    {

        sendRobotCommand(ROBOT_STOP);
    }
    ///PASTE YOUR CODE HERE
    /// ****************
    /// mozne hodnoty v return
    /// ROBOT_VPRED
    /// ROBOT_VZAD
    /// ROBOT_VLAVO
    /// ROBOT_VPRAVO
    /// ROBOT_STOP
    /// ROBOT_ARC
    ///
    /// ****************

    return -1;
}


//--autonomousrobot simuluje slucku robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad polohovy regulator, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
void MainWindow::autonomousrobot(TKobukiData &sens)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
}
//--autonomouslaser simuluje spracovanie dat z robota, ktora bezi priamo na robote
// predstavte si to tak,ze ste naprogramovali napriklad sposob obchadzania prekazky, uploadli ste ho do robota a tam sa to vykonava
// dopravne oneskorenie nema vplyv na data
int MainWindow::autonomouslaser(LaserMeasurement &laserData)
{
    ///PASTE YOUR CODE HERE
    /// ****************
    ///
    ///
    /// ****************
    return -1;
}

///kamera nema svoju vlastnu funkciu ktora sa vola, ak chcete niekde pouzit obrazok, aktualny je v premennej
/// robotPicture alebo ekvivalent AutonomousrobotPicture
/// pozor na synchronizaciu, odporucam akonahle chcete robit nieco s obrazkom urobit si jeho lokalnu kopiu
/// cv::Mat frameBuf; robotPicture.copyTo(frameBuf);

procesedLidarData MainWindow::preprocesLidarData(LaserMeasurement laserData){
    procesedLidarData result;

    result.length = laserData.numberOfScans;
    for(int k=0;k<laserData.numberOfScans;k++)
    {
        result.realDistanceD[k]=laserData.Data[k].scanDistance;

        result.xObstacles[k]=720-(360+result.realDistanceD[k]*sin((360.0-laserData.Data[k].scanAngle)*3.14159/180.0));
        result.yObstacles[k]=620-(310+result.realDistanceD[k]*cos((360.0-laserData.Data[k].scanAngle)*3.14159/180.0));
        result.scanAngleRight[k] = 360.0-laserData.Data[k].scanAngle;
        result.realDistanceZ[k] = result.realDistanceD[k] * cos(result.scanAngleRight[k]*3.14159/180.0);
    }
    return result;
}
//sposob kreslenia na obrazovku, tento event sa spusti vzdy ked sa bud zavola funkcia update() alebo operacny system si vyziada prekreslenie okna

/*cv::Mat MainWindow:: lidarToCam(cv::Mat frameBuf){
    procesedLidarData lidarData;
    lidarData = preprocesLidarData(paintLaserData);
    double z = 0;
    double x = 0;
    cv::Size frameBufSize = frameBuf.size();
    for(int k=0;k<lidarData.length;k++)
    {
        xObr =frameBuf.
        yObr =
    }

    return 0;
}*/

void MainWindow::robotArcMove(double translation,double radius) //stop
{
    std::vector<unsigned char> mess=robot.setArcSpeed(translation,radius);
     if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

     }
}

void MainWindow::robotStop() //stop
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}
void MainWindow::robotRotate(double angl)
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(angl);

    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

double MainWindow::scaleAngle(double angle){
    if(angle < -PI){
        angle += 2*PI;
    }else if (angle>PI){
        angle-=2*PI;
    }
    return angle;
}

int MainWindow:: leftHandClenched(){
    double indexFinDist = euclideanDistance(kostricka.joints[jointnames::left_index_mcp].x, kostricka.joints[jointnames::left_index_mcp].y,
            kostricka.joints[jointnames::left_index_tip].x, kostricka.joints[jointnames::left_index_tip].y);
    double middleFinDist = euclideanDistance(kostricka.joints[jointnames::left_middle_mcp].x, kostricka.joints[jointnames::left_middle_mcp].y,
            kostricka.joints[jointnames::left_middle_tip].x, kostricka.joints[jointnames::left_middle_tip].y);
    double ringFinDist = euclideanDistance(kostricka.joints[jointnames::left_ring_mcp].x, kostricka.joints[jointnames::left_ring_mcp].y,
            kostricka.joints[jointnames::left_ringy_tip].x, kostricka.joints[jointnames::left_ringy_tip].y);
    double pinkyFinDist = euclideanDistance(kostricka.joints[jointnames::left_pink_mcp].x, kostricka.joints[jointnames::left_pink_mcp].y,
            kostricka.joints[jointnames::left_pink_tip].x, kostricka.joints[jointnames::left_pink_tip].y);
    printf("index %f, mid %f, ring %f, pinky %f\n", indexFinDist, middleFinDist, ringFinDist, pinkyFinDist);
    if(indexFinDist < 0.09 && middleFinDist < 0.09 && ringFinDist < 0.09 && pinkyFinDist < 0.09){
        return 1;
    }else{
       return 0;
    }
}
int MainWindow:: rightHandClenched(){
    double indexFinDist = euclideanDistance(kostricka.joints[jointnames::right_index_mcp].x, kostricka.joints[jointnames::right_index_mcp].y,
            kostricka.joints[jointnames::right_index_tip].x, kostricka.joints[jointnames::right_index_tip].y);
    double middleFinDist = euclideanDistance(kostricka.joints[jointnames::right_middle_mcp].x, kostricka.joints[jointnames::right_middle_mcp].y,
            kostricka.joints[jointnames::right_middle_tip].x, kostricka.joints[jointnames::right_middle_tip].y);
    double ringFinDist = euclideanDistance(kostricka.joints[jointnames::right_ring_mcp].x, kostricka.joints[jointnames::right_ring_mcp].y,
            kostricka.joints[jointnames::right_ringy_tip].x, kostricka.joints[jointnames::right_ringy_tip].y);
    double pinkyFinDist = euclideanDistance(kostricka.joints[jointnames::right_pink_mcp].x, kostricka.joints[jointnames::right_pink_mcp].y,
            kostricka.joints[jointnames::right_pink_tip].x, kostricka.joints[jointnames::right_pink_tip].y);
    printf("index %f, mid %f, ring %f, pinky %f\n", indexFinDist, middleFinDist, ringFinDist, pinkyFinDist);
    if(indexFinDist < 0.09 && middleFinDist < 0.09 && ringFinDist < 0.09 && pinkyFinDist < 0.09){
        return 1;
    }else{
       return 0;
    }
}
void  MainWindow::gestureRobotControll(){
    int rightClenched = rightHandClenched();
    int leftCleched = leftHandClenched();
    if(rightClenched && leftCleched){
        //robot start move
        robotArcMove(100, 32000);
    }else if(rightClenched && !leftCleched){
        //robot -R
        robotArcMove(100, 1);
    }
    else if(!rightClenched && leftCleched){
       //robot R
        robotArcMove(100, -1);
    }
    else{
        robotStop();
    }
}
double MainWindow::euclideanDistance(double x1, double y1, double x2, double y2){
   return (double)sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}
cv::Mat MainWindow:: fusionToCam(cv::Mat camPicture){
     procesedLidarData lidarData;
    lidarData = preprocesLidarData(paintLaserData);
    for(int k=0;k<lidarData.length;k++)
    {
        double dist = lidarData.realDistanceD[k]/10.0;
         double X = dist * sin(lidarData.scanAngleRight[k]*PI/180);
         double Z =  dist * cos(lidarData.scanAngleRight[k]*PI/180);
         int xPicture = (camPicture.cols/2) - ((F_PIXL * X)/Z);
         int yPicture = (camPicture.rows/2) + ((F_PIXL * 6.0)/Z);

         cv::Scalar distColor;
         cv::Rect rect;


         if((lidarData.scanAngleRight[k] <= 27 && lidarData.scanAngleRight[k] >=0) || (lidarData.scanAngleRight[k] <= 360 && lidarData.scanAngleRight[k] >=360-27)){
            if(lidarData.realDistanceD[k]<= 1000){
                double distColorCoef = lidarData.realDistanceD[k]/1000;
                distColor = cv::Scalar(255, distColorCoef*255, distColorCoef*255);
                if(xPicture>=0 && yPicture>=0 && xPicture <= camPicture.cols && yPicture <= camPicture.rows){
                   /* int blue = camPicture.at<cv::Vec3b>(yPicture,xPicture)[0];
                    int green = camPicture.at<cv::Vec3b>(yPicture,xPicture)[1];
                    int red = camPicture.at<cv::Vec3b>(yPicture,xPicture)[2];*/
                    /*cv::Point point(xPicture,yPicture);
                    cv::circle(camPicture, point, 5, distColor, -1);*/
                    cv::floodFill(camPicture, cv::Point(xPicture,yPicture), distColor, 0, cv::Scalar(25, 25, 25), cv::Scalar(25, 25, 25));
                    printf("X: %d, Y: %d \n", xPicture, yPicture);

                }
            }
        }
 }
        return camPicture;
}
void MainWindow::paintEvent(QPaintEvent *event)
{
     QPainter painter(this);
     painter.setBrush(Qt::black);
     QPen pero;
     pero.setStyle(Qt::SolidLine);
     pero.setWidth(3);
     //pero.setColor(Qt::green);
     painter.setPen(pero);
    //Initialize frame geometrics
        //skeleton
     skeletonFrame.high =  ui->frame_skeleton->geometry().height();
     skeletonFrame.width = ui->frame_skeleton->geometry().width();
     skeletonFrame.x = ui->frame_skeleton->geometry().x();
     skeletonFrame.y = ui->frame_skeleton->geometry().y();
     skeletonFrame.centerX =  ui->frame_skeleton->geometry().center().x();
     skeletonFrame.centerY = ui->frame_skeleton->geometry().center().y();
        //lidar
     lidarFrame.high =  ui->frame_lidar->geometry().height();
     lidarFrame.width = ui->frame_lidar->geometry().width();
     lidarFrame.x = ui->frame_lidar->geometry().x();
     lidarFrame.y = ui->frame_lidar->geometry().y();
     lidarFrame.centerX =  ui->frame_lidar->geometry().center().x();
     lidarFrame.centerY = ui->frame_lidar->geometry().center().y();
    //______________________________
    QRect rectLidar(lidarFrame.x, lidarFrame.y, lidarFrame.width, lidarFrame.high) ;
    rectLidar = ui->frame_lidar->geometry();
    QRect rectSkeleton(skeletonFrame.x , skeletonFrame.y, skeletonFrame.width, skeletonFrame.high) ;
    rectLidar = ui->frame_lidar->geometry();
    painter.setBrush(QColor(200, 200, 200));
    painter.setPen(QColor(200, 200, 200));
    painter.drawRect(rectLidar);
    painter.drawRect(rectSkeleton);

    painter.setBrush(Qt::white);
    painter.setPen(Qt::white);
    painter.drawEllipse(QPoint(lidarFrame.centerX, lidarFrame.centerY),3,3);

    painter.setBrush(Qt::black);
    painter.setPen(Qt::black);
    painter.drawRect(QRect(lidarFrame.centerX, lidarFrame.centerY,1, -2));
    if(updateCameraPicture==1 && showCamera==true)
    {
        robotPicture=fusionToCam(robotPicture);
        robotPicture.copyTo(robotPicture2);
        updateCameraPicture=0;
        //robotPicture = fusionToCam(robotPicture);
        QImage imgIn= QImage((uchar*) robotPicture.data, robotPicture.cols, robotPicture.rows, robotPicture.step, QImage::Format_BGR888);
        //painter.drawImage(20, 120, imgIn);
        //cv::imshow("client",robotPicture);
        ui->mywidget->paintCamera(imgIn, actualPosition.fi);
        ui->mywidget->update();
    }else if (showCamera==true && !robotPicture2.empty()){
        QImage imgIn= QImage((uchar*) robotPicture2.data, robotPicture2.cols, robotPicture2.rows, robotPicture2.step, QImage::Format_BGR888);
        ui->mywidget->paintCamera(imgIn, actualPosition.fi);
        ui->mywidget->update();
    }

    if(updateLaserPicture==1 && showLidar==true)
    {
        /// ****************
        ///you can change pen or pen color here if you want
        /// ****************
        ///


        //chcem poslat spracované dáta
        procesedLidarData lidarData;
        lidarData = preprocesLidarData(paintLaserData);
        // kresli lidar do frame

        painter.setPen(pero);


        for(int k=0;k<lidarData.length;k++)
        {
            //if(lidarData.xObstacles[k]<721 && lidarData.xObstacles[k]>19 && lidarData.yObstacles[k]<621 && lidarData.yObstacles[k]>121){
                    double dist =lidarData.realDistanceD[k]/30.0;

                if(lidarData.realDistanceD[k]< 500){
                    painter.setBrush(Qt::blue);
                    painter.setPen(Qt::blue);
                }
                else{
                    painter.setBrush(Qt::white);
                    painter.setPen(Qt::white);
                }
                    int xPaint = lidarFrame.width -(lidarFrame.width /2 + dist * sin(lidarData.scanAngleRight[k]*PI/180.0)) + rectLidar.topLeft().x();
                    int yPaint = lidarFrame.high -(lidarFrame.high /2 + dist * cos(lidarData.scanAngleRight[k]*PI/180)) + rectLidar.topLeft().y();
                    if(rectLidar.contains(xPaint, yPaint)){
                        painter.drawEllipse(QPoint(xPaint, yPaint),3,3);
                    }

            //}
          }

        ui->mywidget->paintLidar(lidarData);
        ui->mywidget->update();
    }
    if(updateSkeletonPicture==1 && showSkeleton==true)
    {

        for(int i=0;i<75;i++)
        {

            int xPaintSkeleton =skeletonFrame.x + skeletonFrame.width * kostricka.joints[i].x;
            int yPaintSkeleton =skeletonFrame.y + skeletonFrame.high * kostricka.joints[i].y;
            if(rectSkeleton.contains(xPaintSkeleton, yPaintSkeleton)){
                        painter.drawEllipse(QPoint(xPaintSkeleton, yPaintSkeleton),2,2);
                        }

        }


        //ui->mywidget->paintSkeleton(kostricka);
       // ui->mywidget->update();

    }


}



///konstruktor aplikacie, nic co tu je nevymazavajte, ak potrebujete, mozete si tu pridat nejake inicializacne parametre
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    robotX=0;
    robotY=0;
    robotFi=0;

    showCamera=false;
    showLidar=true;
    showSkeleton=false;
    applyDelay=false;
    dl=0;
    stopall=1;
    prvyStart=true;
    updateCameraPicture=0;
    ipaddress="127.0.0.1";
    std::function<void(void)> f =std::bind(&robotUDPVlakno, (void *)this);
    robotthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f2 =std::bind(&laserUDPVlakno, (void *)this);
    laserthreadHandle=std::move(std::thread(f2));


    std::function<void(void)> f3 =std::bind(&skeletonUDPVlakno, (void *)this);
    skeletonthreadHandle=std::move(std::thread(f3));

    //--ak by ste nahodou chceli konzolu do ktorej mozete vypisovat cez std::cout, odkomentujte nasledujuce dva riadky
   // AllocConsole();
   // freopen("CONOUT$", "w", stdout);


    QFuture<void> future = QtConcurrent::run([=]() {
        imageViewer();
        // Code in this block will run in another thread
    });



        Imager.start();

}

///funkcia co sa zavola ked stlacite klavesu na klavesnici..
/// pozor, ak niektory widget akceptuje klavesu, sem sa nemusite (ale mozete) dostat
/// zalezi na to ako konkretny widget spracuje svoj event
void MainWindow::keyPressEvent(QKeyEvent* event)
{
    //pre pismena je key ekvivalent ich ascii hodnoty
    //pre ine klavesy pozrite tu: https://doc.qt.io/qt-5/qt.html#Key-enum
    std::cout<<event->key()<<std::endl;


}
//--cokolvek za tymto vas teoreticky nemusi zaujimat, su tam len nejake skarede kody






MainWindow::~MainWindow()
{
    stopall=0;
    laserthreadHandle.join();
    robotthreadHandle.join();
    skeletonthreadHandle.join();
    delete ui;
}


void MainWindow::localisation(){

    double newEncLeft = sens.EncoderLeft;
    double newEncRight = sens.EncoderRight;
    double diameter = robot.getB();
    double tickToMeter = robot.getTickToMeter();

    if(actualEncLeft - newEncLeft > ENC_POSITIVE_TRESHOLD){
      lLeft = tickToMeter* (newEncLeft - actualEncLeft + ENC_MAX_VAL);
    }else if (actualEncLeft - newEncLeft < ENC_NEGATIVE_TRESHOLD){
        lLeft = tickToMeter* (newEncLeft - actualEncLeft - ENC_MAX_VAL);
    }else
        lLeft = tickToMeter* (newEncLeft - actualEncLeft);

    if(actualEncRight - newEncRight > ENC_POSITIVE_TRESHOLD){
      lRight = tickToMeter* (newEncRight - actualEncRight + ENC_MAX_VAL);
    }else if (actualEncRight - newEncRight < ENC_NEGATIVE_TRESHOLD){
        lRight = tickToMeter* (newEncRight - actualEncRight - ENC_MAX_VAL);
    }else
        lRight = tickToMeter * (newEncRight - actualEncRight);

    double dAlpha = (lRight - lLeft)*(1.0/diameter);
    newFi = actualFi + dAlpha;

    if (newFi < - PI){
            newFi += 2*PI;
        }
     else if (newFi > PI){
            newFi -= 2*PI;
        }


    actualPosition.fi = newFi;
    if(lRight == lLeft){
        actualPosition.x = actualPosition.x + lRight*cos(actualFi);
        actualPosition.y = actualPosition.y + lRight*sin(actualFi);
    }else{
        actualPosition.x = actualPosition.x + ((diameter*(lRight + lLeft)/(2.0*(lRight-lLeft)))*(sin(newFi)-sin(actualFi)));
        actualPosition.y = actualPosition.y - ((diameter*(lRight + lLeft)/(2.0*(lRight-lLeft)))*(cos(newFi)-cos(actualFi)));
    }

    actualFi = newFi;

}


void MainWindow::robotprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    rob_slen = sizeof(las_si_other);
    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char rob_broadcastene=1;
    DWORD timeout=100;

    setsockopt(rob_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100*1000);
#endif
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(stopall==1)
    {

        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other, &rob_slen)) == -1)
        {

            continue;
        }
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        struct timespec t;
        //      clock_gettime(CLOCK_REALTIME,&t);
        actualEncLeft = sens.EncoderLeft;
        actualEncRight = sens.EncoderRight;
        int returnval=robot.fillData(sens,(unsigned char*)buff);
        if(returnval==0)
        {
            {
                if(init){
                    if(actualEncLeft == 0){
                        actualEncLeft = sens.EncoderLeft;
                    }
                    if(actualEncRight == 0){
                        actualEncRight = sens.EncoderRight;
                    }
                    if(actualFi == 0){
                        actualFi = sens.GyroAngle/100.0 * PI/180.0;
                    }
                    init = false;
                }
                localisation();
                 //rightHandClenched();
                //gestureRobotControll();
            //     memcpy(&sens,buff,sizeof(sens));

            std::chrono::steady_clock::time_point timestampf=std::chrono::steady_clock::now();

            autonomousrobot(sens);

            if(applyDelay==true)
            {
                struct timespec t;
                RobotData newcommand;
                newcommand.sens=sens;
                //    memcpy(&newcommand.sens,&sens,sizeof(TKobukiData));
                //        clock_gettime(CLOCK_REALTIME,&t);
                newcommand.timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
                sensorQuerry.push_back(newcommand);
                for(int i=0;i<sensorQuerry.size();i++)
                {
                    if(( std::chrono::duration_cast<std::chrono::nanoseconds>(timestampf-sensorQuerry[i].timestamp)).count()>(2.5*1000000000))
                    {
                        localrobot(sensorQuerry[i].sens);
                        sensorQuerry.erase(sensorQuerry.begin()+i);
                        i--;
                        break;

                    }
                }

            }
            else
            {
                sensorQuerry.clear();
                localrobot(sens);
            }
        }


    }
    }

    std::cout<<"koniec thread2"<<std::endl;
}
/// vravel som ze vas to nemusi zaujimat. tu nic nieje
/// nosy litlle bastard
void MainWindow::laserprocess()
{
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char las_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    setsockopt(las_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout);
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#else
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,&las_broadcastene,sizeof(las_broadcastene));
#endif
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, rob_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(stopall==1)
    {

        if ((las_recv_len = recvfrom(las_s, (char *)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, &las_slen)) == -1)
        {

            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        //tu mame data..zavolame si funkciu

        //     memcpy(&sens,buff,sizeof(sens));
        int returnValue=autonomouslaser(measure);

        if(applyDelay==true)
        {
            struct timespec t;
            LidarVector newcommand;
            memcpy(&newcommand.data,&measure,sizeof(LaserMeasurement));
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            lidarQuerry.push_back(newcommand);
            for(int i=0;i<lidarQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-lidarQuerry[i].timestamp)).count()>(2.5*1000000000))
                {
                    returnValue=locallaser(lidarQuerry[i].data);
                    if(returnValue!=-1)
                    {
                        //sendRobotCommand(returnValue);
                    }
                    lidarQuerry.erase(lidarQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {


            returnValue=locallaser(measure);
            if(returnValue!=-1)
            {
                //sendRobotCommand(returnValue);
            }
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}

void MainWindow::on_pushButton_3_clicked()
{
    char tt=0x01;
    sendRobotCommand(tt,250);
}

void MainWindow::on_pushButton_7_clicked()
{
    char tt=0x00;
    sendRobotCommand(tt);

}

void MainWindow::on_pushButton_5_clicked()
{
    char tt=0x02;
    sendRobotCommand(tt,-250);
}


void MainWindow::on_pushButton_4_clicked()
{
    char tt=0x04;
    sendRobotCommand(tt,3.14159/4);
}

void MainWindow::on_pushButton_6_clicked()
{
    char tt=0x03;
    sendRobotCommand(tt,-3.14159/4);
}

void MainWindow::on_pushButton_clicked()
{

}

void MainWindow::sendRobotCommand(char command,double speed,int radius)
{
    globalcommand=command;
 //   if(applyDelay==false)
    {

        std::vector<unsigned char> mess;
        switch(command)
        {
        case  ROBOT_VPRED:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VZAD:
            mess=robot.setTranslationSpeed(speed);
            break;
        case ROBOT_VLAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_VPRAVO:
            mess=robot.setRotationSpeed(speed);
            break;
        case ROBOT_STOP:
            mess=robot.setTranslationSpeed(0);
            break;
        case ROBOT_ARC:
            mess=robot.setArcSpeed(speed,radius);
            break;


        }
        if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

        }
    }
  /*  else
    {
        struct timespec t;
        RobotCommand newcommand;
        newcommand.command=command;
        newcommand.radius=radius;
        newcommand.speed=speed;
        //clock_gettime(CLOCK_REALTIME,&t);
        newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        commandQuery.push_back(newcommand);
    }*/
}
/*void MainWindow::autonomousRobotCommand(char command,double speed,int radius)
{
    return;
    std::vector<unsigned char> mess;
    switch(command)
    {
    case  ROBOT_VPRED:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VZAD:
        mess=robot.setTranslationSpeed(speed);
        break;
    case ROBOT_VLAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_VPRAVO:
        mess=robot.setRotationSpeed(speed);
        break;
    case ROBOT_STOP:
        mess=robot.setTranslationSpeed(0);
        break;
    case ROBOT_ARC:
        mess=robot.setArcSpeed(speed,radius);
        break;

    }
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}
void MainWindow::robotexec()
{


    if(applyDelay==true)
    {
        struct timespec t;

        // clock_gettime(CLOCK_REALTIME,&t);
        auto timestamp=std::chrono::steady_clock::now();;//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
        for(int i=0;i<commandQuery.size();i++)
        {
       //     std::cout<<(std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()<<std::endl;
            if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-commandQuery[i].timestamp)).count()>(2.5*1000000000))
            {
                char cmd=commandQuery[i].command;
                std::vector<unsigned char> mess;
                switch(cmd)
                {
                case  ROBOT_VPRED:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VZAD:
                    mess=robot.setTranslationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VLAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_VPRAVO:
                    mess=robot.setRotationSpeed(commandQuery[i].speed);
                    break;
                case ROBOT_STOP:
                    mess=robot.setTranslationSpeed(0);
                    break;
                case ROBOT_ARC:
                    mess=robot.setArcSpeed(commandQuery[i].speed,commandQuery[i].radius);
                    break;

                }
                if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
                {

                }
                commandQuery.erase(commandQuery.begin()+i);
                i--;

            }
        }
    }
}
*/


void MainWindow::paintThisLidar(LaserMeasurement &laserData)
{
    memcpy( &paintLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture=1;
    update();
}

void MainWindow::on_pushButton_8_clicked()//forward
{
    CommandVector help;
    help.command.commandType=1;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=0;
    help.command.desiredDist=100;
    struct timespec t;

    // clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_10_clicked()//right
{
    CommandVector help;
    help.command.commandType=2;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=-20;
    help.command.desiredDist=0;
    struct timespec t;

    // clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_11_clicked()//back
{
    CommandVector help;
    help.command.commandType=1;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=0;
    help.command.desiredDist=-100;
    struct timespec t;

    //   clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}

void MainWindow::on_pushButton_9_clicked()//left
{
    CommandVector help;
    help.command.commandType=2;
    help.command.actualAngle=0;
    help.command.actualDist=0;
    help.command.desiredAngle=20;
    help.command.desiredDist=0;
    struct timespec t;

    //   clock_gettime(CLOCK_REALTIME,&t);
    help.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
    AutonomousCommandQuerry.push_back(help);
}


void MainWindow::skeletonprocess()
{

    std::cout<<"init skeleton"<<std::endl;
#ifdef _WIN32
    WSADATA wsaData = {0};
    int iResult = 0;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
#else
#endif
    ske_slen = sizeof(ske_si_other);
    if ((ske_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    char ske_broadcastene=1;
#ifdef _WIN32
    DWORD timeout=100;

    std::cout<<setsockopt(ske_s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof timeout)<<std::endl;
    std::cout<<setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene))<<std::endl;
#else
    setsockopt(ske_s,SOL_SOCKET,SO_BROADCAST,&ske_broadcastene,sizeof(ske_broadcastene));
#endif
    // zero out the structure
    memset((char *) &ske_si_me, 0, sizeof(ske_si_me));

    ske_si_me.sin_family = AF_INET;
    ske_si_me.sin_port = htons(23432);
    ske_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    ske_si_posli.sin_family = AF_INET;
    ske_si_posli.sin_port = htons(23432);
    ske_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());;//htonl(INADDR_BROADCAST);
    std::cout<<::bind(ske_s , (struct sockaddr*)&ske_si_me, sizeof(ske_si_me) )<<std::endl;;
    char command=0x00;

    skeleton bbbk;
    double measure[225];
    while(stopall==1)
    {

        if ((ske_recv_len = ::recvfrom(ske_s, (char *)&bbbk.joints, sizeof(char)*1800, 0, (struct sockaddr *) &ske_si_other, &ske_slen)) == -1)
        {

        //    std::cout<<"problem s prijatim"<<std::endl;
            continue;
        }


        memcpy(kostricka.joints,bbbk.joints,1800);
     updateSkeletonPicture=1;
  //      std::cout<<"doslo "<<ske_recv_len<<std::endl;
      //  continue;
        for(int i=0;i<75;i+=3)
        {
        //    std::cout<<klby[i]<<" "<<bbbk.joints[i].x<<" "<<bbbk.joints[i].y<<" "<<bbbk.joints[i].z<<std::endl;
        }
    }
    std::cout<<"koniec thread"<<std::endl;
}



void MainWindow::imageViewer()
{
    showLidar = 1;
    showCamera = 1;
    showSkeleton = 1;
    cv::VideoCapture cap;
    cap.open("http://127.0.0.1:8889/stream.mjpg");
    cv::Mat frameBuf;
    while(1)
    {
        cap >> frameBuf;


        if(frameBuf.rows<=0)
        {
            std::cout<<"nefunguje"<<std::endl;
            continue;
        }

        if(applyDelay==true)
        {
            struct timespec t;
            CameraVector newcommand;
            frameBuf.copyTo(newcommand.data);
            //    clock_gettime(CLOCK_REALTIME,&t);
            newcommand.timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            auto timestamp=std::chrono::steady_clock::now();//(int64_t)(t.tv_sec) * (int64_t)1000000000 + (int64_t)(t.tv_nsec);
            cameraQuerry.push_back(newcommand);
            for(int i=0;i<cameraQuerry.size();i++)
            {
                if((std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp-cameraQuerry[i].timestamp)).count()>(2.5*1000000000))
                {

                    cameraQuerry[i].data.copyTo(robotPicture);
                    cameraQuerry.erase(cameraQuerry.begin()+i);
                    i--;
                    break;

                }
            }

        }
        else
        {

            //
           frameBuf.copyTo(robotPicture);
        }
        frameBuf.copyTo(AutonomousrobotPicture);
        updateCameraPicture=1;

        update();
        std::cout<<"vycital som"<<std::endl;
       // cv::imshow("client",frameBuf);
        cv::waitKey(1);
        QCoreApplication::processEvents();
    }
}



