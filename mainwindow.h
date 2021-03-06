#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifdef _WIN32
#include<windows.h>
#else
#include <termios.h>
#include <unistd.h>
#include "unistd.h"
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#endif
#include<iostream>

#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include <QLabel>
#include<algorithm>
#include<chrono>
#include "CKobuki.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rplidar.h"
#include <QThread>
#include <QKeyEvent>
#define ROBOT_VPRED 0x01
#define ROBOT_VZAD 0x02
#define ROBOT_VLAVO 0x04
#define ROBOT_VPRAVO 0x03
#define ROBOT_STOP 0x00
#define ROBOT_ARC 0x05
#define F_PIXL 628.036
#define ENC_POSITIVE_TRESHOLD 60000
#define ENC_NEGATIVE_TRESHOLD -60000
#define ENC_MAX_VAL 65535

static const char *klby[]={{"left_wrist"},{"left_thumb_cmc"},{"left_thumb_mcp"},{"left_thumb_ip"},{"left_thumb_tip"},{"left_index_cmc"},{"left_index_mcp"},{"left_index_ip"},{"left_index_tip"},{"left_middle_cmc"},{"left_middle_mcp"},{"left_middle_ip"},{"left_middle_tip"},{"left_ring_cmc"},{"left_ring_mcp"},{"left_ring_ip"},{"left_ringy_tip"},{"left_pinky_cmc"},{"left_pink_mcp"},{"left_pink_ip"},{"left_pink_tip"},{"right_wrist"},{"right_thumb_cmc"},{"right_thumb_mcp"},{"right_thumb_ip"},{"right_thumb_tip"},{"right_index_cmc"},{"right_index_mcp"},{"right_index_ip"},{"right_index_tip"},{"right_middle_cmc"},{"right_middle_mcp"},{"right_middle_ip"},{"right_middle_tip"},{"right_ring_cmc"},{"right_ring_mcp"},{"right_ring_ip"},{"right_ringy_tip"},{"right_pinky_cmc"},{"right_pink_mcp"},{"right_pink_ip"},{"right_pink_tip"},{"nose"},{"left_eye_in"},{"left_eye"},{"left_eye_out"},{"right_eye_in"},{"right_eye"},{"right_eye_out"},{"left_ear"},{"right_ear"},{"mounth_left"},{"mounth_right"},{"left_shoulder"},{"right_shoulder"},{"left_elbow"},{"right_elbow"},{"left_wrist"},{"right_wrist"},{"left_pinky"},{"right_pinky"},{"left_index"},{"right_index"},{"left_thumb"},{"right_thumb"},{"left_hip"},{"right_hip"},{"left_knee"},{"right_knee"},{"left_ankle"},{"right_ankle"},{"left_heel"},{"righ_heel"},{"left+foot_index"},{"right_foot_index"}};
enum jointnames
{
    left_wrist,
   left_thumb_cmc,
   left_thumb_mcp,
   left_thumb_ip,
   left_thumb_tip,
   left_index_cmc,
   left_index_mcp,
   left_index_ip,
   left_index_tip,
   left_middle_cmc,
   left_middle_mcp,
   left_middle_ip,
   left_middle_tip,
   left_ring_cmc,
   left_ring_mcp,
   left_ring_ip,
   left_ringy_tip,
   left_pinky_cmc,
   left_pink_mcp,
   left_pink_ip,
   left_pink_tip,
   right_wrist,
   right_thumb_cmc,
   right_thumb_mcp,
   right_thumb_ip,
   right_thumb_tip,
   right_index_cmc,
   right_index_mcp,
   right_index_ip,
   right_index_tip,
   right_middle_cmc,
   right_middle_mcp,
   right_middle_ip,
   right_middle_tip,
   right_ring_cmc,
   right_ring_mcp,
   right_ring_ip,
   right_ringy_tip,
   right_pinky_cmc,
   right_pink_mcp,
   right_pink_ip,
   right_pink_tip,
   nose,left_eye_in,
   left_eye,
   left_eye_out,
   right_eye_in,
   right_eye,
   right_eye_out,
   left_ear,
   right_ear,
   mounth_left,
   mounth_right,
   left_shoulder,
   right_shoulder,
   left_elbow,
   right_elbow,
   left_wrist_glob,
   right_wrist_glob,
   left_pinky,
   right_pinky,
   left_index,
   right_index,
   left_thumb,
   right_thumb,
   left_hip,
   right_hip,
   left_knee,
   right_knee,
   left_ankle,
   right_ankle,
   left_heel,
   righ_heel,
   left_foot_index,
   right_foot_index
};
typedef struct
{
    double x;
    double y;
    double z;
}klb;

typedef struct
{
    klb joints[75];
}skeleton;

typedef struct
{
    float realDistanceZ[1000];
    float realDistanceD[1000];
    int xObstacles[1000];
    int yObstacles[1000];
    double scanAngleRight[1000];
    int length;
}procesedLidarData;

typedef struct
{
    bool Start = false;
    bool Stop = true;

}RobotState;

typedef struct{
    double x =0.0;
    double y=0.0;
    double fi=0.0;
    double dist=0.0;
}Coordinates;

typedef struct
{
    double x;
    double y;
    double width;
    double high;
    double centerX;
    double centerY;
}frameGeometric;

typedef struct
{
    bool on = false;
    int command = 0;
}GestureComand;

typedef struct
{
     std::chrono::steady_clock::time_point timestamp;
    int command;
    double speed;
    int radius;
}RobotCommand;

typedef struct
{
     std::chrono::time_point<std::chrono::steady_clock> timestamp;
    TKobukiData sens;
}RobotData;


typedef struct
{
    int commandType;//0 ziaden, 1 pohyb, 2 uhol
     int desiredDist;
     int actualDist;
     int desiredAngle;
     int actualAngle;
}AutonomousCommand;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    AutonomousCommand command;
}CommandVector;

typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    LaserMeasurement data;
}LidarVector;


typedef struct
{
    std::chrono::steady_clock::time_point timestamp;
    cv::Mat data;
}CameraVector;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    double robotX;
    double robotY;
    double robotFi;
    int globalcommand;
    procesedLidarData procesLidarData;
    cv::Mat robotPicture2;

    int EncMax=65536;
    double PolohaX;
    double PolohaY;
    double Uhol;
    double GyroUhol;
    double GyroUholOld;
    double deltaUhol;
    int PomEncoderL;
    int PomEncoderR;
    int deltaEncL;
    int deltaEncR;
    GestureComand gestureCommand;

    double deltaVzdialenostL;
    double deltaVzdialenostR;
    bool prvyStart;
    double gyro;
    frameGeometric skeletonFrame;
    frameGeometric lidarFrame;
    jointnames hand;
    void localisation();
    void robotRotate(double angl);
    void robotStop();
    void robotArcMove(double translation,double radius);
    void gestureRobotControll();
    bool thumpGesture(bool right);
    cv::Mat fusionToCam(cv::Mat camPicture);

std::string ipaddress;
    std::vector<RobotCommand> commandQuery;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    //vlakno co cita robot

    bool naviguj;
    double zX;
    double zY;
    double toleranciaUhla;
    int dl;
    int stopall;
    std::thread robotthreadHandle;
 //   pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna
    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
std::thread laserthreadHandle;
  //  pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna
    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }



    std::thread skeletonthreadHandle;
      //  pthread_t laserthreadHandle; // handle na vlakno
        int skeletonthreadID;  // id vlakna
        static void *skeletonUDPVlakno(void *param)
        {
            std::cout<<"startujem ci co"<<std::endl;
            ((MainWindow*)param)->skeletonprocess();

            return 0;
        }
    QThread Imager;
    void imageViewer();
    void sendRobotCommand(char command,double speed=0,int radius=0);
  //  void autonomousRobotCommand(char command,double speed=0,int radius=0);

    void robotprocess();
    void laserprocess();
void skeletonprocess();
    void localrobot(TKobukiData &sens);
    void autonomousrobot(TKobukiData &sens);
    double scaleAngle(double angle);
    int leftHandClenched();
    int rightHandClenched();
    double jointAngle(int joint1, int joint2);
    double directionAngle(double x1, double y1, double x2, double y2);
    double euclideanDistance(double x1, double y1, double x2, double y2);
    int locallaser(LaserMeasurement &laserData);
    int autonomouslaser(LaserMeasurement &laserData);
    bool outstretchedFinger(int mcpIndex, int tipIndex);

    void paintThisLidar(LaserMeasurement &laserData);
    LaserMeasurement paintLaserData;
    int updateLaserPicture;
    int updateCameraPicture;
    int updateSkeletonPicture;
    RobotState stateRobot;
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;


    struct sockaddr_in ske_si_me, ske_si_other,ske_si_posli;

    int ske_s,  ske_recv_len;

    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    cv::Mat lidarToCam(cv::Mat frameBuf);

#ifdef _WIN32
        int rob_slen;
        int las_slen;
        int ske_slen;
#else
         unsigned int rob_slen;
         unsigned int las_slen;
         unsigned int ske_slen;
#endif
 CKobuki robot;
 TKobukiData sens;
    QTimer *timer;
    std::vector<RobotData> sensorQuerry;
    std::vector<LidarVector> lidarQuerry;
    std::vector<CameraVector> cameraQuerry;
    std::vector< CommandVector> AutonomousCommandQuerry;
    cv::Mat robotPicture;
    cv::Mat AutonomousrobotPicture;

    skeleton kostricka;
private slots:

    void on_pushButton_3_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_clicked();
  //  void robotexec();

    void on_pushButton_8_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_9_clicked();


private:

    bool showCamera;
    bool showLidar;
    bool showSkeleton;
    bool totalStop = false;

    double actualEncLeft, actualEncRight, actualFi = 0.0;
    double newFi, xPosition, yPosition, lRight, lLeft = 0.0;
    bool init = true;

    Coordinates targetPosition;
    Coordinates actualPosition;

    bool applyDelay;
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* event);
    procesedLidarData preprocesLidarData(LaserMeasurement laserData);
};

#endif // MAINWINDOW_H
