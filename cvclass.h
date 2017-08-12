#ifndef CVCLASS_H
#define CVCLASS_H
#include "mygeoclass.h"
#include "spclass.h"
#include "mutex.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
using namespace std;
using namespace cv;
float calcLength(Point2f vec);

class CVClass
{
private:
    int chooseCnt=0;
    vector<Point> goodTargets;
    uchar* buffer;
    struct v4l2_streamparm setfps;
    struct v4l2_capability cap;
    struct v4l2_fmtdesc fmtdesc;
    struct v4l2_format fmt,fmtack;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    enum v4l2_buf_type type;
    int v4l2_grab(void);
    VideoCapture video;
    Mat contoursImg;
    Mat element=getStructuringElement(MORPH_CROSS,Size(2,2));
    RNG rng;
    double time;
   
    int findCar(Mat src);
    void stateCheck(void);
    void sortPts(vector<Point> &pts);         //from left to right
    bool checkObs(vector<Point> pts, Mat &image);
    float calcTriArea(vector<Point> pts);
//    void buttonCallBack(int event,int x,int y,int flags,void* param);
public:
    Mat getImage(void);
    Mat gui=Mat::zeros(Size(990,480),CV_8UC3);
      int findBlack(Mat& src);
 int expo=128;
    int setCameraParam(__u32 parameter,__s32 value);
    void setButton(void);
    Point beginPt=Point(100,90);
    Point home;
    Point preV;
    bool ifChooseTarg=false;
    Point beginPtC=Point(100,0);
    char* name;
    bool isReady=false;
    bool ifUpdateEdge=false;
    int targetID=-1;
    vector<Point> targetBox;
    vector<Point2f> targetRealBox;
    pair<Point,Point> chosenTarg/*=pair<Point,Point>(Point(-1,-1),Point(-1,-1))*/;
    Point dest;
    CVClass();
    CVClass(char *filename);
    void setDevice(char *filename);
    CVClass(int device);
    int init_v4l2(void);
    vector< vector<Point> > garage;
    vector< vector<Point> > oldGarage;
    MyTargetVClass targets;
    MyEdgeClass Ledge;
    MyEdgeClass Redge;
    MyCarClass car;
    void reset(void);
    void init(void);
    void setHome(void);
    pair<Point, Point> chooseTarg(void);
    int CVProcess(void);
    int CVCalc(void);
    int CVThreadStart(void);
    static void* CVThread(void* arg);
};
extern CVClass cvClass;
#endif // CVCLASS_H
