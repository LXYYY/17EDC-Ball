#ifndef MYLINECLASS_H
#define MYLINECLASS_H
//#include "spclass.h"
//#include "cvclass.h"
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class MyTargetVClass
{
public:
    bool isGood=false;
    bool TargGood[4];
    vector<Point> oldTargets;
    vector<Point> curTargets;
    MyTargetVClass(void);
    void update(void);
};

class MyEdgeClass
{
public:
    bool isReady=false;
    enum EdgeSideE
    {
        Edge_Left,
        Edge_Right
    }EdgeSide;
    Point2f pt1;
    Point2f v1;
    Point2f pt2;
    Point2f v2;
    Point2f pt3;
    Point2f v3;
    MyTargetVClass targets;
    MyEdgeClass();
    MyEdgeClass(Point pt1,Point pt2);
    MyEdgeClass(vector<Point> pts);
    void setSide(EdgeSideE side);
    int update();
    void draw(Mat& image,Scalar color);
};

class MyTriClass
{
public:
    Point2f pt1;
    Point2f pt2;
    Point2f pt3;
    Point2f center;
    Point2f v;
    MyTriClass();
    MyTriClass(Point pt1,Point pt2,Point pt3);
    MyTriClass(vector<Point> pts);
    void update(vector<Point> pts);
    void draw(Mat& image,Scalar color);
};

class MyCarClass
{
private:
    Point2f calcCenter(vector<Point> pts);
    Point2f calcCenter(vector<Point2f> pts);
    float calcAngle(Point2f vec1,Point2f vec2);
public:
    bool ifStandby=false;
    enum MissionStageE
    {
        Standby=-1,
        Steering0,
        Reversing,
        Steering1,
        Moving1,
        Steering2,
        Moving2,
        Steering3,
    }MissionStage;
    Mat dist;
    Point preV;
    Point tmpV;
    RotatedRect redBox;
    RotatedRect blueBox;
    vector<Point2f> realBox;
    Point pos_home;
    Point2f pos;
    Point2f realPos;
    Point2f v;
    float x2Edge;
    float x2Targ;
    float y2Targ;
    float yaw;
    MyCarClass();
    Point findDir(RotatedRect redBox, Point2f blueCenter);
    void update(vector<Point> redContour, vector<Point> blueContour, MyEdgeClass Ledge,MyEdgeClass Redge);
    void draw(Mat& image);
};

#endif // MYLINECLASS_H
