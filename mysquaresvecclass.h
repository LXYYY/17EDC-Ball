#ifndef MYSQUARESVECCLASS_H
#define MYSQUARESVECCLASS_H
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

class mySquaresVecClass:vector< vector<Point> >
{
public:
    mySquaresVecClass();

};

#endif // MYSQUARESVECCLASS_H
