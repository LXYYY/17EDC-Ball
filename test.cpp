#include <opencv2/opencv.hpp>
#include <iostream>
#include "spclass.h"
#include "cvclass.h"
#include "mutex.h"
using namespace cv;
using namespace std;
pthread_mutex_t mutex;

struct FrameR_S
{
short ifBall;
short targets[9][2];
short ball[2];
}__attribute__ ((packed)) Frame;

bool _flag=false;
void ColorCallBack(int event,int x,int y,int flags,void* param)
{
    Mat* img=(Mat*)param;
    Mat color=Mat::zeros(Size(500,20),CV_8UC3);
    if(event==EVENT_LBUTTONDOWN)
    {
        _flag=true;
//        cout<<"down "<<flag<<endl;
    }
    if(event==EVENT_LBUTTONUP)
    {
//        cout<<x<<" "<<y<<endl;
//        cout<<flag<<endl;
        if(_flag==true)
        {
            char num[30];
            sprintf(num,"%d,%d,%d,%d,%d",(int)img->at<Vec3b>(y,x)[0],(int)img->at<Vec3b>(y,x)[1],(int)img->at<Vec3b>(y,x)[2],x,y);
            putText(color,num,Point(0,20),CV_FONT_NORMAL,1,Scalar(255,255,255));
//            cout<<(int)img->at<Vec3b>(y,x)[0]<<" "<<(int)img->at<Vec3b>(y,x)[1]<<" "<<(int)img->at<Vec3b>(y,x)[2]<<endl;
        }
        _flag=false;
    }
    imshow("color",color);
}

int main()
{
	vector<Point> pTargets(9);
	serialPort.SerialOpen("/dev/ttyUSB0");
	if(serialPort.SerialStart(115200)==-1){
		cout<<"SerialPort Open Failed"<<endl;
		return 0;
		}


	cvClass.setDevice("/dev/video0");
	cvClass.init();
	VideoCapture cap(0);
	Mat frame;
	Mat src;
	Point tBall;
	namedWindow("src");
	int grayThresh=110;
	createTrackbar("thresh","src",&grayThresh,255);
	
	while(1){
		double time;
   		time=static_cast<double>(getTickCount());
		Mat _frame;
		frame=cvClass.getImage();
		Mat channels[3];
		
		resize(frame,frame,Size(320,240));
		frame=frame(Rect(Point(20,10),Point(258,240)));
		frame.copyTo(src);
		split(frame,channels);
		
//		imshow("blue",channels[0]);
//		imshow("green",channels[1]);
//		imshow("red",(channels[2]-channels[0])*5);
		frame=((channels[0]-channels[2])*10);
		Mat targets=((channels[2]-channels[1])*6);
		threshold(targets,targets,190,255,THRESH_BINARY);
		imshow("targets",targets);
		//cvClass.findBlack(frame);
		setMouseCallback("src",ColorCallBack,&src);
	//	cvtColor(frame,frame,COLOR_BGR2GRAY);	
//	imshow("frame",frame);		
		threshold(frame,frame,grayThresh,255,THRESH_BINARY);
/*		frame.copyTo(_frame);
		vector<vector<Point> > contours;
		findContours(frame,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		//cvtColor(frame,frame,COLOR_GRAY2BGR);
		for(int i=0;i<contours.size();i++){
			if(contourArea(contours.at(i))>5000)
						drawContours(frame,contours,i,Scalar(255),-1);
		}
*/
//imshow("_frame",_frame);

		vector<vector<Point> > target;
		vector<Point> rTarget;
		rTarget.clear();
		findContours(targets,target,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		cvtColor(targets,targets,COLOR_GRAY2BGR);
		for(int i=0;i<target.size();i++){
		//	if(contourArea(balls.at(i))>50){
				RotatedRect box;
				box=minAreaRect(target.at(i));

				Point2f center;
				float r;
				minEnclosingCircle(target.at(i),center,r);

				if(box.size.area()>10){
				if(fabs(box.size.height/(box.size.height+box.size.width)-0.5)<0.2){
				 drawContours(targets,target,i,Scalar(0,0,255),-1);
				 Moments m=moments(target.at(i));
				rTarget.push_back(center);
						
				}
			}
		}

		if(rTarget.size()<=9)
		{
			vector<pair<int,Point> > rTId;

			for(int i=0;i<rTarget.size();i++){
			int id=(2-(rTarget.at(i).x/(targets.cols/3)))*3+(rTarget.at(i).y/(targets.rows/3))+1;

			pTargets.at(id-1)=rTarget.at(i);
			circle(src,rTarget.at(i),3,Scalar(0,0,255),-1);}
		}
//		imshow("targets",targets);
		Mat ball=frame;
			//	imshow("ball",ball);
		vector<vector<Point> > balls;
		findContours(ball,balls,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
		cvtColor(ball,ball,COLOR_GRAY2BGR);
		Frame.ifBall=0;
		Frame.ball[0]=0;
		Frame.ball[1]=0;
		for(int i=0;i<balls.size();i++){
		//	if(contourArea(balls.at(i))>50){
				RotatedRect box;
				box=minAreaRect(balls.at(i));
				if(box.size.area()>10){
				if(fabs(box.size.height/(box.size.height+box.size.width)-0.5)<0.2){
				 drawContours(ball,balls,i,Scalar(0,0,255),-1);
				 //Moments m=moments(balls.at(i));

				Point2f center;
				float r;
				minEnclosingCircle(balls.at(i),center,r);

				//tBall=Point(m.m10/m.m00,m.m01/m.m00);
				tBall=center;
				unsigned short x=tBall.x,y=tBall.y;
				Frame.ball[0]=x-pTargets.at(4).x;
				Frame.ball[1]=-y+pTargets.at(4).y;
				Frame.ifBall=1;
//				tBall.x=0.707*(tBall.x-90);
//				tBall.y=-0.707*(tBall.y-85);
//				x=(unsigned short)((tBall.x+tBall.y)+2048);
//				y=(unsigned short)((-tBall.x+tBall.y)+2048);	
				cout<<x<<","<<y<<endl;
				cout<<Frame.ball[0]<<","<<Frame.ball[1]<<endl;

				}
			}
		}
	for(int i=0;i<9;i++){
		Frame.targets[i][0]=pTargets.at(i).x-pTargets.at(4).x;
		Frame.targets[i][1]=-pTargets.at(i).y+pTargets.at(4).y;
			cout<<"target:"<<i<<" "<<Frame.targets[i][0]<<" "<<Frame.targets[i][1]<<endl;
	}
	serialPort.SerialWrite((unsigned char*)&Frame,sizeof(Frame));
	circle(src,tBall,3,Scalar(0,255,0),-1);
	//cout << tBall << endl;
	//imshow("contours",ball);


		imshow("src",src);

		
		if(waitKey(1)=='q')
		  return 0;
    time=(double)(-time+static_cast<double>(getTickCount()))/getTickFrequency();
    cout<<"time="<<time<<endl<<endl;
		}
	return 0;
}
