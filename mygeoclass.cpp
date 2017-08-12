#include "mygeoclass.h"
#include "cvclass.h"

float calcCos(Point2f vec1,Point2f vec2)
{
    float a,b,ab;
    ab=vec1.x*vec2.x+vec1.y*vec2.y;
    a=(float)sqrt(vec1.x*vec1.x+vec1.y*vec1.y);
    b=(float)sqrt(vec2.x*vec2.x+vec2.y*vec2.y);
    return (float)(ab/(a*b));
}

float MyCarClass::calcAngle(Point2f vec1,Point2f vec2)
{
    int dir=1;
    dir=(vec1.x*vec2.y-vec1.y*vec2.x)>0?1:-1;
    return (float)acos(calcCos(vec1,vec2))*dir;
}

//line
MyEdgeClass::MyEdgeClass()
{

}

void MyEdgeClass::setSide(EdgeSideE edgeSide)
{
    EdgeSide=edgeSide;
}

MyEdgeClass::MyEdgeClass(Point pt1,Point pt2)
{
    this->pt1=pt1;
    this->pt2=pt2;
}

MyEdgeClass::MyEdgeClass(vector<Point> pts)
{
    Vec4i line;
    fitLine(pts,line,CV_DIST_L2,0,0.01,0.01);
    this->v1.x=line[0];
    this->v1.y=line[1];
    this->pt1.x=line[2];
    this->pt1.y=line[3];
}

int MyEdgeClass::update()
{
    //    cout<<(Mat)pts<<endl
    //    cout<<"test"<<endl;

    if(targets.oldTargets.size()==0)
    {
        targets.oldTargets=targets.curTargets;
        //        cout<<"wrong"<<endl;
        //        getchar();
    }
    else
    {
        vector<bool> matched(targets.oldTargets.size(),false);
        for(int i=0;i<targets.curTargets.size();i++)
        {
            bool match=false;
            for(int j=0;j<targets.oldTargets.size();j++)
            {
                if(!matched.at(j))
                {
                    targets.TargGood[j]=false;
                    if(((targets.curTargets.at(i).x-targets.oldTargets.at(j).x)*(targets.curTargets.at(i).x-targets.oldTargets.at(j).x)
                        +(targets.curTargets.at(i).y-targets.oldTargets.at(j).y)*(targets.curTargets.at(i).y-targets.oldTargets.at(j).y))<10)
                    {
                        targets.oldTargets.at(j)=targets.curTargets.at(i);
                        matched.at(j)=true;
                        targets.TargGood[j]=true;
                        match=true;
                        break;
                    }
                }
            }
            if(!match)
            {
                if(targets.oldTargets.size()<4&&EdgeSide==Edge_Left)
                {
                    targets.oldTargets.push_back(targets.curTargets.at(i));
                }
                else if(targets.oldTargets.size()<4&&EdgeSide==Edge_Right)
                {
                    targets.oldTargets.push_back(targets.curTargets.at(i));
                    //                    cout<<"new:"<<targets.curTargets.at(i)<<endl;
                }
            }
        }
    }
    //    if(isReady=true) return 1;
    if(EdgeSide==Edge_Left&&!isReady)
    {
        if(targets.oldTargets.size()>=2)
        {
            Vec4f line;
            fitLine(targets.oldTargets,line,CV_DIST_HUBER,0,0.01,0.01);
            this->v1.x=line[0];
            this->v1.y=line[1];
            this->pt1.x=line[2];
            this->pt1.y=line[3];
            if(this->v1.y>0) this->v1*=-1;
            isReady=true;
        }
    }
    else if(EdgeSide==Edge_Right&&!isReady)
    {
        //        cout<<"targets.oldTargets"<<endl<<(Mat)targets.oldTargets<<endl;
        //        cout<<"test"<<targets.oldTargets.size()<<endl;

        if(targets.oldTargets.size()==4)
        {
            for(int n=0;n<6;n++)
            {
                for(int i=0;i<targets.oldTargets.size();i++)
                {
                    for(int j=i+1;j<targets.oldTargets.size();j++)
                    {
                        if(targets.oldTargets.at(i).y>targets.oldTargets.at(j).y)
                        {
                            Point2f tmp;
                            tmp=targets.oldTargets.at(i);
                            targets.oldTargets.at(i)=targets.oldTargets.at(j);
                            targets.oldTargets.at(j)=tmp;
                        }
                    }
                }
            }
            vector<Point> pts2fit;
            pts2fit.push_back(targets.oldTargets.at(0));
            //            pts2fit.push_back(targets.oldTargets.at(1));
            //            pts2fit.push_back(targets.oldTargets.at(2));
            pts2fit.push_back(targets.oldTargets.at(3));
            Vec4f line;
            fitLine(pts2fit,line,CV_DIST_HUBER,0,0.01,0.01);
            this->v1.x=-line[0];
            this->v1.y=-line[1];
            this->pt1.x=line[2];
            this->pt1.y=line[3];
            if(this->v1.y>0) this->v1*=-1;
            pts2fit.clear();
            pts2fit.push_back(targets.oldTargets.at(1));
            pts2fit.push_back(targets.oldTargets.at(2));
            //            pts2fit.push_back(targets.oldTargets.at(4));
            //            pts2fit.push_back(targets.oldTargets.at(5));
            //            pts2fit.push_back(targets.oldTargets.at(6));
            //            pts2fit.push_back(targets.oldTargets.at(7));
            fitLine(pts2fit,line,CV_DIST_HUBER,0,0.01,0.01);
            this->v2.x=-line[0];
            this->v2.y=-line[1];
            this->pt2.x=line[2];
            this->pt2.y=line[3];
            if(this->v2.y>0) this->v2*=-1;

            pts2fit.clear();
            for(int y=0;y<480;y++)
            {
                Point2f pt[2];
                float n=0,x;
                n=(y-cvClass.Ledge.pt1.y)/cvClass.Ledge.v1.y;
                x=cvClass.Ledge.pt1.x+n*cvClass.Ledge.v1.x;
                pt[0]=Point(x,y);
                n=0;
                n=(y-cvClass.Redge.pt1.y)/cvClass.Redge.v1.y;
                x=cvClass.Redge.pt1.x+n*cvClass.Redge.v1.x;
                pt[1]=Point(x,y);
    //            x2Edge=350/fabs(pt[1].x-pt[0].x)*fabs(pos.x-pt[0].x);
                x=fabs(pt[1].x-pt[0].x)/350*500+pt[0].x;
    //            cout<<x<<" "<<y<<endl;
                pts2fit.push_back(Point(x,y));
//                circle(fframe,Point(x,y),1,Scalar(0,0,255));
            }
            fitLine(pts2fit,line,CV_DIST_HUBER,0,0.01,0.01);
            this->v3.x=-line[0];
            this->v3.y=-line[1];
            this->pt3.x=line[2];
            this->pt3.y=line[3];
            if(this->v3.y>0) this->v3*=-1;
            isReady=true;
        }
    }
    targets.curTargets.clear();
}

void MyEdgeClass::draw(Mat& image, cv::Scalar color)
{
    line(image,pt1,pt1+v1*1000,Scalar(0,0,255));
    line(image,pt1,pt1-v1*1000,Scalar(255,0,0));
    char num[10];
    for(int i=0;i<targets.oldTargets.size();i++)
    {
        circle(image,targets.oldTargets.at(i),3,Scalar(0,255,0),-1);
        sprintf(num,"%d",i);
        putText(image,num,targets.oldTargets.at(i),CV_FONT_BLACK,1,Scalar(0,0,255));
    }

    if(EdgeSide==Edge_Right)
    {
        line(image,pt2,pt2+v2*1000,Scalar(0,0,255));
        line(image,pt2,pt2-v2*1000,Scalar(255,0,0));

        line(image,pt3,pt3+v3*1000,Scalar(0,0,255));
        line(image,pt3,pt3-v3*1000,Scalar(255,0,0));
    }
}

//triangle
MyTriClass::MyTriClass()
{

}

MyTriClass::MyTriClass(Point pt1,Point pt2,Point pt3)
{
    this->pt1=pt1;
    this->pt2=pt2;
}

MyTriClass::MyTriClass(vector<Point> pts)
{
    Vec4i line;
    fitLine(pts,line,CV_DIST_L2,0,0.01,0.01);
    this->v.x=line[0];
    this->v.y=line[1];
    this->pt1.x=line[2];
    this->pt1.y=line[3];
}

void MyTriClass::update(vector<Point> pts)
{
    //    cout<<(Mat)pts<<endl;
    Vec4f line;
    fitLine(pts,line,CV_DIST_HUBER,0,0.01,0.01);
    this->v.x=line[0];
    this->v.y=line[1];
    this->pt1.x=line[2];
    this->pt1.y=line[3];
}

void MyTriClass::draw(Mat& image, cv::Scalar color)
{
    double phi = atan2(this->v.y, this->v.x) + CV_PI / 2.0;
    double rho = this->pt1.y * this->v.x - this->pt1.x*this->v.y;
    if (phi < CV_PI/4. || phi > 3.*CV_PI/4.)// ~vertical line
    {
        cv::Point pt1(rho/cos(phi), 0);
        cv::Point pt2((rho - image.rows * sin(phi))/cos(phi), image.rows);
        cv::line( image, pt1, pt2, cv::Scalar(255), 1);
    }
    else
    {
        cv::Point pt1(0, rho/sin(phi));
        cv::Point pt2(image.cols, (rho - image.cols * cos(phi))/sin(phi));
        cv::line(image, pt1, pt2, color, 1);
    }
}

//targets
MyTargetVClass::MyTargetVClass(void)
{}

void MyTargetVClass::update(void)
{

}

//car
MyCarClass::MyCarClass()
{}

void drawBox(Mat& image,vector<Point2f> box)
{
    Point2f pts[4];
    if(box.size()==4)
        for(int i=0;i<4;i++)
        {
            line(image,box.at(i)+Point2f(100,0),box.at((i+1)%4)+Point2f(100,0),Scalar(0,0,255));
        }
}
void drawBox(Mat& image,vector<Point> box)
{
    if(box.size()==4)
        for(int i=0;i<4;i++)
        {
            line(image,box.at(i)+Point(100,0),box.at((i+1)%4)+Point(100,0),Scalar(0,0,255));
        }
}
Point MyCarClass::findDir(RotatedRect redBox,Point2f blueCenter)
{
    Point2f pts[4];
    redBox.points(pts);
    Point2f dir;
    Point2f tmpCenter;
    float minAngle=CV_PI;
    Point rlt;
    for(int i=0;i<4;i++)
    {
        //        cout<<"calc="<<calcLength(pts[i]-pts[(i+1)%4])<<endl;
        //        cout<<"theo="<<redBox.size.width<<endl;
        dir=pts[i]-pts[(i+1)%4];
        tmpCenter=(pts[i]+pts[(i+1)%4])/2.f;
        tmpCenter=tmpCenter-blueCenter;
        if(calcAngle(dir,tmpCenter)>CV_PI/4)
            dir*=-1;
        if(calcAngle(dir,tmpCenter)<minAngle)
        {
            minAngle=calcAngle(dir,tmpCenter);
            rlt=dir;
        }
    }
    return dir;
}

void MyCarClass::update(vector<Point> redContour, vector<Point> blueContour, MyEdgeClass Ledge, MyEdgeClass Redge)
{
    Point2f redCenter,blueCenter;
    redCenter=minAreaRect(redContour).center;
    redBox=minAreaRect(redContour);
    blueBox=minAreaRect(blueContour);
    //    cout<<redBox.angle<<endl;
    blueCenter=minAreaRect(blueContour).center;
    //    blueContour.insert(blueContour.end(),redContour.begin(),redContour.end());
    for(int i=0;i<redContour.size();i++)
    {
        blueContour.push_back(redContour.at(i));
    }
    //    v=findDir(redBox,blueCenter);

    //    cout<<"edge.v="<<Ledge.v1<<endl;
    //    cout<<"angleL="<<calcAngle(v,Ledge.v1)<<endl;
    //    cout<<"angleR1="<<calcAngle(v,Redge.v1)<<endl;
    //    cout<<"angleR2="<<calcAngle(v,Redge.v2)<<endl;
    //    cout<<"angleHoriz="<<calcAngle(v,Point(-1,0))<<endl;

    //    cout<<"cvClass.garage.at(0).size()"<<cvClass.garage.at(0).size()<<endl;
    //    for(int n=0;n<6;n++)
    //    {
    //        for(int i=0;i<cvClass.garage.at(0).size();i++)
    //        {
    //            for(int j=i+1;j<cvClass.garage.at(0).size();j++)
    //            {
    //                if(cvClass.garage.at(0).at(i).x>cvClass.garage.at(0).at(j).x)
    //                {
    //                    Point2f tmp;
    //                    tmp=cvClass.garage.at(0).at(i);
    //                    cvClass.garage.at(0).at(i)=cvClass.garage.at(0).at(j);
    //                    cvClass.garage.at(0).at(j)=tmp;
    //                }
    //            }
    //        }
    //    }

    vector<Point3f> objPts;
    objPts.push_back(Point3f(1430,0,0));
    objPts.push_back(Point3f(1430,1430,0));
    objPts.push_back(Point3f(0,1430,0));
    objPts.push_back(Point3f(0,0,0));
    vector<Point2f> imgPts;
    vector<Point> convex;
    convexHull(blueContour,convex);
    //    for(int i=1;imgPts.size()!=4&&i<6;i++)
    //    {
    //        imgPts.clear();
    //        approxPolyDP(convex,imgPts,i,true);
    //    }
    approxPolyDP(convex,imgPts,7,true);
    v=redBox.center-blueBox.center;
    pos=calcCenter(blueContour)+Point2f(100,0);
//        cout<<imgPts.size()<<endl;
    try
    {
        if(imgPts.size()==4&&pos.y>90)
        {
            Mat rvec,tvec;
            float fx,fy,px,py;
            fx=1200.f;
            fy=1200.f;
            px=320;
            py=240;
            Matx33f cameraMatrix(
                        fx, 0  , px,
                        0 , fy , py,
                        0 , 0  , 1
                        );
            cv::Vec4f distParam(0,0,0,0); // all 0?
            cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
            //            cout<<"rvec"<<rvec<<endl;
            //            cout<<"tvec"<<tvec<<endl;
            vector<Point3f> center;
            //            center.push_back(Point3d(715,715,1080));
            center.push_back(Point3f(1430,0,1080));
            center.push_back(Point3f(1430,1430,1080));
            center.push_back(Point3f(0,1430,1080));
            center.push_back(Point3f(0,0,1080));
            realBox.clear();
            //            if(rvec.at<float>(0)>0) rvec.at<float>(0)*=-1;
            //            if(rvec.at<float>(1)<0) rvec.at<float>(1)*=-1;
            //            cout<<"rvec"<<rvec<<endl;
            projectPoints(center,rvec,tvec,cameraMatrix,distParam,realBox);
//            cout<<cvClass.targetBox<<endl;
//            projectPoints(cvClass.targetBox,rvec,tvec,cameraMatrix,distParam,cvClass.targetRealBox);
            //////calc v//////
//            float maxLen=-1;
//            Point2f maxVec;
//            for(int i=0;i<4;i++)
//            {
//                if(calcLength(realBox.at(i)-realBox.at((i+1)%4))>maxLen)
//                {
//                    maxLen=calcLength(realBox.at(i)-realBox.at((i+1)%4));
//                    maxVec=realBox.at(i)-realBox.at((i+1)%4);
//                }
//            }
////            cout<<maxVec<<"  ";
////            cout<<calcCos(maxVec,v)<<endl;
//            if((calcCos(maxVec,v)-1)>-0.2)
//            {
//                v=maxVec;
//            }
//            else if((calcCos(maxVec,v)+1)<0.2)
//            {
//                v=maxVec;
//                v*=-1;
//            }
//            else if(fabs(calcCos(maxVec,v))<0.8)
//            {
//                Point2f tmp=Point2f(maxVec.y,maxVec.x);
//                {
//                    if((calcCos(tmp,v)-1)>-0.3)
//                    {
////                        cout<<"test1"<<endl;
//                        v=tmp;
//                    }
//                    else if((calcCos(tmp,v)+1)<0.3)
//                    {
////                        cout<<"test2  "<<maxVec<<" "<<tmp<<endl;
//                        v=tmp;
//                        v.y*=-1;
//                        v.x*=-1;
////                        cout<<tmp<<" "<<v<<endl;
//                    }
//                }
//            }
            /////////////////

            pos=calcCenter(realBox)+Point2f(100,0);
            Point2f tmpCenter=calcCenter(blueContour)+Point2f(100,0);
            if(pos.y<tmpCenter.y)
            {
                pos=tmpCenter-pos+tmpCenter;
            }
        }
        Point2f pt[2];
        float n=0,x;
        n=(pos.y-Ledge.pt1.y)/Ledge.v1.y;
        x=Ledge.pt1.x+n*Ledge.v1.x;
        pt[0]=Point(x,pos.y);
        n=0;
        n=(pos.y-Redge.pt1.y)/Redge.v1.y;
        x=Redge.pt1.x+n*Redge.v1.x;
        pt[1]=Point(x,pos.y);
        x2Edge=350/fabs(pt[1].x-pt[0].x)*fabs(pos.x-pt[0].x);
        //    cout<<"x2Edge="<<x2Edge<<endl;

        //perspectiveTransform();
        //            cout<<rvec*(Mat)imgPts+tvec<<endl;
//        Mat image(Size(640,480),CV_8UC3);
//        Ledge.draw(image,Scalar(0,0,255));
//        Redge.draw(image,Scalar(0,0,255));

//        imshow("lines",image);
//        waitKey(1);
        if(cvClass.targetID<4)
        {
            tmpV=cvClass.chosenTarg.first-cvClass.chosenTarg.second;
            n=(pos.x-cvClass.chosenTarg.second.x)/tmpV.x;
            float y;
            y=cvClass.chosenTarg.second.y+n*tmpV.y;
            cvClass.dest=Point(pos.x,y);
        }
        pthread_mutex_lock(&mutex);
//        serialPort.Msg[serialPort.MISSION_STAGE]=Steering2;
//        cvClass.targetID=2;
//        cout<<"x="<<x2Edge<<endl;
//        cout<<"targetID="<<cvClass.targetID<<endl;
//        yaw=calcAngle(v,tmpV)/CV_PI*180;
//        cout<<"YAW="<<yaw<<endl;+
        switch((int)serialPort.Msg[serialPort.MISSION_STAGE])
        {
        ifStandby=false;
        case Standby:
            ifStandby=true;
            break;
        case Steering0:
            cout<<"Steering0" <<"Car_YAW="<<yaw<<endl;
            yaw=calcAngle(v,preV)/CV_PI*180;
            serialPort.SerialSendMsg(serialPort.TARG_ID,cvClass.targetID);
            serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
            break;
        case Reversing:
            //        waitKey(0);
            cout<<"Reversing" <<"Car_X="<<x2Edge<<"Car_Y="<<pos.y<<" target="<<cvClass.chosenTarg.second<<" targetID="<<cvClass.targetID<<endl;
            serialPort.SerialSendMsg(serialPort.Car_X,x2Edge);
            serialPort.SerialSendMsg(serialPort.Car_Y,pos.y);
            break;
        case Steering1:
            cout<<"Steering1" <<"Car_YAW="<<yaw<<endl;
            yaw=calcAngle(v,Redge.v1)/CV_PI*180;
            y2Targ=cvClass.dest.y-pos.y;
            serialPort.SerialSendMsg(serialPort.P0_Y_OFS,y2Targ);
            serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
            break;
        case Moving1:
            cout<<"Moving1" <<" Car_X="<<x2Edge<<" P0_Y_OFS"<<y2Targ<<" target="<<cvClass.chosenTarg.second<<endl;
            y2Targ=cvClass.dest.y-pos.y;
            serialPort.SerialSendMsg(serialPort.Car_X,x2Edge);
            serialPort.SerialSendMsg(serialPort.P0_Y_OFS,y2Targ);
            serialPort.SerialSendMsg(serialPort.TARG_ID,cvClass.targetID);
//            serialPort.SerialSendMsg(serialPort.P0_Y_OFS,300);
            break;
        case Steering2:
            if(cvClass.targetID==3||cvClass.targetID==0)
            {
                yaw=calcAngle(v,Point2f(Redge.v3.y,Redge.v3.x*-1))/CV_PI*180;
//                serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
//                break;
            }
            else if(cvClass.targetID==4)
            {
                yaw=-calcAngle(v,Point2f(Redge.v3.y,Redge.v3.x*-1))/CV_PI*180;
            }
            else
            {
//                cout<<"Steering2" <<"Car_YAW="<<yaw<<endl;
                yaw=calcAngle(v,tmpV)/CV_PI*180;
            }
//            cout<<v<<" "<<Redge.v2.y<<" "<<Redge.v2.x*-1<<" ";
            cout<<"Steering2" <<"Car_YAW="<<yaw<<endl;
            serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
            break;
        case Moving2:
            cout<<"Moving2" <<"P0_X_OFS="<<x2Edge<<"P0_Y_OFS"<<y2Targ<<endl;
            y2Targ=cvClass.dest.y-pos.y;
            if(cvClass.targetID==3||cvClass.targetID==0)
            {
                yaw=calcAngle(v,Point2f(Redge.v3.y,Redge.v3.x*-1))/CV_PI*180;
//                serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
//                break;
            }
            else if(cvClass.targetID==4)
            {
                yaw=-calcAngle(v,Point2f(Redge.v3.y,Redge.v3.x*-1))/CV_PI*180;
            }
            else
            {
//                cout<<"Steering2" <<"Car_YAW="<<yaw<<endl;
                yaw=calcAngle(v,tmpV)/CV_PI*180;
            }            serialPort.SerialSendMsg(serialPort.Car_X,x2Edge);
//            serialPort.SerialSendMsg(serialPort.P0_Y_OFS,y2Targ);
            serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
            break;
        case Steering3:
            cout<<"Steering3" <<"Car_YAW="<<yaw<<" "<<cvClass.targetID<<endl;
            if(cvClass.targetID==1||cvClass.targetID==2)
                yaw=calcAngle(v,Redge.v2)/CV_PI*180;
            else if(cvClass.targetID==0||cvClass.targetID==3)
                yaw=calcAngle(v,tmpV)/CV_PI*180;
            serialPort.SerialSendMsg(serialPort.TARG_ID,cvClass.targetID);
            serialPort.SerialSendMsg(serialPort.Car_YAW,yaw);
            break;
        }
        pthread_mutex_unlock(&mutex);
    }
    catch(...)
    {
        pthread_mutex_unlock(&mutex);
        cout<<"error"<<endl;
    }
}

void MyCarClass::draw(Mat &image)
{
    dist=Mat::zeros(Size(350,250),CV_8UC3);
    circle(image,pos,3,Scalar(255,255,0),-1);
    line(image,pos,pos+v*2,Scalar(255,255,0));
//    Mat dist(Size(640,480),CV_8UC3);
    drawBox(image,realBox);
    Mat tmp=image(Rect(Point(0,90),Point(500,480)));
//    drawBox(tmp,cvClass.targetBox);
//    drawBox(dist,realBox);
//    imshow("dist",dist);
    circle(image,redBox.center+Point2f(100,0),3,Scalar(0,0,255),-1);
    circle(image,blueBox.center+Point2f(100,0),3,Scalar(255,0,0),-1);

    Mat warp_mat(2,3,CV_32FC1);
    vector<Point2f> src;
    vector<Point2f> dst;
    vector<Point2f> warp_dst;
    for(int i=0;i<cvClass.targetBox.size();i++)
        src.push_back(Point2f(cvClass.targetBox.at(i).x,cvClass.targetBox.at(i).y+90));
   try
    {
    if(cvClass.targetID==0||cvClass.targetID==3)
    {
        dst.push_back(Point2f(0,0));
        dst.push_back(Point2f(0,250));
        dst.push_back(Point2f(350,250));
        dst.push_back(Point2f(350,0));
    }
    else
    {
        dst.push_back(Point2f(0,0));
        dst.push_back(Point2f(0,250));
        dst.push_back(Point2f(350,250));
        dst.push_back(Point2f(350,0));
    }
    if(src.size()==4&&dst.size()==4)
        warp_mat=getPerspectiveTransform(src,dst);
    cout<<"src:"<<src<<endl<<"dst;"<<dst<<endl<<"warp:"<<warp_mat<<endl;
//    cout<<warp_mat<<endl;
    if(realBox.size()==4)
    {
        try
        {
        vector<Point2f> rlt;
        perspectiveTransform(realBox,rlt,warp_mat);
        cout<<rlt<<endl;
        for(int i=0;i<rlt.size();i++)
        {
            rlt.at(i).x=rlt.at(i).x>0?rlt.at(i).x:-rlt.at(i).x;
            rlt.at(i).y=rlt.at(i).y>0?rlt.at(i).y:-rlt.at(i).y;
            line(dist,rlt.at(i),rlt.at((i+1)%rlt.size()),Scalar(0,0,255));
        }
        float maxX=0,minX=999,maxY=0,minY=999;
        for(int i=0;i<rlt.size();i++)
        {
            float x,y;
            x=(rlt.at(i).x+rlt.at((i+1)%rlt.size()).x)/2;
            y=(rlt.at(i).y+rlt.at((i+1)%rlt.size()).y)/2;
            if(x>maxX)
                maxX=x;
            if(x<minX)
                minX=x;
            if(y>maxY)
                maxY=y;
            if(y<minY)
                minY=y;
        }
//        cout<<minX<<" "<<maxX<<" "<<minY<<" "<<maxY<<endl;
        if(minY<250&&minY>0&&minX<350&&minX>0&&maxY<250&&maxY>0&&maxX<350&&maxX>0)
        {
            char num[20];
            sprintf(num,"%-5.2f",350-maxX);
            putText(dist,num,Point(310,120),CV_FONT_NORMAL,0.5,Scalar(255,255,255));
            sprintf(num,"%-5.2f",minX);
            putText(dist,num,Point(0,120),CV_FONT_NORMAL,0.5,Scalar(255,255,255));
            sprintf(num,"%-5.2f",250-maxY);
            putText(dist,num,Point(165,230),CV_FONT_NORMAL,0.5,Scalar(255,255,255));
            sprintf(num,"%-5.2f",minY);
            putText(dist,num,Point(165,10),CV_FONT_NORMAL,0.5,Scalar(255,255,255));
        }
        if(ifStandby)
        {
            putText(dist,"Standby",Point(165,130),CV_FONT_NORMAL,0.5,Scalar(255,255,255));
        }
        dist.copyTo(cvClass.gui(Rect(Point(640,230),Point(990,480))));
        }
        catch(...)
        {
            cout<<"fuck you"<<endl;
        }
    }
    }
    catch(...)
    {
        cout<<"fuck you twice"<<endl;
    }

//    for(int i=0;i<realBox.size();i++)
//    {
//        Point2f rlt;
////        float x=realBox.at(i).x;
////        float y=realBox.at(i).y;
////        rlt.x=warp_mat.at<float>(0,0)*x+warp_mat.at<float>(0,1)*y+warp_mat.at<float>(0,2);
////        rlt.y=warp_mat.at<float>(1,0)*x+warp_mat.at<float>(1,1)*y+warp_mat.at<float>(1,2);
////        float t=warp_mat.at<float>(2,0)*x+warp_mat.at<float>(2,1)*y+warp_mat.at<float>(2,2);
////        rlt.x/=t;
////        rlt.y/=t;
//        cout<<x<<" "<<y<<endl<<rlt<<endl;

////        Mat srcMat(3,1,CV_32FC1);
////        srcMat.at<float>(0,0)=realBox.at(i).x;
////        srcMat.at<float>(1,0)=realBox.at(i).y;
////        srcMat.at<float>(2,0)=1;
////        cout<<warpMat*srcMat<<endl;
//    }
}

Point2f MyCarClass::calcCenter(vector<Point> pts)
{
    Moments mu=moments(pts);
    return  Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
}

Point2f MyCarClass::calcCenter(vector<Point2f> pts)
{
    Moments mu=moments(pts);
    return  Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
}
