#include "cvclass.h"
#define TRUE 1
#define FALSE 0
#define IMAGEWIDTH 640 
#define IMAGEHEIGHT 480 
CVClass cvClass;
static int fd;
CVClass::CVClass()
{
}

CVClass::CVClass(char *filename)
{
    name=filename;
}

void CVClass::setDevice(char *filename)
{
    name=filename;
}

CVClass::CVClass(int device)
{
}

int CVClass::setCameraParam(__u32 parameter,__s32 value)
{
    int io_rel=0;
    struct v4l2_control camera_ctrl;
    memset(&camera_ctrl,0,sizeof(struct v4l2_control));
    camera_ctrl.id=parameter;
    camera_ctrl.value=value;
    io_rel=ioctl(fd,VIDIOC_S_CTRL,&camera_ctrl);
    if(io_rel!=0)
    {
        cout<<"Unable to set the value of parameter"<<endl;
        return -1;
    }
    return 0;
}

int CVClass::init_v4l2()
{
    if ((fd = open(name, O_RDWR)) == -1){
        printf("Opening video device error\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1){
            printf("unable Querying Capabilities\n");
            return FALSE;
    }
    else
/*
    {
    printf( "Driver Caps:\n"
            "  Driver: \"%s\"\n"
            "  Card: \"%s\"\n"
            "  Bus: \"%s\"\n"
            "  Version: %d\n"
            "  Capabilities: %x\n",
            cap.driver,
            cap.card,
            cap.bus_info,
            cap.version,
            cap.capabilities);

    }

    if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE){
        printf("Camera device %s: support capture\n",FILE_VIDEO1);
    }
    if((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING){
        printf("Camera device %s: support streaming.\n",FILE_VIDEO1);
    }
*/
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format: \n");
    while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc) != -1){
        printf("\t%d. %s\n",fmtdesc.index+1,fmtdesc.description);
        fmtdesc.index++;
    }
    //set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMAGEWIDTH;
    fmt.fmt.pix.height = IMAGEHEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; //*************************V4L2_PIX_FMT_YUYV****************
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1){
        printf("Setting Pixel Format error\n");
        return FALSE;
    }
    if(ioctl(fd,VIDIOC_G_FMT,&fmt) == -1){
        printf("Unable to get format\n");
        return FALSE;
    }
//        else

/*        {
        printf("fmt.type:\t%d\n",fmt.type);
        printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF,(fmt.fmt.pix.pixelformat >> 8) & 0xFF,\
               (fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        printf("pix.height:\t%d\n",fmt.fmt.pix.height);
        printf("pix.field:\t%d\n",fmt.fmt.pix.field);
    }
*/
/*
    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 100;
    setfps.parm.capture.timeperframe.denominator = 100;
    printf("init %s is OK\n",FILE_VIDEO1);
*/
    return TRUE;
}

int CVClass::v4l2_grab()
{
    //struct v4l2_requestbuffers req = {0};
    //4  request for 4 buffers
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1)
    {
        printf("Requesting Buffer error\n");
        return FALSE;
    }
    //5 mmap for buffers
    buffer = (uchar*)malloc(req.count * sizeof(*buffer));
    if(!buffer){
        printf("Out of memory\n");
        return FALSE;
    }
    unsigned int n_buffers;
    for(n_buffers = 0;n_buffers < req.count; n_buffers++){
    //struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;
    if(ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1){
        printf("Querying Buffer error\n");
        return FALSE;
        }

    buffer = (uchar*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

    if(buffer == MAP_FAILED){
        printf("buffer map error\n");
        return FALSE;
        }
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);
    }
    //6 queue
    for(n_buffers = 0;n_buffers <req.count;n_buffers++){
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd,VIDIOC_QBUF,&buf)){
            printf("query buffer error\n");
            return FALSE;
        }
    }
    //7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd,VIDIOC_STREAMON,&type) == -1){
        printf("stream on error\n");
        return FALSE;
    }
    return TRUE;
}

Mat CVClass::getImage(void)
{
    Mat img;
    ioctl(fd,VIDIOC_DQBUF,&buf);
    buf.index = 0;
    img=imdecode(cv::Mat(Size(640,480),CV_8UC3,(void*)buffer),CV_LOAD_IMAGE_COLOR );
    ioctl(fd,VIDIOC_QBUF,&buf);
    return img;
}

float calcLength(Point2f vec)
{
    return (float)sqrt(vec.x*vec.x+vec.y*vec.y);
}

void CVClass::reset()
{
    isReady=false;
    ifUpdateEdge=false;
    targetID=-1;
    garage.clear();
    targets.curTargets.clear();
    targets.oldTargets.clear();
    targets.isGood=false;
    targets.TargGood[0]=false;
    targets.TargGood[1]=false;
    targets.TargGood[2]=false;
    targets.TargGood[3]=false;
    Ledge.targets.curTargets.clear();
    Ledge.targets.oldTargets.clear();
    Ledge.targets.isGood=false;
    Ledge.targets.TargGood[0]=false;
    Ledge.targets.TargGood[1]=false;
    Ledge.targets.TargGood[2]=false;
    Ledge.targets.TargGood[3]=false;
    Ledge.isReady=false;
    Redge.targets.curTargets.clear();
    Redge.targets.oldTargets.clear();
    Redge.targets.isGood=false;
    Redge.targets.TargGood[0]=false;
    Redge.targets.TargGood[1]=false;
    Redge.targets.TargGood[2]=false;
    Redge.targets.TargGood[3]=false;
    Redge.isReady=false;
}
int bT=100,gT=100,rT=100;
int dbT=100,dgT=100,drT=100;
int CVClass::findBlack(Mat& src)
{
	cout<<"running"<<endl;
    try
    {
        for(int i=0;i<src.rows;i++)
        {
            uchar* data=src.ptr<uchar>(i);
            for(int j=0;j<src.cols*3;j+=3)
            {
                if(/*i<50&&*/data[j]<120&&data[j+1]>data[j]*1.3&&data[j+2]>data[j]*1.3){       //Unreliable
				if(data[j+1]<data[j]*1.5&&data[j+2]<data[j]*1.5)
				{
                    data[j]=255;
                    data[j+1]=255;
                    data[j+2]=255;
                }
//                else if(i>=50&&data[j]<dbT&&data[j+1]<dgT&&data[j+2]<drT)       //Unreliable
//                {
//                    data[j]=255;
//                    data[j+1]=255;
//                    data[j+2]=255;
//                }
				}
                else
                {
                    data[j]=0;
                    data[j+1]=0;
                    data[j+2]=0;
                }
            }
        }
        imshow("tmp",src);
		return 0;
    }
    catch(...)
    {
        cout<<"color change error"<<endl;
        return 1;
    }
}
bool cflag=false;
void colorCallBack(int event,int x,int y,int flags,void* param)
{
    Mat* img=(Mat*)param;
    Mat color=Mat::zeros(Size(200,20),CV_8UC3);
    if(event==EVENT_LBUTTONDOWN)
    {
        cflag=true;
//        cout<<"down "<<flag<<endl;
    }
    if(event==EVENT_LBUTTONUP)
    {
//        cout<<x<<" "<<y<<endl;
//        cout<<flag<<endl;
        if(cflag==true)
        {
            char num[30];
            sprintf(num,"%d,%d,%d",(int)img->at<Vec3b>(y,x)[0],(int)img->at<Vec3b>(y,x)[1],(int)img->at<Vec3b>(y,x)[2]);
            putText(color,num,Point(0,20),CV_FONT_NORMAL,1,Scalar(255,255,255));
//            cout<<(int)img->at<Vec3b>(y,x)[0]<<" "<<(int)img->at<Vec3b>(y,x)[1]<<" "<<(int)img->at<Vec3b>(y,x)[2]<<endl;
        }
        cflag=false;
    }
    imshow("color",color);
}

int rbTT=80,rgTT=80,rrTT=100;
int bbTT=100,bgTT=70,brTT=70;
int CVClass::findCar(Mat src)
{
    Mat frame;
    Mat red,blue;
    src.copyTo(frame);
    red.create(frame.size(),CV_8UC1);
    blue.create(frame.size(),CV_8UC1);
    try
    {

        for(int i=0;i<frame.rows;i++)
        {
            uchar* data=frame.ptr<uchar>(i);
            uchar* redData=red.ptr<uchar>(i);
            uchar* blueData=blue.ptr<uchar>(i);
            for(int j=0;j<frame.cols*3;j+=3)
            {
                if(data[j]<rbTT&&data[j+1]<rgTT&&data[j+2]>rrTT)       //Unreliable
                {
                    redData[j/3]=255;
                    blueData[j/3]=0;
                }
                else if(data[j]>bbTT&&data[j+1]<bgTT&&data[j+2]<brTT)
                {
                    blueData[j/3]=255;
                    redData[j/3]=0;
                }
                else
                {
                    blueData[j/3]=0;
                    redData[j/3]=0;
                }
            }
        }

        //dilate(frame,frame,element);
        vector<vector<Point> > redContours;
        vector<vector<Point> > blueContours;
        findContours(red,redContours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        cvtColor(red,red,COLOR_GRAY2BGR);
        float redMaxArea=-1;
        int redMaxId=-1;
        for(int i=0;i<redContours.size();i++)
        {
            float area=contourArea(redContours.at(i));
            if(area>redMaxArea)
            {
                redMaxArea=area;
                redMaxId=i;
            }
        }
        drawContours(red,redContours,redMaxId,Scalar(0,0,255),-1);
        findContours(blue,blueContours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        cvtColor(blue,blue,COLOR_GRAY2BGR);
        float blueMaxArea=-1;
        int blueMaxId=-1;
        for(int i=0;i<blueContours.size();i++)
        {
            float area=contourArea(blueContours.at(i));
            if(area>blueMaxArea)
            {
                blueMaxArea=area;
                blueMaxId=i;
            }
        }

        drawContours(blue,blueContours,blueMaxId,Scalar(0,0,255),-1);
        imshow("red",red);
        imshow("blue",blue);
        waitKey(1);
        try
        {
            car.update(redContours.at(redMaxId),blueContours.at(blueMaxId),Ledge,Redge);
            car.draw(frame);
        }
        catch(...)
        {
            cout<<"fuck "<<endl;
        }

        //imshow("frame",frame);
    }
    catch(...)
    {
//        cout<<"findCar error"<<endl;
        return 1;
    }
}

void CVClass::sortPts(vector<Point>& pts)
{
    for(int n=0;n<6;n++)
    {
        for(int i=0;i<pts.size();i++)
        {
            for(int j=i+1;j<pts.size();j++)
            {
                if(pts.at(i).x>pts.at(j).x)
                {
                    Point2f tmp;
                    tmp=pts.at(i);
                    pts.at(i)=pts.at(j);
                    pts.at(j)=tmp;
                }
            }
        }
    }
}


void CVClass::stateCheck()
{
    if(Ledge.isReady&&Redge.isReady)
    {
        isReady=true;
        serialPort.SerialSendMsg(serialPort.START,1);
        serialPort.SerialSendMsg(serialPort.CV_READY,1);
    }
}

float CVClass::calcTriArea(vector<Point> pts)
{
    if(pts.size()!=3) return -1;
    float a[3];
    for(int i=0;i<3;i++)
    {
        a[i]=calcLength(pts.at(i)-pts.at((i+1)%3));
    }
    float s=(a[0]+a[1]+a[2])/2.f;
    return sqrt(s*(s-a[0])*(s-a[1])*(s-a[2]));
}
Mat tmpI(Size(640,480),CV_8UC1);
bool CVClass::checkObs(vector<Point> pts,Mat& image)
{
    int blackCnt=0;
    try
    {
        float maxX=0,minX=999,maxY=0,minY=999;
        for(int i=0;i<pts.size();i++)
        {
            if(pts.at(i).x>maxX)
            {
                maxX=pts.at(i).x;
            }
            if(pts.at(i).x<minX)
            {
                minX=pts.at(i).x;
            }
            if(pts.at(i).y>maxY)
            {
                maxY=pts.at(i).y;
            }
            if(pts.at(i).y<minY)
            {
                minY=pts.at(i).y;
            }
        }
//        cout<<minX<<" "<<maxX<<" "<<minY<<" "<<maxY<<endl;
    Moments mu=moments(pts);
    Point center=Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
    for(int i=0;i<pts.size();i++)
    {
        pts.at(i)=center+0.9*(pts.at(i)-center);
    }
    vector<Point> tri1,tri2;
    tri1.push_back(pts.at(0));
    tri1.push_back(pts.at(1));
    tri1.push_back(pts.at(3));
    tri2.push_back(pts.at(1));
    tri2.push_back(pts.at(2));
    tri2.push_back(pts.at(3));
    float area4=calcTriArea(tri1)+calcTriArea(tri2);
    for(int i=minY+5;i<maxY-5;i++)
    {
//        cout<<"test"<<endl;
        uchar* data=image.ptr<uchar>(i);
        uchar* data1=tmpI.ptr<uchar>(i);
        for(int j=minX+5;j<maxX-5;j++)
        {
//            float area[4];
//            vector<Point> sTri1,sTri2,sTri3,sTri4;
//            sTri1.push_back(Point(j/3,i));
//            sTri1.push_back(pts.at(0));
//            sTri1.push_back(pts.at(1));
//            area[0]=calcTriArea(sTri1);

//            sTri2.push_back(Point(j/3,i));
//            sTri2.push_back(pts.at(1));
//            sTri2.push_back(pts.at(2));
//            area[1]=calcTriArea(sTri2);

//            sTri3.push_back(Point(j/3,i));
//            sTri3.push_back(pts.at(2));
//            sTri3.push_back(pts.at(3));
//            area[2]=calcTriArea(sTri3);

//            sTri4.push_back(Point(j/3,i));
//            sTri4.push_back(pts.at(3));
//            sTri4.push_back(pts.at(0));
//            area[3]=calcTriArea(sTri4);

//            if(fabs(area4-area[0]-area[1]-area[2]-area[3])<3)
//            circle(image,Point(j,i),3,Scalar(0,0,255),-1);
//            cout<<calcLength(Point(j,i)-center)<<endl;
            if(calcLength(Point(j,i)-center)<30)

            {
//                cout<<(int)data[j]<<endl;
                if(data[j]>0)
                {
//                    data1[j]=155;
                    blackCnt++;
                }
            }
        }
//        imshow("shit",tmpI);
    }
    }
    catch(...)
    {
        cout<<"check error"<<endl;
    }
//    imshow("image",image);
//    cout<<blackCnt<<endl;
    if(blackCnt>=200)
        return false;
    else return true;
}

int CVClass::CVProcess(void)
{
    time=static_cast<double>(getTickCount());
    Mat fframe,frame;
//    cout<<"new frame"<<endl;
//    vector<Point> pts2fit;
    vector< vector<Point> > squares;
    vector<RotatedRect> bBox;
    contoursImg.setTo(0);
    fframe=getImage();
    imshow("colortest",fframe);
    setMouseCallback("colortest",colorCallBack,&fframe);
//    waitKey(1);
//    return 0;
    frame=fframe(Rect(beginPt,Point(500,480)));
    findCar(fframe(Rect(beginPtC,Point(500,480))));
    garage.clear();
    stateCheck();

    if(ifUpdateEdge)
    {
    //    imshow("car",fframe);

        GaussianBlur(frame,frame,Size(3,3),0.5);
//        if(isReady)

         findBlack(frame);                               //Unreliable


        cvtColor(frame,frame,COLOR_BGR2GRAY);
//        if(!isReady)
//            adaptiveThreshold(frame,frame,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,7,7);
    //    Canny(frame,frame,1,5);
        dilate(frame,frame,element);
        Mat img2check;
        frame.copyTo(img2check);
        imshow("thresh",frame);
        vector< vector<Point> > contours;
        vector< vector<Point> > polyContours;
        findContours(frame,contours,RETR_LIST,CHAIN_APPROX_NONE);
        cvtColor(frame,frame,COLOR_GRAY2BGR);
        contoursImg.create(Size(frame.cols,frame.rows),CV_8UC3);
        polyContours.resize(contours.size());
        vector< vector<Point> > sqr2check;
        for(int i=0;i<contours.size();i++)
        {
            approxPolyDP(contours.at(i),polyContours.at(i),20,true);
            if(polyContours.at(i).size()==4)
            {
                Point2f vec1=polyContours.at(i).at(0)-polyContours.at(i).at(1);
                Point2f vec2=polyContours.at(i).at(3)-polyContours.at(i).at(2);
                if(calcLength(vec2-vec1)<50)
                    if(contourArea(polyContours.at(i))>2000&&contourArea(polyContours.at(i))<100000)
                    {
                        RotatedRect box=minAreaRect(polyContours.at(i));
                        bBox.push_back(box);
                        //                    cout<<contourArea(contours.at(i))<<endl;
//                        if(checkObs(polyContours.at(i),img2check))
//                        {
                            sqr2check.push_back(polyContours.at(i));
                            //                        cout<<"angle="<<box.angle<<endl;
//                            polyContours.erase(polyContours.begin()+i);
//                        }
                    }
            }
            //            cout<<polyContours.at(i).size()<<endl;
        }

        for(int i=0;i<sqr2check.size();i++)
        {
            bool rlt=checkObs(sqr2check.at(i),img2check);
            if(rlt)
            {
              squares.push_back(sqr2check.at(i));
            }
        }
//                imshow("shit",tmpI);
//        cout<<squares.size()<<endl;
//        cout<<endl;
        for(int i=0;i<squares.size();i++)
            drawContours(contoursImg,squares,i,Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)));

//        for(int i=0;i<polyContours.size();i++)
//        {
//            Moments mu=moments(polyContours.at(i));
//            Point center=Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
//            circle(contoursImg,center,3,Scalar(0,255,0),-1);
//            for(int j=0;j<squares.size();j++)
//            {
//                Moments mu=moments(squares.at(j));
//                Point center2=Point2f(mu.m10/mu.m00 , mu.m01/mu.m00);
//                if(calcLength(center-center2)<50)
//                {
//                    squares.erase(squares.begin()+j);
//                    polyContours.erase(polyContours.begin()+i);
//                    break;
//                }
//            }
//        }

        if(squares.size()>=2)
        {
            vector< pair<Point,int> > centersID(squares.size());
            for(int i=0;i<squares.size();i++)
            {
                for(int j=0;j<4;j++)
                {
                    centersID.at(i).first+=squares.at(i).at(j);
                }
                centersID.at(i).first/=4;
                centersID.at(i).second=i;
            }
            for(int n=0;n<6;n++)
            {
                for(int i=0;i<squares.size();i++)
                {
                    for(int j=i+1;j<squares.size();j++)
                    {
                        if(centersID.at(i).first.y>centersID.at(j).first.y)
                        {
                            pair<Point,int> tmp;
                            tmp=centersID.at(i);
                            centersID.at(i)=centersID.at(j);
                            centersID.at(j)=tmp;
                        }
                    }
                }
            }
            for(int i=0;i<squares.size();i++)
            {
                garage.push_back(squares.at(centersID.at(i).second));
            }

            if(garage.size()==4)
            {
                oldGarage=garage;
            }
//////////////////////////////////////////////////////////////
            if(ifChooseTarg)
            {
                chosenTarg=chooseTarg();
                if(chosenTarg.first.x!=-1)
                    ifChooseTarg=false;
            }
/////////////////////////////////////////////////////////////
            Mat fuck=fframe(Rect(beginPt,Point(500,480)));
            for(int i=0;i<garage.size();i++)
            {
                    drawContours(fuck,garage,i,Scalar(70*(i+3),0,70*(-i+3)),3);
            }
//            vector<vector
            drawContours(fuck,vector<vector<Point> >(1,targetBox),0,Scalar(0,255,0),3);
            for(int i=0;i<squares.size();i++)
            {
                sortPts(garage.at(i));
                Ledge.targets.curTargets.push_back(((garage.at(i).at(0)+garage.at(i).at(1))/2+beginPt));
//                Redge.targets.curTargets.push_back(((garage.at(i).at(2))+Point(100,100)));
//                Redge.targets.curTargets.push_back(((garage.at(i).at(3))+Point(100,100)));
                Redge.targets.curTargets.push_back(((garage.at(i).at(2)+garage.at(i).at(3))/2+beginPt));

                //            circle(contoursImg,findLeftPt(garage.at(i)),5,Scalar(0,0,255),-1);
                drawContours(contoursImg,garage,i,Scalar(50*i+50,50*i+50,50*i+50));
            }
            Ledge.update();
            Redge.update();
            //        waitKey(0);
        }
        Ledge.draw(fframe,Scalar(0,0,255));
        Redge.draw(fframe,Scalar(0,0,255));
//        for(int y=0;y<480;y++)
//        {
//            Point2f pt[2];
//            float n=0,x;
//            n=(y-Ledge.pt1.y)/Ledge.v1.y;
//            x=Ledge.pt1.x+n*Ledge.v1.x;
//            pt[0]=Point(x,y);
//            n=0;
//            n=(y-Redge.pt1.y)/Redge.v1.y;
//            x=Redge.pt1.x+n*Redge.v1.x;
//            pt[1]=Point(x,y);
////            x2Edge=350/fabs(pt[1].x-pt[0].x)*fabs(pos.x-pt[0].x);
//            x=fabs(pt[1].x-pt[0].x)/350*500+pt[0].x;
////            cout<<x<<" "<<y<<endl;
//            circle(fframe,Point(x,y),1,Scalar(0,0,255));
//        }
        circle(fframe,chosenTarg.first,7,Scalar(0,255,0));
        circle(fframe,chosenTarg.second,7,Scalar(0,255,0));
        circle(fframe,dest,7,Scalar(0,255,0));
        circle(fframe,home,4,Scalar(0,255,0));
        car.draw(fframe);
        imshow("black",contoursImg);
    }

    try
    {

//        if(isReady)
//            waitKey(100);
//        else if(!isReady)
        //    cout<<squares.size()<<endl;
        //    rectangle(fframe,Rect(Point(100,0),Point(500,480)),Scalar(0,255,0));
        rectangle(fframe,Rect(beginPt,Point(500,480)),Scalar(255,0,0));
        rectangle(fframe,Rect(beginPt+Point(0,-10),Point(500,480)),Scalar(255,0,0));
        fframe.copyTo(gui(Rect(Point(0,0),Point(640,480))));
        setButton();
        imshow("gui",gui);
        char c=waitKey(1);
        if(c=='f')
        {
            reset();
            ifUpdateEdge=true;
        }
    }
    catch(...)
    {

    }

    time=(double)(-time+static_cast<double>(getTickCount()))/getTickFrequency();
//    cout<<"time="<<time<<endl;
    return 0;
}

bool flag=false;
void buttonCallBack(int event,int x,int y,int flags,void* param)
{
    if(event==EVENT_LBUTTONDOWN)
    {
        flag=true;
//        cout<<"down "<<flag<<endl;
    }
    if(event==EVENT_LBUTTONUP)
    {
//        cout<<x<<" "<<y<<endl;
//        cout<<flag<<endl;
        if(flag==true)
        {
            if(y<230&&y>0)
            {
//                cout<<"fuck"<<endl;
                if(x>640&&x<756)
                {
//                    cout<<cvClass.expo<<endl;
                    cvClass.expo-=1;
                    cvClass.setCameraParam(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL);
                    cvClass.setCameraParam(V4L2_CID_EXPOSURE_ABSOLUTE,cvClass.expo);
                }
                else if(x>756&&x<872)
                {
//                    cout<<cvClass.expo<<endl;
                    cvClass.expo+=1;
                    cvClass.setCameraParam(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL);
                    cvClass.setCameraParam(V4L2_CID_EXPOSURE_ABSOLUTE,cvClass.expo);
                }
                else if(x>872&&x<990)
                {
                    cvClass.setCameraParam(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_SHUTTER_PRIORITY);

                }
            }
        }
        flag=false;
    }
}

void CVClass::setButton(void)
{
    rectangle(gui,Rect(Point(640,0),Point(756,230)),Scalar(0,0,0),-1);
    rectangle(gui,Rect(Point(756,0),Point(872,230)),Scalar(255,255,255),-1);
    rectangle(gui,Rect(Point(872,0),Point(990,230)),Scalar(125,125,125),-1);
    setMouseCallback("gui",buttonCallBack);
}

pair<Point,Point> CVClass::chooseTarg()
{
    int rlt=-1;
    Point tmp=Point(999,999);
    int id=-1;
//    cout<<Ledge.targets.oldTargets<<endl;
    for(int i=0;i<4;i++)
        cout<<Redge.targets.TargGood[i]<<endl;
    for(int i=0;i<Redge.targets.oldTargets.size();i++)
    {
        if(Redge.targets.TargGood[(i+1)%Redge.targets.oldTargets.size()]&&Redge.targets.oldTargets.at((i+1)%Redge.targets.oldTargets.size()).y<tmp.y)
        {
            tmp=Redge.targets.oldTargets.at((i+1)%Redge.targets.oldTargets.size());
            id=(i+1)%Redge.targets.oldTargets.size();
        }
    }
//    if(id==0)
//    {
//        targetID=id+1;
//        return pair<Point,Point>(Redge.targets.oldTargets.at(id+1),Ledge.targets.oldTargets.at(id+1));
//    }
//    else
    if(id>-1)
    {
        targetID=id;
        targetBox=oldGarage.at(id);
        return pair<Point,Point>(tmp,Ledge.targets.oldTargets.at(id));
    }
    else if(id==-1)
    {
        return pair<Point,Point>(Point(-1,-1),Point(-1,-1));
    }
////    return -1;
//    for(int i=0;i<4;i++)
//    {
//        if(targets.TargGood[i])
//        {
//            targetID=i;
//            return i;
//        }
//    }
}

void CVClass::setHome(void)
{
    home=car.pos;
}
void CVClass::init()
{
    printf("first~~\n");
    if(init_v4l2() == FALSE){
        printf("Init fail~~\n");
        exit(1);
    }
    printf("second~~\n");
    if(v4l2_grab() == FALSE){
        printf("grab fail~~\n");
        exit(2);
    }
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    printf("third~~\n");
    setCameraParam(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_SHUTTER_PRIORITY);
    setCameraParam(V4L2_CID_WHITE_BALANCE_TEMPERATURE,8000);
//    setCameraParam(V4L2_CID_EXPOSURE_ABSOLUTE,expo);
    cvClass.Ledge.setSide(Ledge.Edge_Left);
    cvClass.Redge.setSide(Redge.Edge_Right);
/*    namedWindow("gui",WINDOW_AUTOSIZE);
    setWindowProperty("gui",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
//    namedWindow("trackbar");
    createTrackbar("rbTT","gui",&rbTT,255);
    createTrackbar("rgTT","gui",&rgTT,255);
    createTrackbar("rrTT","gui",&rrTT,255);
    createTrackbar("bbTT","gui",&bbTT,255);
    createTrackbar("bgTT","gui",&bgTT,255);
    createTrackbar("btTT","gui",&brTT,255);
*/
}

void* CVClass::CVThread(void* arg)
{
    cvClass.init();
    while(1)
    {
        try
        {
            cvClass.CVProcess();

        }
        catch(...)
        {
            cout<<"camera error"<<endl;
        }
    }
}

int CVClass::CVThreadStart(void)
{
    pthread_t A_ARR;

    if(pthread_create(&A_ARR,0,CVThread,NULL)!=0)
    {
        return -1;
    }
    return 0;
}
