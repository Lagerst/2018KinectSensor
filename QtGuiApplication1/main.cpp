#include "stdafx.h"
#include <strsafe.h>
#include <d2d1.h>
#include <iostream> 
#include "Kinect.h"
#include <fstream>
#include <string>
#include "Kinect.Face.h"
#include "myKinect.h"
#include "resource.h"
#include <opencv2/opencv.hpp>  
#include "QtGuiApplication1.h"
#include <QtWidgets/QApplication>
#include <qpushbutton.h>
#include <widget.h>
#include <iostream>
#include <cstdio>
#include <QGridLayout>
#include "thread.h"
#include <QDebug>
using namespace std;
using namespace cv;

ofstream oFile;
CFaceBasics myFace;

// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

string save_file_name="";

int OutputDataStreams() {
	return 0;
}

/// Constructor
CFaceBasics::CFaceBasics() :
	m_pKinectSensor(nullptr),
	m_pCoordinateMapper(nullptr),
	m_pBodyFrameReader(nullptr)
{
	LARGE_INTEGER qpf ={0};
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}
/// Destructor
CFaceBasics::~CFaceBasics()
{
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = nullptr;
	}

	// done with face sources and readers
	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with color frame reader
	SafeRelease(m_pColorFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

HRESULT CFaceBasics::InitializeDefaultSensor()
{
	//用于判断每次读取操作的成功与否
	HRESULT hr;
	//搜索kinect
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)) {
		return hr;
	}
	//找到kinect设备

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;//读取骨架
		IDepthFrameSource* pDepthFrameSource = NULL;//读取深度信息
		IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;//读取背景二值图
		//打开kinect
		hr = m_pKinectSensor->Open();

		//coordinatemapper
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
		//bodyframe
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		//depth frame
		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}
		if (SUCCEEDED(hr)) {
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		//body index frame
		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		}
		if (SUCCEEDED(hr)) {
			hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
		}
		IColorFrameSource* pColorFrameSource = nullptr;
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateFaceFrameSource(m_pKinectSensor,0,c_FaceFrameFeatures,&m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
			}

		}
		SafeRelease(pColorFrameSource);
		SafeRelease(pBodyFrameSource);
		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyIndexFrameSource);
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cout << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}
	return hr;
}
/// Main processing function
void CFaceBasics::Update()
{
	//如果丢失了kinect，则不继续操作
	if (!m_pBodyFrameReader)
	{
		return;
	}
	IBodyFrame* pBodyFrame = NULL;//骨架信息
	//记录每次操作的成功与否
	HRESULT hr = S_OK;
	//-----------------------------获取骨架并显示----------------------------
	if (SUCCEEDED(hr)) {
		hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);//获取骨架信息
	}
	if (SUCCEEDED(hr))
	{
		IBody* ppBodies[BODY_COUNT] ={0};//每一个IBody可以追踪一个人，总共可以追踪六个人
		if (SUCCEEDED(hr))
		{
			//把kinect追踪到的人的信息，分别存到每一个IBody中
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies),ppBodies);
		}
		if (SUCCEEDED(hr))
		{
			ProcessBody(BODY_COUNT,ppBodies);
		}
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);//释放所有
		}
	}
	SafeRelease(pBodyFrame);//必须要释放，否则之后无法获得新的frame数据

}

string s[32]={
		"JointType_SpineBase	= 0",
		"JointType_SpineMid	= 1",
		"JointType_Neck	= 2",
		"JointType_Head	= 3",
		"JointType_ShoulderLeft	= 4",
		"JointType_ElbowLeft	= 5",
		"JointType_WristLeft	= 6",
		"JointType_HandLeft	= 7",
		"JointType_ShoulderRight	= 8",
		"JointType_ElbowRight	= 9",
		"JointType_WristRight	= 10",
		"JointType_HandRight	= 11",
		"JointType_HipLeft	= 12",
		"JointType_KneeLeft	= 13",
		"JointType_AnkleLeft	= 14",
		"JointType_FootLeft	= 15",
		"JointType_HipRight	= 16",
		"JointType_KneeRight	= 17",
		"JointType_AnkleRight	= 18",
		"JointType_FootRight	= 19",
		"JointType_SpineShoulder	= 20",
		"JointType_HandTipLeft	= 21",
		"JointType_ThumbLeft	= 22",
		"JointType_HandTipRight	= 23",
		"JointType_ThumbRight	= 24",

		"FacePointType_None	= 25",
		"FacePointType_EyeLeft = 26",
		"FacePointType_EyeRight	= 27",
		"FacePointType_Nose	= 28",
		"FacePointType_MouthCornerLeft	= 29",
		"FacePointType_MouthCornerRight	= 30",
		"FacePointType_Count	= (FacePointType_MouthCornerRight + 1)"
};

void ExtractFaceRotationInDegrees(const Vector4* pQuaternion,int* pPitch,int* pYaw,int* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;
	static const double c_FaceRotationIncrementInDegrees = 5.0f;

	// convert face rotation quaternion to Euler angles in degrees		
	double dPitch,dYaw,dRoll;
	dPitch = atan2(2 * (y * z + w * x),w * w - x * x - y * y + z * z) / M_PI * 180.0;
	dYaw = asin(2 * (w * y - x * z)) / M_PI * 180.0;
	dRoll = atan2(2 * (x * y + w * z),w * w + x * x - y * y - z * z) / M_PI * 180.0;

	// clamp rotation values in degrees to a specified range of values to control the refresh rate
	double increment = c_FaceRotationIncrementInDegrees;
	*pPitch = static_cast<int>(floor((dPitch + increment/2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pYaw = static_cast<int>(floor((dYaw + increment/2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * increment);
	*pRoll = static_cast<int>(floor((dRoll + increment/2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * increment);
}

int r[30]={};

/// Handle new body data
void CFaceBasics::ProcessBody(int nBodyCount,IBody** ppBodies)
{
	HRESULT hr;
	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];
				hr = pBody->GetJoints(_countof(joints),joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < _countof(joints); ++j)
					{
						float x=joints[j].Position.X,y=joints[j].Position.Y,z=joints[j].Position.Z;
						//获取joint[j]的坐标 
						//CameraSpacePoint cameraSpacePoint ={x, y, z};
						//cout<<s[j]<<endl;
						//cout<<"        x="<<x<<"  y="<<y<<"  z="<<z<<endl;
						oFile<<x<<","<<y<<","<<z<<",";
					}
				}
			}
		}
	}
}

Mat CFaceBasics::ConvertMat(const UINT16* pBuffer,int nWidth,int nHeight,USHORT nMinDepth,USHORT nMaxDepth)
{
	Mat img(nHeight,nWidth,CV_16UC1);
	UINT16* p_mat = (UINT16*)img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;
		*p_mat = (depth >= nMinDepth) && (depth <= nMaxDepth) ? depth<<3 : 0;
		p_mat++;
		++pBuffer;
	}
	return img;
}

/// <summary>
/// Updates body data
/// </summary>
/// <param name="ppBodies">pointer to the body data storage</param>
/// <returns>indicates success or failure</returns>
HRESULT CFaceBasics::UpdateBodyData(IBody** ppBodies)
{
	HRESULT hr = S_OK;

	if (m_pBodyFrameReader != nullptr)
	{
		IBodyFrame* pBodyFrame = NULL;
		hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT,ppBodies);
		}
		SafeRelease(pBodyFrame);
	}

	return hr;
}

bool CFaceBasics::ProcessFaces()
{
	HRESULT hr;
	bool flag=false;
	IBody* ppBodies[BODY_COUNT] ={0};
	bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies));

	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		if (SUCCEEDED(hr))
		{
			if (bFaceTracked)
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				RectI faceBox ={0};
				PointF facePoints[FacePointType::FacePointType_Count];
				Vector4 faceRotation;
				DetectionResult faceProperties[FaceProperty::FaceProperty_Count];

				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
				{
					hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

					if (SUCCEEDED(hr))
					{
						IDepthFrame* pDepthFrame = NULL;
						UINT16*depthData = new UINT16[424 * 512];
						CameraSpacePoint* m_pColorCoordinates = new CameraSpacePoint[1920 * 1080];//RGB图输出为1920*1080
						hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count,facePoints);
						hr= m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
						if (SUCCEEDED(hr)) {
							hr= pDepthFrame->CopyFrameDataToArray(424 * 512,depthData);
							if (SUCCEEDED(hr)) {
								HRESULT hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(512 * 424,depthData,1920 * 1080,m_pColorCoordinates);
								for (int i=0;i<FacePointType_Count;i++) {
									float x=m_pColorCoordinates[int(facePoints[i].Y) * 1920 + int(facePoints[i].X)].X;
									float y=m_pColorCoordinates[int(facePoints[i].Y) * 1920 + int(facePoints[i].X)].Y;
									float z=m_pColorCoordinates[int(facePoints[i].Y) * 1920 + int(facePoints[i].X)].Z;
									//cout<<s[i+26]<<endl;
									//cout<<"        x="<<x<<"  y="<<y<<"  z="<<z<<endl;
									oFile<<x<<","<<y<<","<<z<<",";
								}
								flag=true;
							}
						}
						cv::Mat mDepthImg(424,512,CV_16UC1);
						pDepthFrame->CopyFrameDataToArray(424*512,(UINT16 *)mDepthImg.data);
						mDepthImg.convertTo(mDepthImg,CV_8UC1,255.0 / 4500);
						//cout<<"save picture to "<<save_file_name<<endl;
						string temp="C:\\Temp\\Savepic\\Depth_"+save_file_name;
						imwrite(temp,mDepthImg);
						SafeRelease(pDepthFrame);
						free(depthData);
						mDepthImg.release();
						free(m_pColorCoordinates);
						ColorPrint();
					}


					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					}

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count,faceProperties);
					}

					if (SUCCEEDED(hr))
					{
						int pitch,yaw,roll;
						ExtractFaceRotationInDegrees(&faceRotation,&pitch,&yaw,&roll);
						//cout<<"faceRotation:"<<"    pitch="<<pitch<<endl<<"    yaw="<<yaw<<endl<<"    roll="<<roll<<endl;
						oFile<<pitch<<","<<yaw<<","<<roll<<",";
						//for (int i=0;i<FaceProperty_Count;i++) {
						//	cout<<"    FaceProperty["<<i<<"]="<<faceProperties[i]<<endl;
						//}
					}
				}

				SafeRelease(pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}

		SafeRelease(pFaceFrame);
	}
	for (int i = 0; i < _countof(ppBodies); ++i)
	{
		SafeRelease(ppBodies[i]);//释放所有
	}
	return flag;
}

String csvFileName;

void output() {
	oFile.open(csvFileName,ios::out|ios::trunc);
	oFile<<"序号"<<","<<"结果"<<","<<"Time"<<",";
	for (int i=26;i<=30;i++) {
		oFile<<s[i]<<"X,"<<s[i]<<"Y,"<<s[i]<<"Z,";
	}
	oFile<<"pitch"<<","<<"yaw"<<","<<"roll"<<",";
	for (int i=0;i<=24;i++) {
		oFile<<s[i]<<"X,"<<s[i]<<"Y,"<<s[i]<<"Z,";
	}
	oFile<<endl;
	oFile.close();
}

void CFaceBasics::ColorPrint() {
	IColorFrameSource   * mySource = nullptr;
	HRESULT hr=S_OK;
	m_pKinectSensor->get_ColorFrameSource(&mySource);
	int height=1080,width=1920;
	Mat img(height,width,CV_8UC4);
	IColorFrame     * myFrame = nullptr;
	while (1)
	{
		if (m_pColorFrameReader->AcquireLatestFrame(&myFrame) == S_OK) //第4步获取Frame
		{
			UINT    size = 0;
			myFrame->CopyConvertedFrameDataToArray(width * height * 4,(BYTE *)img.data,ColorImageFormat_Bgra);
			imwrite("C:\\Temp\\Savepic\\Color_"+save_file_name,img);
			myFrame->Release();
			break;
		}
	}
	img.release();
	mySource->Release();
}

string Get_Current_Date()
{
	time_t nowtime;
	nowtime = time(NULL); //获取日历时间   
	char tmp[64];
	strftime(tmp,sizeof(tmp),"%Y-%m-%d-%H_%M_%S",localtime(&nowtime));
	return tmp;
}

bool steady;

Widget::Widget(QWidget *parent)
	: QWidget(parent)   
	//初始化列表, 在子类中调用父类构造函数, parent的值(默认为0)传给了QWidget()构造函数; 这是为了创建一个新的窗体, 这个窗体是一个顶层窗体(因为QWidget()构造函数的第一个形参代表父窗口, 如果对应的形参为0, 则代表生成一个父窗口)
{
	btn1 = new QPushButton(this);   //动态子对象btn1通过QPushButton类的构造函数创建
	btn2 = new QPushButton(this);
	edit1 = new QLineEdit(this);
	edit2 = new QLineEdit(this);
	edit3 = new QLineEdit(this);
	label1 = new QLabel(this);  //上面都是在子类Widget的构造函数里面创建控件(即"动态对象"作"成员对象")
	label2 = new QLabel(this);
	label3 = new QLabel(this);
	label1->setText("persongId");  //setText()是对象label1的成员函数; 记得包含相关头文件
	label2->setText("RoundNo");
	label3->setText("label");
	btn1->setText("start");
	btn2->setText("end");
	QGridLayout *layout = new QGridLayout(this);    //建立一个布局(layout), 这个布局是一个动态对象
	layout->addWidget(edit1,1,0);
	layout->addWidget(edit2,1,1);
	layout->addWidget(edit3,1,2);
	layout->addWidget(btn1,2,0);
	layout->addWidget(btn2,2,1);
	layout->addWidget(label1,0,0); //上面的layout对象的addWidget()成员函数用于在布局中插入控件
	layout->addWidget(label2,0,1);
	layout->addWidget(label3,0,2);
	connect(btn1,SIGNAL(clicked()),this,SLOT(add()));//将btn1的点击事件和add函数关联
	connect(btn2,SIGNAL(clicked()),this,SLOT(stop()));
	connect(btn2,SIGNAL(clicked()),this,SLOT(stop()));
}

Widget::~Widget()
{

}

void Widget::add()  //实现槽函数
{
	if (!steady) {
		QString s1 = edit1->text();//得到在edit1控件中用户输入的字符
		QString s2 = edit2->text();
		QString s3 = edit3->text();
		thread.setThread(s1,s2,s3);
		thread.start();
	}
}

void Widget::stop()  //实现槽函数
{
	thread.stop();
}


Thread::Thread()
{
	steady = false;
}

void Thread::stop()
{
	steady = false;
}


void Thread::setThread(QString p,QString r,QString l)
{
	personID = p;
	roundNo = r;
	label = l;
}

void Thread::run()
{
	std::string PersonID= personID.toStdString();
	std::string RoundNo = roundNo.toStdString();
	std::string Label = label.toStdString();
	int num=0;
	steady=true;
	csvFileName= "C:\\Temp\\Savepic\\PersonID_"+PersonID+"_RoundNo_"+RoundNo+"_Label_"+Label+".csv";
	while (steady) {
		num++;
		save_file_name= PersonID+"_"+RoundNo+"_"+Label+"_"+Get_Current_Date()+std::to_string(num)+".jpg";
		oFile.open(csvFileName,ios::app);
		oFile<<Label<<","<<RoundNo<<","<<PersonID<<","<<num<<","<<Get_Current_Date()<<",";
		bool flag=false;
		while (!flag) {
			flag=myFace.ProcessFaces();
			if (flag) {
				myFace.Update();
			}
		}
		oFile<<"Depth_"+save_file_name<<","<<"Color_"+save_file_name<<endl;
		oFile.close();
		Sleep(125);
	}
}

int main(int argc,char *argv[])
{
	HRESULT hr= myFace.InitializeDefaultSensor();
	QApplication a(argc,argv);
	Widget w;
	w.show();
	w.resize(800,100);
	return a.exec();
}

