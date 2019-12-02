#pragma once
#include <d2d1.h>
#include <Kinect.h>
#include "Kinect.Face.h"

#include <opencv2/opencv.hpp>  

using namespace cv;

class CFaceBasics
{
	static const int       cColorWidth  = 1920;
	static const int       cColorHeight = 1080;
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CFaceBasics();

	/// <summary>
	/// Destructor
	/// </summary>
	~CFaceBasics();

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK  MessageRouter(HWND hWnd,UINT uMsg,WPARAM wParam,LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK       DlgProc(HWND hWnd,UINT uMsg,WPARAM wParam,LPARAM lParam);

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	void                    Run();

public:
	Mat ConvertMat(const UINT16* pBuffer,int nWidth,int nHeight,USHORT nMinDepth,USHORT nMaxDepth);
	/// <summary>
	/// Main processing function
	/// </summary>
	void                   Update();
	void ColorPrint();
	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success else the failure code</returns>
	HRESULT                InitializeDefaultSensor();

	/// <summary>
	/// Renders the color and face streams
	/// </summary>			
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	void                   DrawStreams(INT64 nTime,RGBQUAD* pBuffer,int nWidth,int nHeight);

	/// <summary>
	/// Processes new face frames
	/// </summary>
	bool                   ProcessFaces();

	/// <summary>
	/// Computes the face result text layout position by adding an offset to the corresponding 
	/// body's head joint in camera space and then by projecting it to screen space
	/// </summary>
	/// <param name="pBody">pointer to the body data</param>
	/// <param name="pFaceTextLayout">pointer to the text layout position in screen space</param>
	/// <returns>indicates success or failure</returns>
	HRESULT                GetFaceTextPositionInColorSpace(IBody* pBody,D2D1_POINT_2F* pFaceTextLayout);

	/// <summary>
	/// Updates body data
	/// </summary>
	/// <param name="ppBodies">pointer to the body data storage</param>
	/// <returns>indicates success or failure</returns>
	HRESULT                UpdateBodyData(IBody** ppBodies);
private:
	HWND                   m_hWnd;
	INT64                  m_nStartTime;
	INT64                  m_nLastCounter;
	double                 m_fFreq;
	ULONGLONG              m_nNextStatusTime;
	DWORD                  m_nFramesSinceUpdate;

	// Current Kinect
	IKinectSensor*         m_pKinectSensor;

	// Coordinate mapper
	ICoordinateMapper*     m_pCoordinateMapper;

	// Color reader
	IColorFrameReader*     m_pColorFrameReader;

	// Body reader
	IBodyFrameReader*      m_pBodyFrameReader;

	// Face sources
	IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];

	// Face readers
	IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];

	RGBQUAD*               m_pColorRGBX;

	IDepthFrameReader*      m_pDepthFrameReader;//用于深度数据读取

	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//用于背景二值图读取

	void                    ProcessBody(int nBodyCount,IBody** ppBodies);
};