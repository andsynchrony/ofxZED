#include "ofxZED.h"

ofxZED::ofxZED()
{
}

ofxZED::~ofxZED()
{
}

void ofxZED::init(bool useColorImage, bool useDepthImage, int cameraID, sl::zed::MODE mode, sl::zed::ZEDResolution_mode resolution, float fps)
{
	bUseColorImage = useColorImage;
	bUseDepthImage = useDepthImage;

	ofLog() << "Initializing ZED camera." << std::endl;

	zed = new sl::zed::Camera(resolution, fps);
	ofLog() << "Resolution Mode:" << resolution << std::endl;

	sl::zed::ERRCODE zederr = zed->init(mode, cameraID, true, false, false);

	if (zederr != sl::zed::SUCCESS)
	{
		ofLog() << "ERROR: " << sl::zed::errcode2str(zederr) << endl;
		ofExit();
	}
	else
	{
		ofLog() << "ZED initialized." << endl;

	}

	zedWidth = zed->getImageSize().width;
	zedHeight = zed->getImageSize().height;

	ofLog() << "Resolution: " << zedWidth << ", " << zedHeight << endl;
	ofLog() << "FPS: " << getCurrentFPS() << endl;

	colorBuffer = new uchar[zedWidth * zedHeight * 3];
	depthBuffer = new uchar[zedWidth * zedHeight];

	colorTexture.allocate(zedWidth, zedHeight, GL_RGB, false);
	depthTexture.allocate(zedWidth, zedHeight, GL_LUMINANCE, false);
}

ofVec2f ofxZED::getImageDimensions()
{
	return ofVec2f(zedWidth, zedHeight);
}

float ofxZED::getCurrentFPS() 
{
	return zed->getCurrentFPS();
}


void ofxZED::update()
{
	if (bUseDepthImage)
	{
		zed->grab(sl::zed::SENSING_MODE::FULL, true, true);
	}
	else if(bUseColorImage)
	{
		zed->grab(sl::zed::SENSING_MODE::RAW, false, false);
	}

	if(bUseColorImage)
		colorTexture.loadData(getColorBuffer(), zedWidth, zedHeight, GL_RGB);

	if(bUseDepthImage)
	depthTexture.loadData(getDepthBuffer(), zedWidth, zedHeight, GL_LUMINANCE);
}

void ofxZED::fillColorBuffer()
{
	sl::zed::Mat zedView = zed->retrieveImage(sl::zed::SIDE::LEFT);
	/*
	// wrong color / pixel format, unfunctional for now.
	memcpy(imageBuffer, zedView.data, zedWidth * zedHeight * 3 * sizeof(sl::uchar));
	// instead using:
	*/
	for (int y = 0; y < zedHeight; y++)
	{
		for (int x = 0; x < zedWidth; x++)
		{
			sl::uchar3 pixel = zedView.getValue(x, y);
			int index = 3 * (x + y * zedWidth);

			colorBuffer[index + 0] = pixel.c3;
			colorBuffer[index + 1] = pixel.c2;
			colorBuffer[index + 2] = pixel.c1;
			
		}
	}
	
}

void ofxZED::fillDepthBuffer()
{
	sl::zed::Mat zedView = zed->normalizeMeasure(sl::zed::MEASURE::DEPTH);
	for (int y = 0; y < zedHeight; y++)
	{
		for (int x = 0; x < zedWidth; x++)
		{
			sl::uchar3 pixel = zedView.getValue(x, y);
			
			int index = (x + y * zedWidth);
			depthBuffer[index] = pixel.c1;
		}
	}
}

uchar * ofxZED::getColorBuffer()
{
	if (!bUseColorImage)
		ofLogWarning() << "trying to access color buffer without setting useColorImage to true." << endl;
	else
		fillColorBuffer();
	return colorBuffer;
}

uchar * ofxZED::getDepthBuffer()
{
	if (!bUseDepthImage)
		ofLogWarning() << "trying to access depth buffer without setting useDepthImage to true." << endl;
	else
		fillDepthBuffer();
	return depthBuffer;
}

ofTexture * ofxZED::getColorTexture()
{
	if (!bUseColorImage)
		ofLogWarning() << "trying to access color buffer without setting useColorImage to true." << endl;
	return &colorTexture;
}

ofTexture * ofxZED::getDepthTexture()
{
	if (!bUseDepthImage)
		ofLogWarning() << "trying to access depth buffer without setting useDepthImage to true." << endl;
	return &depthTexture;
}
