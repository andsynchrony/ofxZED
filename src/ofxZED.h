#pragma once

#include "ofMain.h"
#include <zed/Camera.hpp>

class ofxZED
{
public:
	ofxZED();
	~ofxZED();
	void init(bool useColorImage = true, bool useDepthImage = true, int cameraID = 0, sl::zed::MODE mode = sl::zed::PERFORMANCE, sl::zed::ZEDResolution_mode resolution = sl::zed::HD720, float fps = 0.0);
	void update();
	uchar * getColorBuffer();
	uchar * getDepthBuffer();
	ofVec2f getImageDimensions();

	ofTexture * getColorTexture();
	ofTexture * getDepthTexture();

	int zedWidth;
	int zedHeight;

	float getCurrentFPS();

	int getDepthAtPoint(int x, int y);

private:
	void fillColorBuffer();
	void fillDepthBuffer();

	bool bUseColorImage;
	bool bUseDepthImage;

	sl::zed::Camera* zed = 0;
	uchar * colorBuffer;
	uchar * depthBuffer;
	ofTexture colorTexture;
	ofTexture depthTexture;
};

