#pragma once

#include "ofMain.h"
#include <zed/Camera.hpp>

class ofxZED
{
public:
	ofxZED();
	~ofxZED();
	void init(bool useColorImage = true, bool useDepthImage = true);
	void update();
	uchar * getColorBuffer();
	uchar * getDepthBuffer();
	ofVec2f getImageDimensions();

	ofTexture * getColorTexture();
	ofTexture * getDepthTexture();

private:
	void fillColorBuffer();
	void fillDepthBuffer();

	bool bUseColorImage;
	bool bUseDepthImage;

	sl::zed::Camera* zed = 0;
	int zedWidth;
	int zedHeight;
	uchar * colorBuffer;
	uchar * depthBuffer;
	ofTexture colorTexture;
	ofTexture depthTexture;
};

