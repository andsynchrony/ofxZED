#pragma once

#include "ofMain.h"
#include <sl_zed/Camera.hpp>

class ofxZED
{
public:
	ofxZED();
	~ofxZED();
	void init(bool useColorImage = true, bool useDepthImage = true, int cameraID = 0, 
		sl::DEPTH_MODE mode = sl::DEPTH_MODE::DEPTH_MODE_PERFORMANCE, 
		sl::RESOLUTION resolution = sl::RESOLUTION::RESOLUTION_HD720, float fps = 0.0);
	void close();
	void update();
	ofVec2f getImageDimensions();

	ofTexture& getColorLeftTexture() { return colorLeftTexture;  }
	ofTexture& getColorRightTexture() { return colorRightTexture; }
	ofTexture& getDepthLeftTexture() { return depthLeftTexture; }
	ofTexture& getDepthRightTexture() { return depthRightTexture; }

	int zedWidth;
	int zedHeight;

	float getCurrentFPS();

protected:
	sl::Mat cl,cr,dl,dr;

	bool bUseColorImage;
	bool bUseDepthImage;

	sl::Camera* zed = nullptr;
	ofTexture colorLeftTexture;
	ofTexture colorRightTexture;
	ofTexture depthLeftTexture;
	ofTexture depthRightTexture;
};

