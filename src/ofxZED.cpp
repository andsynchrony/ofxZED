#include "ofxZED.h"

ofxZED::ofxZED()
{
}

ofxZED::~ofxZED()
{
	close();
}

void ofxZED::init(bool useColorImage, bool useDepthImage, int cameraID, sl::DEPTH_MODE mode, sl::RESOLUTION resolution, float fps)
{
	// Set configuration parameters
	sl::InitParameters init_params;
	init_params.camera_resolution = resolution;
	init_params.camera_fps = fps;
	init_params.depth_mode = mode;
	init_params.enable_right_side_measure = true;
	init_params.input.setFromCameraID(cameraID);
	init_params.coordinate_units = sl::UNIT_METER;


	bUseColorImage = useColorImage;
	bUseDepthImage = useDepthImage;

	ofLog() << "Initializing ZED camera." << std::endl;

	zed = new sl::Camera();
	ofLog() << "Resolution Mode:" << resolution << std::endl;

	sl::ERROR_CODE zederr = zed->open(init_params);

	if (zederr != sl::SUCCESS)
	{
		ofLog() << "ERROR: " << sl::errorCode2str(zederr).c_str() << endl;
		ofExit();
	}
	else
	{
		ofLog() << "ZED initialized." << endl;

	}

	zedWidth = zed->getResolution().width;
	zedHeight = zed->getResolution().height;

	ofLog() << "Resolution: " << zedWidth << ", " << zedHeight << endl;
	ofLog() << "FPS: " << getCurrentFPS() << endl;

	colorLeftTexture.allocate(zedWidth, zedHeight, GL_RGBA);
	colorRightTexture.allocate(zedWidth, zedHeight, GL_RGBA);
	depthLeftTexture.allocate(zedWidth, zedHeight, GL_LUMINANCE32F_ARB);
	depthRightTexture.allocate(zedWidth, zedHeight, GL_LUMINANCE32F_ARB);
}

void ofxZED::close()
{
	if (zed) {
		zed->close();
		zed = nullptr;
	}
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
		sl::RuntimeParameters rt;
		rt.enable_depth = true;
		rt.sensing_mode = sl::SENSING_MODE::SENSING_MODE_FILL;
		auto ret = zed->grab(rt);
		if (ret != sl::SUCCESS) {
			ofLogError() << sl::errorCode2str(ret).c_str() << endl;
		}
	}
	else if(bUseColorImage)
	{
		sl::RuntimeParameters rt;
		rt.enable_depth = false;
		auto ret = zed->grab(rt);
		if (ret != sl::SUCCESS) {
			ofLogError() << sl::errorCode2str(ret).c_str() << endl;
		}
	}

	if (bUseColorImage) {
		{
			auto ret = zed->retrieveImage(this->cl, sl::VIEW_LEFT);
			if (ret == sl::SUCCESS) {
				colorLeftTexture.loadData(this->cl.getPtr<uint8_t>(), zedWidth, zedHeight, GL_RGBA);
			}
		}
		{
			auto ret = zed->retrieveImage(this->cr, sl::VIEW_RIGHT);
			if (ret == sl::SUCCESS) {
				colorRightTexture.loadData(this->cr.getPtr<uint8_t>(), zedWidth, zedHeight, GL_RGBA);
			}
		}
	}

	if (bUseDepthImage) {
		{
			auto ret = zed->retrieveMeasure(this->dl, sl::MEASURE_DEPTH);
			if (ret == sl::SUCCESS) {
				depthLeftTexture.loadData(this->dl.getPtr<float>(), zedWidth, zedHeight, GL_LUMINANCE);
			}
			else {
				ofLogError() << sl::errorCode2str(ret).c_str() << endl;
			}
		}
		{
			auto ret = zed->retrieveMeasure(this->dr, sl::MEASURE_DEPTH_RIGHT);
			if (ret == sl::SUCCESS) {
				depthRightTexture.loadData(this->dr.getPtr<float>(), zedWidth, zedHeight, GL_LUMINANCE);
			}
			else {
				ofLogError() << sl::errorCode2str(ret).c_str() << endl;
			}
		}
	}
}

