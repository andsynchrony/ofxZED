#include "ofxZED.h"
namespace ofxZED
{
	Camera::Camera()
	{
	}

	Camera::~Camera()
	{
		close();
	}

	void Camera::init(bool useColorImage, bool useDepthImage, bool useTracking, int cameraID, sl::DEPTH_MODE mode, sl::RESOLUTION resolution, float fps)
	{
		// Set configuration parameters
		sl::InitParameters init_params;
		init_params.camera_resolution = resolution;
		init_params.camera_fps = fps;
		init_params.depth_mode = mode;
		init_params.enable_right_side_measure = true;
		init_params.input.setFromCameraID(cameraID);
		init_params.coordinate_units = sl::UNIT::METER;
		init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	
		bUseColorImage = useColorImage;
		bUseDepthImage = useDepthImage;
		bUseTracking = useTracking;

		ofLog() << "Initializing ZED camera." << std::endl;

		zed = new sl::Camera();
		ofLog() << "Resolution Mode:" << resolution << std::endl;

		auto zederr = zed->open(init_params);

		if (zederr != sl::ERROR_CODE::SUCCESS)
		{
			close();
			ofLog() << "ERROR: " << sl::errorCode2str(zederr).c_str() << endl;
			return;
		}
		else
		{
			bZedReady = true;
			ofLog() << "ZED initialized." << endl;
		}

		if (bUseTracking) {
			// Enable positional tracking with default parameters
			sl::PositionalTrackingParameters tracking_parameters;
			zederr = zed->enablePositionalTracking(tracking_parameters);

			if (zederr != sl::ERROR_CODE::SUCCESS)
			{
				close();
				ofLog() << "ERROR: " << sl::errorCode2str(zederr).c_str() << endl;
				return;
			}
		}

		zedWidth = zed->getCameraInformation().camera_resolution.width;
		zedHeight = zed->getCameraInformation().camera_resolution.height;

		ofLog() << "Resolution: " << zedWidth << ", " << zedHeight << endl;
		ofLog() << "FPS: " << getCurrentFPS() << endl;

		colorLeftTexture.allocate(zedWidth, zedHeight, GL_RGBA);
		colorRightTexture.allocate(zedWidth, zedHeight, GL_RGBA);
		depthLeftTexture.allocate(zedWidth, zedHeight, GL_LUMINANCE);
		depthRightTexture.allocate(zedWidth, zedHeight, GL_LUMINANCE);

		bRequestNewFrame = true;
		if (bUseDepthImage)
		{
			rt.enable_depth = true;
			rt.sensing_mode = sl::SENSING_MODE::FILL;
		}
		else if (bUseColorImage)
		{
			rt.enable_depth = false;
		}

		startThread();
	}

	void Camera::close()
	{
		if (isThreadRunning()) {
			waitForThread(true);
		}

		if (zed) {
			if (bUseTracking) {
				zed->disablePositionalTracking();
			}
			zed->close();
			zed = nullptr;
			bZedReady = false;
		}
	}

	ofVec2f Camera::getImageDimensions()
	{
		return ofVec2f(zedWidth, zedHeight);
	}

	float Camera::getCurrentFPS() 
	{
		return zed->getCurrentFPS();
	}

	void Camera::threadedFunction()
	{
		while (isThreadRunning()) {
			sl::ERROR_CODE zederr = sl::ERROR_CODE::SUCCESS;
			if (bRequestNewFrame && bZedReady) {
				zederr = zed->grab(rt);

				if (lock()) {
					if (zederr == sl::ERROR_CODE::CAMERA_NOT_DETECTED) {
						bDisconnected = true;
						bRequestNewFrame = false;
						ofSleepMillis(10);
					}
					else if (zederr == sl::ERROR_CODE::SUCCESS) {
						cameraTimestampBack = zed->getTimestamp(sl::TIME_REFERENCE::IMAGE);

						if (bUseTracking) {
							trackingState = zed->getPosition(pose);
						}
						else {
							trackingState = sl::POSITIONAL_TRACKING_STATE::OFF;
						}

						bNewBuffer = true;
						bRequestNewFrame = false;
					}
					unlock();
				}
			}
			ofSleepMillis(1);
		}
	}


	void Camera::update()
	{
		bNewFrame = false;

		if (bZedReady && bDisconnected) {
			close();
			bDisconnected = false;
			return;
		}

		if (!bZedReady || !zed) {
			return;
		}

		if (bNewBuffer) {
			if (lock()) {
				if (bUseColorImage) {
					{
						auto ret = zed->retrieveImage(this->cl, sl::VIEW::LEFT);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							colorLeftTexture.loadData(this->cl.getPtr<uint8_t>(), zedWidth, zedHeight, GL_RGBA);
						}
					}
					{
						auto ret = zed->retrieveImage(this->cr, sl::VIEW::RIGHT);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							colorRightTexture.loadData(this->cr.getPtr<uint8_t>(), zedWidth, zedHeight, GL_RGBA);
						}
					}
				}

				if (bUseDepthImage) {
					{
						auto ret = zed->retrieveMeasure(this->dl, sl::MEASURE::DEPTH);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							depthLeftTexture.loadData(this->dl.getPtr<float>(), zedWidth, zedHeight, GL_LUMINANCE);
						}
						else {
							ofLogError() << sl::errorCode2str(ret).c_str() << endl;
						}
					}
					{
						auto ret = zed->retrieveMeasure(this->dr, sl::MEASURE::DEPTH_RIGHT);
						if (ret == sl::ERROR_CODE::SUCCESS) {
							depthRightTexture.loadData(this->dr.getPtr<float>(), zedWidth, zedHeight, GL_LUMINANCE);
						}
						else {
							ofLogError() << sl::errorCode2str(ret).c_str() << endl;
						}
					}
				}
				cameraTimestamp = cameraTimestampBack;
				unlock();
				bNewBuffer = false;
				bRequestNewFrame = true;
				bNewFrame = true;
				lastNewFrame = ofGetFrameNum();
			}
		}

	}
}
