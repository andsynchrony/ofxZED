#pragma once

#include "ofMain.h"
#include <sl/Camera.hpp>

namespace ofxZED
{
	// convert func
	inline ofMatrix4x4 toOf(const sl::Transform& t)
	{
		auto m = ofMatrix4x4(&t.m[0]);
		return ofMatrix4x4::getTransposedOf(m);
	}

	inline sl::Transform toZed(const ofMatrix4x4& mat)
	{
		ofMatrix4x4 m = ofMatrix4x4::getTransposedOf(mat);
		return sl::Transform(m.getPtr());
	}

	inline sl::Transform toZed(const glm::mat4& mat)
	{
		ofMatrix4x4 m = mat;
		return toZed(m);
	}

	inline ofVec2f toOf(const sl::float2& v)
	{
		return ofVec2f(v.x, v.y);
	}

	inline ofVec3f toOf(const sl::float3& v)
	{
		return ofVec3f(v.x, v.y, v.z);
	}

	inline ofVec4f toOf(const sl::float4& v)
	{
		return ofVec4f(v.x, v.y, v.z, v.w);
	}

	inline ofVec3f toOf(const sl::Translation& v)
	{
		return ofVec3f(v.x, v.y, v.z);
	}

	inline ofQuaternion toOf(const sl::Orientation& v)
	{
		return ofQuaternion(v.x, v.y, v.z, v.w);
	}

	inline ofMatrix4x4 toOf(sl::Pose v)
	{
		ofMatrix4x4 mat;
		mat.setRotate(toOf(v.getOrientation()));
		mat.setTranslation(toOf(v.getTranslation()));
		return mat;
	}

	inline sl::float2 toZed(const ofVec2f& v)
	{
		return sl::float2(v.x, v.y);
	}

	inline sl::float3 toZed(const ofVec3f& v)
	{
		return sl::float3(v.x, v.y, v.z);
	}

	inline sl::float4 toZed(const ofVec4f& v)
	{
		return sl::float4(v.x, v.y, v.z, v.w);
	}

	inline sl::Orientation toZed(const ofQuaternion& v)
	{
		return sl::Orientation(sl::float4(v.x(), v.y(), v.z(), v.w()));
	}

	static constexpr std::array<int, 34> BODY18_CONNECTIONS = {0,1,1,2,2,3,3,4,1,5,5,6,6,7,1,8,8,9,9,10,1,11,11,12,12,13,0,14,14,16,0,15,15,17};

	class MR;

	class Camera : public ofThread
	{
	public:
		friend class MR;

		Camera();
		~Camera();
		
		// for backward compatibility
		void init(
			bool useColorImage = true, 
			bool useDepthImage = true, 
			bool useTracking = true,
			bool bUseSensor = false,
			int cameraID = 0,
			sl::DEPTH_MODE mode = sl::DEPTH_MODE::ULTRA,
			sl::RESOLUTION resolution = sl::RESOLUTION::HD720,
			float fps = 0.0,
			bool enableRightSideMeasure = true,
			float minDepth = -1.0, float maxDepth = -1.0,
			int sdkGpuId = 0);

		// advanced open
		bool open(
			sl::InitParameters init_params,
			sl::RuntimeParameters runtime_params,
			bool bStartThread = true);
		void enablePositionalTracking(sl::PositionalTrackingParameters tracking_params);
		void enableSensorThread();
		void enableObjectDetection(
			sl::ObjectDetectionParameters obj_det_params,
			sl::ObjectDetectionRuntimeParameters obj_det_params_rt);

		void disablePositionalTracking();
		void disableSensorThread();
		void disableObjectDetection();

		void close();
		void update();
		ofVec2f getImageDimensions();

		ofTexture& getColorLeftTexture() { return colorLeftTexture; }
		ofTexture& getColorRightTexture() { return colorRightTexture; }
		ofTexture& getDepthLeftTexture() { return depthLeftTexture; }
		ofTexture& getDepthRightTexture() { return depthRightTexture; }

		ofPixels& getColorLeftPixels() { return colorLeftPixels; }
		ofPixels& getColorRightPixels() { return colorRightPixels; }
		ofFloatPixels& getDepthLeftPixels() { return depthLeftPixels; }
		ofFloatPixels& getDepthRightPixels() { return depthRightPixels; }

		int zedWidth;
		int zedHeight;

		float getCurrentFPS();

		bool isFrameNew() const { return bNewFrame; }
		sl::Camera* getZedCamera() { return zed; }

		ofMatrix4x4 getTrackedPose() const { return toOf(pose); }
		const sl::Objects& getObjectDetectionResult() const { return objDetResult; }

		void setSensingMode(sl::SENSING_MODE mode);
		sl::RuntimeParameters& getRuntimeParams() { return rt; }
		sl::ObjectDetectionRuntimeParameters& getObjectDetectionRuntimeParams() { return objDetRtParams; }
		void setEnableUpdateColorImage(bool b) { bUseColorImage = b; }

		void debugDrawObjectDetectionResult2D() const;
		void debugDrawObjectDetectionResult3D() const;
	protected:
		void threadedFunction() override;

		sl::Mat cl, cr, dl, dr;

		bool bUseColorImage = true;
		bool bUseDepthImage = true;
		bool bUseTracking = false;
		bool bUseObjectDetection = false;
		bool bUseSensor = false;
		bool bEnableRightSideMeasure = false;
		bool bNewFrame = false;
		uint64_t lastNewFrame = 0;

		sl::Camera* zed = nullptr;
		ofTexture colorLeftTexture;
		ofTexture colorRightTexture;
		ofTexture depthLeftTexture;
		ofTexture depthRightTexture;

		ofPixels colorLeftPixels;
		ofPixels colorRightPixels;
		ofFloatPixels depthLeftPixels;
		ofFloatPixels depthRightPixels;

		bool bRequestNewFrame = true;
		bool bNewBuffer = false;
		bool bZedReady = false;
		bool bDisconnected = false;

		sl::RuntimeParameters rt;
		sl::Timestamp cameraTimestampBack;
		sl::Timestamp cameraTimestamp;
		sl::POSITIONAL_TRACKING_STATE trackingState;
		sl::Pose poseBack;
		sl::Pose pose;

		class SensorThread : public ofThread
		{
			Camera* parent_ = nullptr;
			vector<sl::SensorsData::IMUData> imuBack;
			void threadedFunction() override;
		public:
			SensorThread(Camera* parent);
			void update(vector<sl::SensorsData::IMUData>& imuMainThread);
		};

		shared_ptr<SensorThread> sensorThread;

		vector<sl::SensorsData::IMUData> imu;

		sl::ObjectDetectionRuntimeParameters objDetRtParams;
		sl::Objects objDetResultBack;
		sl::Objects objDetResult;
		

		bool handleZedResponse(sl::ERROR_CODE zederr) {
			auto success = zederr == sl::ERROR_CODE::SUCCESS;
			if (!success) {
				ofLog() << "ERROR: " << sl::errorCode2str(zederr).c_str();
			}
			return success;
		}
	};
}


