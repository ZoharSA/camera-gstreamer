#ifndef CAMERAS_MANAGER_H
#define CAMERAS_MANAGER_H

#include <camera-interface.h>

#include "CameraGstreamer.h"

#include <vector>

class CamerasManager {
    static constexpr const unsigned DEFAULT_TRIGGER_CAMERA = 0;
    DUFrameArrivedCallback _frameArrived = nullptr;
    DUCameraStateChangedCallback _cameraStateChanged = nullptr;
    void *_opaq = nullptr;
    std::vector<CameraGstreamer*> _cameras;
    unsigned _framePerSecond;
    unsigned _numCameras;
    bool _triggerCamera;

public:
    CamerasManager(
            DUFrameArrivedCallback frameArrived,
            unsigned framesPerSecond,
            DUCameraStateChangedCallback cameraStateChanged,
            void *opaq) :
        _frameArrived(frameArrived),
        _cameraStateChanged(cameraStateChanged),
        _opaq(opaq),
        _framePerSecond(framesPerSecond),
        _triggerCamera(false)
    {}

    ~CamerasManager()
    {}

    void frameGrabbed( CameraId id,
                    const uint8_t* buffer,
                    size_t size,
                    std::chrono::steady_clock::time_point timestamp,
                    uint32_t frameIndex,
                    bool trigger );
    void addCameras( const DUCameraDescriptor *descriptors, size_t numCameras );
    void start( CameraId );
    void stop( CameraId );

    CameraGstreamer &operator[]( CameraId id ) {
        return *_cameras[id];
    }
    void goodCamera( CameraId id );
    void badCamera( CameraId id );
    void badFrame( CameraId id );
    void badPipeline( CameraId id );
    void updateTiggerCamera();
};

#endif // CAMERAS_MANAGER_H
