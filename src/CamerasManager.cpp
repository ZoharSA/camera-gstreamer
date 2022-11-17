#include <CamerasManager.h>

#include <iostream>
#include <string>
#include <assert.h>
#include <string.h>

void CamerasManager::addCameras(const DUCameraDescriptor *descriptors, size_t numCameras) {

    for( CameraId id=0; id<numCameras; ++id ) {
        try {
            _cameras.push_back( new CameraGstreamer(id, descriptors[id], this, _framePerSecond) );
            _cameraStateChanged( id, 0, _opaq );
        } catch( ... ) {
            std::cerr<<"Failed to open camera "<<id<<"\n";
            badCamera(id);
        }
    }
    if (numCameras > 0) {
        bool foundTriggerCamera = false;
        for( CameraId id=0; id<numCameras; ++id ) {
            if ( (foundTriggerCamera) and (_cameras[id]->getIsTrigger()) ){
                std::cerr<<"More than one trigger camera: " << id << std::endl;
            }
            else if ( _cameras[id]->getIsTrigger() ) {
                foundTriggerCamera = true;
            }
        }
        if ( !foundTriggerCamera ) {
            _cameras[ DEFAULT_TRIGGER_CAMERA ]->setTrigger( true );
        }
    }
}

void CamerasManager::frameGrabbed( CameraId id,
                                const uint8_t* buffer,
                                size_t size,
                                std::chrono::steady_clock::time_point timestamp,
                                uint32_t frameIndex,
                                bool trigger )
{
    DUBuffer frame{
        .ptr = static_cast<const uint8_t *>(buffer),
        .length = size,
        .capturedTimestamp = timestamp,
        .frameIndex = frameIndex,
        .ancillaryData = 0,
        .trigger = trigger,
        .customerData = {},
        .ctx = nullptr
    };
    _frameArrived( id, frame, _opaq );
}

void CamerasManager::start( const CameraId cameraId )
{
    assert(_cameras[cameraId]);
    _cameras[cameraId]->start();
}

void CamerasManager::stop( const CameraId cameraId )
{
    assert(_cameras[cameraId]);
    _cameras[cameraId]->stop();
}

void CamerasManager::goodCamera( CameraId id )
{
    _cameraStateChanged( id, 0, _opaq );
}

void CamerasManager::badCamera( CameraId id )
{
    _cameraStateChanged( id, 10000, _opaq );
}

void CamerasManager::badFrame( CameraId id )
{
    _cameraStateChanged( id, 20000, _opaq );
}
