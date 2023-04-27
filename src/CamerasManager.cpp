#include <CamerasManager.h>
#include <mutex>
#include <iostream>
#include <string>
#include <assert.h>
#include <string.h>

void CamerasManager::addCameras(const DUCameraDescriptor *descriptors, size_t numCameras) {
    _numCameras = numCameras;
    for( CameraId id=0; id<numCameras; ++id ) {
        try {
            _cameras.push_back( new CameraGstreamer(id, descriptors[id], this, _framePerSecond) );
            _cameraStateChanged( id, 0, _opaq );      
        } catch( ... ) {
            std::cerr<<"Failed to open camera "<<id<<"\n";
            badCamera(id);
        }
    }
    updateTiggerCamera();
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
    if ( !_triggerCamera ) {
        updateTiggerCamera();
    }
}

void CamerasManager::stop( const CameraId cameraId )
{
    assert(_cameras[cameraId]);
    _cameras[cameraId]->stop();
    if( _cameras[cameraId]->getIsTrigger() ) {
        _cameras[cameraId]->setTrigger( false );
        updateTiggerCamera();
    }
}

void CamerasManager::goodCamera( CameraId id )
{
    _cameraStateChanged( id, 0, _opaq );
    if ( !_triggerCamera ) {
        updateTiggerCamera();
    }
}

void CamerasManager::badCamera( CameraId id )
{
    std::cout << "badCamera id:" << id << std::endl;
    _cameraStateChanged( id, 10000, _opaq );
    if( _cameras[id]->getIsTrigger() ) {
        _cameras[id]->setTrigger( false );
        std::cout << "is trigger id:" << id << std::endl;
        updateTiggerCamera();
    }
}

void CamerasManager::badFrame( CameraId id )
{
    _cameraStateChanged( id, 20000, _opaq );
}

void CamerasManager::badPipeline( CameraId id )
{
    _cameraStateChanged( id, 10001, _opaq );
    if( _cameras[id]->getIsTrigger() ) {
        _cameras[id]->setTrigger( false );
        updateTiggerCamera();
    }
}

void CamerasManager::updateTiggerCamera()
{
    for( CameraId id=0; id<_numCameras; ++id ) {
        if( _cameras[id]->isActive() ) {
            if( _cameras[id]->getIsTrigger() ) {
                std::cout << "Camera id: " << id << " is allready trigger camera" << std::endl;
            } else {
                std::cout << "Setting camera id: " << id << " to be trigger camera" << std::endl;
                _cameras[id]->setTrigger( true );
            }
            _triggerCamera = true;
            return;
        }
    }
    std::cout << "No trigger camera" << std::endl;
    _triggerCamera = false;
}