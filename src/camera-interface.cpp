#include <camera-interface.h>

#include "CamerasManager.h"

#include <memory>
#include <assert.h>
#include <stdio.h>

std::unique_ptr<CamerasManager> camerasManager;

bool duCamerasInit(
        const DUCameraDescriptor cameras[],
        unsigned framesPerSecond,
        unsigned numCameras,
        DUFrameArrivedCallback frameArrived,
        DUCameraStateChangedCallback cameraStateChangedCallback,
        void *ctx ){
    try {
        camerasManager = std::unique_ptr<CamerasManager>(
                new CamerasManager(frameArrived,
                                   framesPerSecond,
                                   cameraStateChangedCallback,
                                   ctx) );
        camerasManager->addCameras(cameras, numCameras);
    } catch( std::exception &ex ) {
        std::cerr<<"GSTREAMER camera initialization failed: "<<ex.what()<<"\n";

        return false;
    }

    return true;
}

void duCamerasShutdown() {
}

void duCamerasReleaseFrame( const DUBuffer *frame ) {

}

void duCameraEnable( CameraId id ) {
    camerasManager->start(id);
}

void duCameraDisable( CameraId id ) {
    camerasManager->stop(id);
    // MIYAD-6205: Currently we don't have a mechanism to switch trigger cameras, so we should never stop the trigger cam
    std::cout << "[camera " << id << "] duCameraDisable called for camera. Doing nothing." << std::endl;
}

uint64_t duLibraryVersion() {
    return DU_CURRENT_VERSION;
}
