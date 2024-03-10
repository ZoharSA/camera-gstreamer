#ifndef CAMERAS_MANAGER_H
#define CAMERAS_MANAGER_H

#include <camera-interface.h>

#include "UdpServerSocket.h"
#include "CameraGstreamer.h"

#include <vector>

class CamerasManager {
    static constexpr const unsigned DEFAULT_TRIGGER_CAMERA = 0;
    DUFrameArrivedCallback _frameArrived = nullptr;
    DUCameraStateChangedCallback _cameraStateChanged = nullptr;
    void *_opaq = nullptr;
    std::vector<CameraGstreamer*> _cameras;
    unsigned _framePerSecond;

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
        _socket(),
        _customerDataInterface({0, 0, 0}),
        _customerDataReceiveLoopThread()
    {
        initCustomerDataUdpReceiver();
    }

    ~CamerasManager()
    {
        _stopCustomerDataUdpReceiver = true;
        unblockCustomerDataUdpReceiverThread();
        _customerDataReceiveLoopThread.join();
    }

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

private:
    OS::UdpServerSocket _socket;
    CustomerDataInterface _customerDataInterface;
    std::thread _customerDataReceiveLoopThread;
    bool _stopCustomerDataUdpReceiver = false;
    unsigned short _customerDataPort = 0;

    void initCustomerDataUdpReceiver();
    CustomerDataInterface lastReceivedCustomerData();
    void customerDataReceiveLoop();
    void unblockCustomerDataUdpReceiverThread();
};

#endif // CAMERAS_MANAGER_H
