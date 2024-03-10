#include <CamerasManager.h>
#include <camera-interface.h>
#include <UdpServerSocket.h>

#include <iostream>
#include <string>
#include <assert.h>
#include <string.h>
#include <functional>

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
                _customerDataPort = _cameras[id]->getCustomerDataPort();
                _socket.bind(_customerDataPort);
            }
        }
        if ( !foundTriggerCamera ) {
            _cameras[ DEFAULT_TRIGGER_CAMERA ]->setTrigger( true );
            _customerDataPort = _cameras[DEFAULT_TRIGGER_CAMERA]->getCustomerDataPort();
            _socket.bind(_customerDataPort);
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
        .customerData = CamerasManager::lastReceivedCustomerData(),
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

void CamerasManager::badPipeline( CameraId id )
{
    _cameraStateChanged( id, 10001, _opaq );
}

CustomerDataInterface CamerasManager::lastReceivedCustomerData()
{
    return _customerDataInterface;
}

void CamerasManager::initCustomerDataUdpReceiver()
{
    _customerDataReceiveLoopThread = std::thread(
        [&]() {
            while(!_stopCustomerDataUdpReceiver) {
                OS::UdpServerSocket::PointerLength buffer{&_customerDataInterface, sizeof(_customerDataInterface)};
                _socket.receive(buffer);
            }
        }
    );
    return;
}

void CamerasManager::unblockCustomerDataUdpReceiverThread()
{
    int fd( socket(AF_INET, SOCK_DGRAM, 0) );
    if (fd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(_customerDataPort);
    address.sin_addr.s_addr = INADDR_ANY;

    const char *emptyMessage = "";
    sendto(
        fd,
        (const char *)emptyMessage,
        strlen(emptyMessage),
        MSG_CONFIRM,
        (const struct sockaddr *) &address,
        sizeof(address)
    );
    close(fd);
    return;
}
