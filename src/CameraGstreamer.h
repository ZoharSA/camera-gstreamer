#pragma once

#include <camera-interface.h>

#include <gst/app/app.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/syscall.h>
#include <unordered_map>
#include <atomic>
#include <queue>

constexpr const unsigned RING = 3;

typedef std::chrono::duration<double, std::micro> DurationUS;
using time_point = std::chrono::steady_clock::time_point;
using microseconds = unsigned long long;

class CamerasManager;

class CameraGstreamer {
public:
    CameraGstreamer(
          CameraId id,
          const DUCameraDescriptor description,
          CamerasManager *manager,
          unsigned framesPerSecond );
    ~CameraGstreamer();

    CameraGstreamer(const CameraGstreamer &) = delete;
    CameraGstreamer& operator=(const CameraGstreamer &) = delete;

    void init();
    void start();
    void stop();
    void stopCheckPipelineStateThread();
    void playGetVideoPackets();
    bool getIsTrigger() const { return _trigger; }
    void setTrigger(bool trigger){ _trigger = trigger; }
    void getStartTimestamp();
    bool isActive();
private:
    static constexpr microseconds NO_SIGNAL_TIMEOUT_IN_US = 300'000;
    static constexpr microseconds RESTART_PIPELINE_TIMEOUT_IN_US = 2'000'000;
    static constexpr guint INVALID_BUS_WATCH_ID = 0;

    struct FrameBuffer{
          uint8_t *buffer     = nullptr;
          int bufferSize      = 0;
          time_point timestamp;
          uint32_t frameIndex = 0;
    };

    struct StartTimestamp{
        time_point timestamp;
        unsigned long long frameIndex = 0;
    };

    GMainLoop *loop = NULL;
    GstElement *_currentPipelineElement = NULL;

    void startPipeline();
    void stopPipeline();
    void pausePipeline();
    void resumePipeline();
    void createPipeline();
    static gboolean busCallback( GstBus *bus, GstMessage *msg, gpointer data );
    static GstFlowReturn onEos( GstElement *element, gpointer user_data );
    static GstFlowReturn onNewSample( GstElement *element, gpointer user_data );
    void onVideoFrame( GstVideoFrame *frame );
    void checkPipelineState();
    static unsigned getBufferSize( DUFrameFormat format, unsigned size );
    void setRestartPipeline() { _restartPipeline = true; }
    void optionLoop( const char *key, const char *value );
    void optionTrigger( const char *key, const char *value );
    static bool parseBoolValue( const char *key, const char *value );
    void allocateFrameBufferPool(unsigned bufferSize);
    void releaseBufferPool();
    void handleCustomParameters( const DUCustomCameraParameter * customParameters, uint8_t numCustomParameters);
    void restartPipeline();
    microseconds timeSinceLastRestartPipelineUS() const;
    bool shouldRestartPipelineFromNoSignal() const;
    GstStateChangeReturn getPipelineStateChangeReturn() const;
    bool pipelineFailure() const;
    void addBusWatch();
    void removeBusWatch();
    static const std::unordered_map< std::string, void (CameraGstreamer::*)( const char *key, const char *value ) > optionHandlers;


    const unsigned int      _bufferSize;
    const unsigned int      _cameraId;
    CamerasManager * const   _manager;
    std::atomic<int>        _readyToUseBuffer;
    const unsigned int      _framesPerSecond;
    bool                    _noSignal               = false;
    uint8_t                 *_frameBufferPool[RING] = {nullptr};
    bool                    _noStartTimestamp       = false;
    time_point              _lastCapturedTimestamp;
    pid_t                   _threadCheckPipelineStateId     = -1;
    std::thread             _checkPipelineStateThread;
    const std::string       _gstreamPipeline;
    std::atomic<bool>       _restartPipeline        { false };
    bool                    _loop                   = false;
    std::atomic<bool>       _isRunning;
    bool                    _trigger                = false;
    uint32_t                _appSinkFrameIndex      = 0;
    time_point              _lastPipelineRestartTimestamp;
    std::queue<StartTimestamp>  _startTimestampQueue;
    bool                    _pipelineFailed;
    guint                   _busWatchId             = INVALID_BUS_WATCH_ID;
    unsigned long long      _startTimestampFrameIndex = 0;
};
