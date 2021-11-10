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

constexpr const unsigned RING = 3;

typedef std::chrono::duration<double, std::micro> DurationUS;

class CamerasManager;

class CameraGstreamer {
   public:
    CameraGstreamer(
          CameraId id,
          const DUCameraDescriptor description,
          CamerasManager *manager,
          unsigned framesPerSecond );
    virtual ~CameraGstreamer();
    void init();
    void start();
    void stop();
    void playGetVideoPackets();
    bool getIsTrigger(){ return _trigger; }
    void setTrigger(bool trigger){ _trigger = trigger; }

   private:

    struct frameBuffer{
          uint8_t *buffer;
          int bufferSize;
          std::chrono::steady_clock::time_point timestamp;
          uint32_t frameIndex;
          frameBuffer():buffer(nullptr){}
    };

    GMainLoop *loop = NULL;
    GstElement *current_pipeline = NULL;

    void startPipeline();
    void stopPipeline();
    void pausePipeline();
    void resumePipeline();
    void createPipeline( std::string pipeline );
    static gboolean busCallback( GstBus *bus, GstMessage *msg, gpointer data );
    static GstFlowReturn onEos( GstElement *element, gpointer user_data );
    static GstFlowReturn onNewSample( GstElement *element, gpointer user_data );
    void onVideoFrame( GstVideoFrame *frame );
    void alignToFps();
    unsigned getBufferSize( DUFrameFormat format, unsigned size );
    void setRestartPipeline() { _restartPipeline = true; }
    void optionLoop( const char *key, const char *value );
    void optionTrigger( const char *key, const char *value );
    void getPipelineValue( const char *key, const char *value );
    bool parseBoolValue( const char *key, const char *value );

    static const std::unordered_map< std::string, void (CameraGstreamer::*)( const char *key, const char *value ) > optionHandlers;


    unsigned int _cameraId;
    CamerasManager * _manager;
    std::atomic<int> _readyToUseBuffer;
    unsigned int _framesPerSecond;
    void* _opaq = nullptr;
    bool _noSignal = false;
    frameBuffer _frameBufferPool[RING];
    std::chrono::steady_clock::time_point _lastCapturedTimestamp;
    pid_t _threadAlignToFpsId = -1;
    std::thread _alignToFpsThread;
    std::string _gstreamPipeline;
    std::atomic<bool> _restartPipeline;
    bool _loop = false;
    std::atomic<bool> _isRunning;
    bool _trigger = false;
    uint32_t _frameIndex = 0;
};
