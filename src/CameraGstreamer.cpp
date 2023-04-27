#include <CameraGstreamer.h>
#include <CamerasManager.h>
#include <assert.h>
#include <cstring>

const std::unordered_map< std::string, void (CameraGstreamer::*)( const char *key, const char *value ) > CameraGstreamer::optionHandlers
{
    {"loop", &CameraGstreamer::optionLoop},
    {"trigger", &CameraGstreamer::optionTrigger}
};

gboolean CameraGstreamer::busCallback( GstBus *bus, GstMessage *msg, gpointer unusedUserData) {
    GError *err = NULL;
    gchar *debug = NULL;

    std::cout << "Bus callback called with message of type: " << GST_MESSAGE_TYPE_NAME(msg) << std::endl;

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("got EOS\n");
            break;
        case GST_MESSAGE_WARNING:
            gst_message_parse_warning(msg, &err, &debug);
            g_print("[WARNING] %s\n%s\n", err->message, debug);
            break;
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(msg, &err, &debug);
            g_print("[ERROR] %s\n%s\n", err->message, debug);
        default:
            std::cout << "Bus callback called with unhandled message type: " << GST_MESSAGE_TYPE_NAME(msg) << std::endl;
            break;
    }
    return TRUE;
}

CameraGstreamer::CameraGstreamer( CameraId id,
        const DUCameraDescriptor description,
        CamerasManager *manager,
        unsigned framesPerSecond ):
    _bufferSize( getBufferSize( description.format, description.width*description.height ) ),
    _cameraId( id ),
    _manager( manager ),
    _readyToUseBuffer ( -1 ),
    _framesPerSecond( framesPerSecond ),
    _noSignal( true ),
    _gstreamPipeline( description.id ),
    _isRunning( false ),
    _pipelineFailed( false )
{
    assert( _framesPerSecond > 0 );
    assert( description.width > 0 );
    assert( description.height > 0 );

    allocateFrameBufferPool( _bufferSize );

    handleCustomParameters( description.customParameters, description.numCustomParameters );

    init();
}

void CameraGstreamer::handleCustomParameters( const DUCustomCameraParameter * customParameters, uint8_t numCustomParameters) {
    std::cout << "[camera "<<_cameraId<<"] Handling custom parameters. Number of parameters: " << unsigned(numCustomParameters) << std::endl;
    for( unsigned i=0; i<numCustomParameters; ++i ) {
        const char * const key = customParameters[i].key;
        const char * const value = customParameters[i].value;
        std::cout << "[camera "<<_cameraId<<"] Handling custom parameter key:value ("<<key<<":"<<value<<")" << std::endl;
        auto handler = optionHandlers.find(key);
        if( handler == optionHandlers.end() ) {
            std::ostringstream errorMsg;
            errorMsg<<"[ERROR] GStreamer Camera "<<_cameraId<<" passed unsupported custom parameter \""<<key<<"\"" << std::endl;
            std::cerr << errorMsg.str();
            throw std::runtime_error( errorMsg.str() );
        }

        (this->*(handler->second))( key, value );
    }

}

void CameraGstreamer::allocateFrameBufferPool(unsigned bufferSize) {
    std::cout << "Allocating " << RING << " buffers of " << bufferSize << " bytes each" << std::endl;
    for (unsigned i = 0; i < RING; ++i) {
        assert(_frameBufferPool[i] == nullptr);
        _frameBufferPool[i] = new uint8_t[bufferSize];
    }
}

void CameraGstreamer::releaseBufferPool() {
    for (unsigned i = 0; i < RING; ++i) {
        if (_frameBufferPool[i] != nullptr) {
            delete[] _frameBufferPool[i];
            _frameBufferPool[i] = nullptr;
        }
    }
}

CameraGstreamer::~CameraGstreamer() {
    stop();
    releaseBufferPool();
};

void CameraGstreamer::init() {
    gst_init(NULL, NULL);
    this->loop = g_main_loop_new(NULL, false);
}

void CameraGstreamer::start() {
    playGetVideoPackets();
    if( _threadCheckPipelineStateId == -1 ) {
        _checkPipelineStateThread = std::thread([&]{checkPipelineState();});
    }
}

void CameraGstreamer::stop() {
    stopPipeline();
    stopCheckPipelineStateThread();
}

void CameraGstreamer::stopCheckPipelineStateThread() {
    if( _checkPipelineStateThread.joinable() ) {
        _checkPipelineStateThread.join();
    }
    _threadCheckPipelineStateId = -1;
}

GstFlowReturn CameraGstreamer::onEos( GstElement *element, gpointer user_data ) {
    assert(user_data);
    CameraGstreamer *cameraGstreamer = static_cast<CameraGstreamer*>(user_data);
    std::cout << "Received 'eos' signal, setting pipeline to restart.\n";
    cameraGstreamer->setRestartPipeline();

    return GST_FLOW_OK;
}
static GstPadProbeReturn startTimestamp_srcpad_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
    CameraGstreamer *self = (CameraGstreamer *)user_data;
    self->getStartTimestamp();
    return GST_PAD_PROBE_OK;
}

void CameraGstreamer::playGetVideoPackets() {
    if ( !_isRunning ) {
        createPipeline();
        addBusWatch();
        GstElement *appsink = gst_bin_get_by_name( GST_BIN(_currentPipelineElement), "appsink" );
        if ( appsink ) {
            g_signal_connect( appsink, "new_sample", G_CALLBACK(CameraGstreamer::onNewSample), this );
            if ( _loop ) {
                g_signal_connect( appsink, "eos", G_CALLBACK(onEos), this );
            }
        } else {
            std::cout << "[ERROR] [camera " << _cameraId << "] Failed to find 'appsink' in pipeline." << std::endl;
            _manager->badPipeline(_cameraId);
        }
        GstElement *startTimestamp = gst_bin_get_by_name(GST_BIN(_currentPipelineElement), "start_timestamp");
        if ( startTimestamp ) {
            std::cout << "[camera " << _cameraId << "] Found start_timestamp in pipeline." << std::endl;
            GstPad *startTimestamp_srcpad = gst_element_get_static_pad(startTimestamp, "src");
            gst_pad_add_probe(startTimestamp_srcpad, GST_PAD_PROBE_TYPE_BUFFER, startTimestamp_srcpad_probe, this, NULL);
        }
        startPipeline();
    }
    else {
        std::cout << "[camera "<<_cameraId<<"] playGetVideoPackets called, but checkPipelineState thread is already running." << std::endl;
    }
}


void CameraGstreamer::startPipeline() {
    if ( !_isRunning && _currentPipelineElement != NULL ) {
        std::cout << "Start pipeline,  camera id: " << _cameraId << std::endl;
        gst_element_set_state( _currentPipelineElement, GST_STATE_PLAYING );
        _isRunning = true;
        _noSignal = false;
    }
}

void CameraGstreamer::stopPipeline() {
    if ( _isRunning && _currentPipelineElement != NULL ) {
        std::cout << "Stop pipeline, camera id: " << _cameraId << " startTimestampQueue size: " << _startTimestampQueue.size() << std::endl;
        gst_element_set_state( _currentPipelineElement, GST_STATE_NULL );
        gst_object_unref(_currentPipelineElement);
        removeBusWatch();
        _currentPipelineElement = NULL;
        _isRunning = false;
    }
}

void CameraGstreamer::pausePipeline() {
    if ( _currentPipelineElement != NULL ) {
        gst_element_set_state( _currentPipelineElement, GST_STATE_PAUSED );
        _isRunning = false;
    }
}

void CameraGstreamer::resumePipeline() {
    if ( _currentPipelineElement != NULL ) {
        if ( gst_element_set_state( _currentPipelineElement, GST_STATE_PLAYING ) != GST_STATE_CHANGE_FAILURE ) {
            _isRunning = true;
        }
    }
}

void CameraGstreamer::createPipeline() {
    assert(_currentPipelineElement == nullptr);
    GError *e = NULL;
    const std::string pipe = _gstreamPipeline;
    _currentPipelineElement = gst_parse_launch( pipe.c_str(), &e );
    if ( e != NULL || _currentPipelineElement == NULL ) {
        std::cout << "[ERROR] Failed to run pipeline: \n ~~~ " << pipe << "\n"
                  << "[Error]: " << e->message << std::endl;
        throw std::runtime_error(e->message);
    }
    std::cout << "Running pipeline: \n ~~~ " << pipe << std::endl;
    assert(_currentPipelineElement != nullptr);
}

void CameraGstreamer::addBusWatch() {
    assert(_currentPipelineElement != nullptr);
    assert(_busWatchId == INVALID_BUS_WATCH_ID);

    GstBus *bus = gst_element_get_bus( _currentPipelineElement );
    constexpr auto UNUSED_USER_DATA = nullptr;
    _busWatchId = gst_bus_add_watch(bus, busCallback, UNUSED_USER_DATA);
    assert(_busWatchId != INVALID_BUS_WATCH_ID);
    gst_object_unref( bus );
}

void CameraGstreamer::removeBusWatch() {
    assert( _busWatchId != INVALID_BUS_WATCH_ID );
    g_source_remove(_busWatchId);
    _busWatchId = INVALID_BUS_WATCH_ID;
}

void CameraGstreamer::onVideoFrame( GstVideoFrame *frame ) {
    int currBufferIndex = (_readyToUseBuffer + 1) % RING;

    _lastCapturedTimestamp = std::chrono::steady_clock::now();

    const size_t frameSize = GST_VIDEO_FRAME_SIZE(frame);
    if ( _bufferSize != frameSize ) {
        std::cerr << "[camera "<<_cameraId<<"] CRITICAL ERROR: Received a frame of size " << frameSize << \
                     " while allocated buffer size is" << _bufferSize << " bytes. " << \
                     "Please check configured sizes in camers.yaml. Aborting execution." << std::endl;
        assert( ("Received frame size doesn't match allocated buffer size.", false) );
    }

    std::memcpy(_frameBufferPool[currBufferIndex], GST_VIDEO_FRAME_PLANE_DATA(frame, 0), GST_VIDEO_FRAME_SIZE(frame) );

    StartTimestamp startTimestamp;
    if (_startTimestampQueue.size() != 0) {
        startTimestamp = _startTimestampQueue.front();
        if (startTimestamp.frameIndex != _appSinkFrameIndex) {
            std::cout << "Lost frames in pipline. start timestamp frame index: " << startTimestamp.frameIndex <<" sink timestamp index: "  << _appSinkFrameIndex << std::endl;
        }
        _startTimestampQueue.pop();
        _noStartTimestamp = false;
    } else if (_noStartTimestamp == false) {
        std::cout << "[camera "<<_cameraId<<"] no start_timestamp name was set." << std::endl;
        _noStartTimestamp = true;
    }

    _manager->frameGrabbed( _cameraId,
    _frameBufferPool[currBufferIndex],
    GST_VIDEO_FRAME_SIZE(frame),
    startTimestamp.timestamp,
    _appSinkFrameIndex,
    _trigger );

    _appSinkFrameIndex++;
    _readyToUseBuffer = currBufferIndex;
}

void CameraGstreamer::getStartTimestamp() {
    StartTimestamp startTimestamp;
    startTimestamp.timestamp = std::chrono::steady_clock::now();
    startTimestamp.frameIndex = _startTimestampFrameIndex++;
    _startTimestampQueue.push(startTimestamp);
}

bool CameraGstreamer::isActive() {
    std::cout << "isActive. cameraId: "<<_cameraId <<" _isRunning: " << _isRunning << " , _noSignal: " << _noSignal << std::endl;
    return _isRunning & !_noSignal;
}

GstFlowReturn CameraGstreamer::onNewSample( GstElement *element, gpointer user_data ) {
    CameraGstreamer *self = (CameraGstreamer *)user_data;
    GstAppSink *appsink = (GstAppSink *)element;

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (sample == NULL) return GST_FLOW_ERROR;

    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (caps == NULL || buffer == NULL) return GST_FLOW_ERROR;

    GstVideoInfo info;
    if (!gst_video_info_from_caps(&info, caps)) return GST_FLOW_ERROR;

    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) return GST_FLOW_ERROR;

    self->onVideoFrame(&frame);

    gst_video_frame_unmap(&frame);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

void CameraGstreamer::restartPipeline() {
    std::cout << "[camera "<<_cameraId<<"] Restarting pipeline." << std::endl;
    stopPipeline();
    playGetVideoPackets();
    _lastPipelineRestartTimestamp = std::chrono::steady_clock::now();
    std::cout << "[camera "<<_cameraId<<"] Finished restarting pipeline." << std::endl;
}

microseconds CameraGstreamer::timeSinceLastRestartPipelineUS() const {
    return DurationUS(std::chrono::steady_clock::now() - _lastPipelineRestartTimestamp).count();
}

bool CameraGstreamer::shouldRestartPipelineFromNoSignal() const {
    return ( _pipelineFailed && (timeSinceLastRestartPipelineUS() >= RESTART_PIPELINE_TIMEOUT_IN_US) );
}

void CameraGstreamer::checkPipelineState()
{
    _threadCheckPipelineStateId = syscall( SYS_gettid );
    const microseconds timeBetweenFramesInUS = 1'000'000 / _framesPerSecond;
    _lastCapturedTimestamp = std::chrono::steady_clock::now();

    while ( _isRunning ) {
        const microseconds curr_duration = DurationUS(std::chrono::steady_clock::now() -  _lastCapturedTimestamp).count();
        if( _restartPipeline ) {
            restartPipeline();
            continue;
        }
        if( !_noSignal && curr_duration >= NO_SIGNAL_TIMEOUT_IN_US ){
            std::cout << "[camera "<<_cameraId<<"] No frame captured in last "<<curr_duration<<"us (no-signal timeout is "<<NO_SIGNAL_TIMEOUT_IN_US<<"us)." << std::endl;
            _noSignal = true;
            _manager->badCamera( _cameraId );
        }
        else if ( shouldRestartPipelineFromNoSignal() ) {
            restartPipeline();
            continue;
        }
        else if ( _noSignal && curr_duration < NO_SIGNAL_TIMEOUT_IN_US ){
            std::cout << "[camera "<<_cameraId<<"] Frame successfully captured - camera recovered from no-signal." << std::endl;
            _noSignal = false;
            _manager->goodCamera( _cameraId );
            _pipelineFailed = false;
        }
        // if ( ( _noSignal ) && ( !_pipelineFailed and pipelineFailure() ) ){
        //      std::cout << "[camera "<<_cameraId<<"]  Pipline state changed to FAILURE." <<std::endl;
        //      restartPipeline();
        //      _pipelineFailed = true;
        // }
        usleep(timeBetweenFramesInUS);
    }
    std::cout << "[camera "<<_cameraId<<"] checkPipelineState thread finished running." << std::endl;
}

GstStateChangeReturn CameraGstreamer::getPipelineStateChangeReturn() const {
    return gst_element_get_state( _currentPipelineElement, NULL, NULL, -1) ;
}

bool CameraGstreamer::pipelineFailure() const {
    // NOTE: I experimented and saw that when a camera disconnects,
    // the state stays stuck on GST_STATE_PLAYING (4),
    // but the GstStateChangeReturn switches to GST_STATE_CHANGE_FAILURE (0).
    // So using the state_change failure as an indication for camera failure
    return getPipelineStateChangeReturn() == GST_STATE_CHANGE_FAILURE;
}

unsigned CameraGstreamer::getBufferSize( DUFrameFormat format, unsigned size )
{
    assert( format == DUFrameFormat::RGBA or format == DUFrameFormat::YUV420 or format == DUFrameFormat::NV12
            or format == DUFrameFormat::YUV422 or format == DUFrameFormat::Greyscale8 or format == DUFrameFormat::RGB or format == DUFrameFormat::UYVY422 );
    switch( format ) {
        case DUFrameFormat::NV12:
        case DUFrameFormat::YUV420:
            return size * 3 / 2;
        case DUFrameFormat::RGBA:
            return size * 4;
        case DUFrameFormat::Greyscale8:
            return size;
        case DUFrameFormat::YUV422:
        case DUFrameFormat::UYVY422:
            return size * 2;
        case DUFrameFormat::RGB:
            return size * 3;
        default:
            abort();
    }
}

void CameraGstreamer::optionLoop( const char *key, const char *rawValue ) {
    bool value = parseBoolValue(key, rawValue);
    _loop = value;
}
void CameraGstreamer::optionTrigger( const char *key, const char *rawValue ) {
    bool value = parseBoolValue(key, rawValue);
    _trigger = value;
}

bool CameraGstreamer::parseBoolValue( const char *key, const char *value ) {
    using namespace std::string_literals;

    if( value == "true"s ) {
        return true;
    } else if( value == "false"s ) {
        return false;
    }

    std::ostringstream errorMsg;
    errorMsg << "GStreamer Camera - Boolean value for custom parameter \""<<key<<"\" must be either \"true\" or \"false\". Not \""<<
            value<<"\"";
    throw std::runtime_error( errorMsg.str() );
}
