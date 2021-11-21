#include <CameraGstreamer.h>
#include <CamerasManager.h>
#include <assert.h>
#include <cstring>

const std::unordered_map< std::string, void (CameraGstreamer::*)( const char *key, const char *value ) > CameraGstreamer::optionHandlers
{
    {"loop", &CameraGstreamer::optionLoop},
    {"trigger", &CameraGstreamer::optionTrigger}
};

gboolean CameraGstreamer::busCallback( GstBus *bus, GstMessage *msg, gpointer data) {
    GError *err = NULL;
    gchar *debug = NULL;

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
            break;
    }
    return TRUE;
}

CameraGstreamer::CameraGstreamer( CameraId id,
        const DUCameraDescriptor description,
        CamerasManager *manager,
        unsigned framesPerSecond ):
    _cameraId( id ),
    _manager( manager ),
    _readyToUseBuffer ( -1 ),
    _framesPerSecond( framesPerSecond ),
    _noSignal( false ),
    _gstreamPipeline( description.id ),
    _isRunning( false )
{
    assert( _framesPerSecond > 0 );
    assert( description.width > 0 );
    assert( description.height > 0 );

    const unsigned bufferSize = getBufferSize( description.format, description.width*description.height );
    allocateFrameBufferPool( bufferSize );

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
    for (unsigned i = 0; i < RING; ++i) {
        assert(_frameBufferPool[i].buffer == nullptr);
        _frameBufferPool[i].buffer = new uint8_t[bufferSize];
    }
}

void CameraGstreamer::releaseBufferPool() {
    for (unsigned i = 0; i < RING; ++i) {
        if (_frameBufferPool[i].buffer != nullptr) {
            delete[] _frameBufferPool[i].buffer;
            _frameBufferPool[i].buffer = nullptr;
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
    if( _threadAlignToFpsId == -1 ) {
        _alignToFpsThread = std::thread([&]{alignToFps();});
    }
}

void CameraGstreamer::stop() {
    stopPipeline();
    if( _alignToFpsThread.joinable() ) {
        _alignToFpsThread.join();
    }
    _threadAlignToFpsId = -1;
}

GstFlowReturn CameraGstreamer::onEos( GstElement *element, gpointer user_data ) {
    assert(user_data);
    CameraGstreamer *cameraGstreamer = static_cast<CameraGstreamer*>(user_data);
    std::cout << "Received 'eos' signal, setting pipeline to restart.\n";
    cameraGstreamer->setRestartPipeline();

    return GST_FLOW_OK;
}

void CameraGstreamer::playGetVideoPackets() {
    if ( !_isRunning ) {
        std::stringstream s;
        s << _gstreamPipeline;
        createPipeline(s.str());
        GstElement *appsink = gst_bin_get_by_name( GST_BIN(this->current_pipeline), "appsink" );
        g_signal_connect( appsink, "new_sample", G_CALLBACK(CameraGstreamer::onNewSample), this );
        if( _loop ) {
            g_signal_connect( appsink, "eos", G_CALLBACK(onEos), this );
        }
        startPipeline();
    }
}


void CameraGstreamer::startPipeline() {
    if ( !_isRunning && this->current_pipeline != NULL ) {
        std::cout << "Start pipeline,  camera id: " << _cameraId << std::endl;
        gst_element_set_state( this->current_pipeline, GST_STATE_PLAYING );
        _isRunning = true;
    }
}

void CameraGstreamer::stopPipeline() {
    if ( _isRunning && this->current_pipeline != NULL ) {
        std::cout << "Stop pipeline, camera id: " << _cameraId << std::endl;
        gst_element_set_state( this->current_pipeline, GST_STATE_NULL );
        this->current_pipeline = NULL;
        _isRunning = false;
    }
}

void CameraGstreamer::pausePipeline() {
    if ( this->current_pipeline != NULL ) {
        gst_element_set_state( this->current_pipeline, GST_STATE_PAUSED );
        _isRunning = false;
    }
}

void CameraGstreamer::resumePipeline() {
    if ( this->current_pipeline != NULL ) {
        if ( gst_element_set_state( this->current_pipeline, GST_STATE_PLAYING ) != GST_STATE_CHANGE_FAILURE ) {
            _isRunning = true;
        }
    }
}

void CameraGstreamer::createPipeline( std::string pipe ) {
    GError *e = NULL;
    this->current_pipeline = gst_parse_launch( pipe.c_str(), &e );
    if ( e != NULL || this->current_pipeline == NULL ) {
        std::cout << "[ERROR] Failed to run pipeline: \n ~~~ " << pipe << "\n"
                  << "[Error]: " << e->message << std::endl;
        throw std::runtime_error(e->message);
    }
    std::cout << "Running pipeline: \n ~~~ " << pipe << std::endl;
    GstBus *bus = gst_element_get_bus( this->current_pipeline );
    gst_bus_add_watch( bus, CameraGstreamer::busCallback, this );
    gst_object_unref( bus );
}

void CameraGstreamer::onVideoFrame( GstVideoFrame *frame ) {
    int currBufferIndex = (_readyToUseBuffer + 1) % RING;

    _lastCapturedTimestamp = std::chrono::steady_clock::now();
    std::memcpy( &_frameBufferPool[currBufferIndex].buffer[0], GST_VIDEO_FRAME_PLANE_DATA(frame, 0), GST_VIDEO_FRAME_SIZE(frame) );
    _frameBufferPool[currBufferIndex].bufferSize = GST_VIDEO_FRAME_SIZE(frame);
    _frameBufferPool[currBufferIndex].timestamp = std::chrono::steady_clock::now();
    _frameBufferPool[currBufferIndex].frameIndex = ++_frameIndex; // frame->id;
    _readyToUseBuffer =  currBufferIndex;
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

void CameraGstreamer::alignToFps()
{
    _threadAlignToFpsId = syscall( SYS_gettid );
    unsigned long long timeBetweenFramesInUS = 1000000 / _framesPerSecond;
    unsigned long long noSignalTimeOutInUs = 300000;
    while ( _isRunning ) {
        unsigned long long curr_duration = DurationUS(std::chrono::steady_clock::now() -  _lastCapturedTimestamp).count();
        if( _restartPipeline ) {
            stopPipeline();
            playGetVideoPackets();
            _restartPipeline = false;
            continue;
        }
        if( !_noSignal && curr_duration >= noSignalTimeOutInUs ){
            _manager->badCamera( _cameraId );
            _noSignal = true;
        }
        else if ( _noSignal && curr_duration < noSignalTimeOutInUs ){
            _manager->goodCamera( _cameraId );
            _noSignal = false;
        }
        if( !_noSignal ){
            int usingBuffer = _readyToUseBuffer;
            if ( usingBuffer >= 0 ){
                   _manager->frameGrabbed( _cameraId,
                        static_cast<const uint8_t *>(&_frameBufferPool[usingBuffer].buffer[0]),
                        _frameBufferPool[usingBuffer].bufferSize,
                        _frameBufferPool[usingBuffer].timestamp,
                        _frameBufferPool[usingBuffer].frameIndex,
                        _trigger );
             }
        }
        usleep(timeBetweenFramesInUS);
    }

}

unsigned CameraGstreamer::getBufferSize( DUFrameFormat format, unsigned size )
{
    assert( format == DUFrameFormat::RGBA or format == DUFrameFormat::YUV420 or format == DUFrameFormat::NV12
        or format == DUFrameFormat::YUV422 or format == DUFrameFormat::Greyscale8 or format == DUFrameFormat::RGB );
    switch( format ) {
        case DUFrameFormat::NV12:
        case DUFrameFormat::YUV420:
            return size * 3 / 2;
        case DUFrameFormat::RGBA:
            return size * 4;
        case DUFrameFormat::Greyscale8:
            return size;
        case DUFrameFormat::YUV422:
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
