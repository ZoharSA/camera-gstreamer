#include <camera-interface.h>

int main() {
    void * volatile functions[] = {
        (void*)duLibraryVersion,
        (void*)duCamerasInit,
        (void*)duCamerasReleaseFrame,
        (void*)duCamerasShutdown,
        (void*)duCameraEnable,
        (void*)duCameraDisable
    };

    // Hide "unused variable" warning
    functions;
}
