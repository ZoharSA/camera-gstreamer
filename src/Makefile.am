lib_LTLIBRARIES=gstreamer.la
noinst_PROGRAMS=test_link

CPPFLAGS+=$(GS_CFLAGS) $(GSBASE_CFLAGS) $(GSVIDEO_CFLAGS) $(GSAPP_CFLAGS)
LDFLAGS+=$(GS_LIBS) $(GSBASE_LIBS) $(GSVIDEO_LIBS) $(GSAPP_LIBS)

gstreamer_la_LDFLAGS = -module -avoid-version
gstreamer_la_SOURCES = CamerasManager.cpp camera-interface.cpp CameraGstreamer.cpp

test_link_SOURCES=test_link.cpp
test_link_LDADD=gstreamer.la
