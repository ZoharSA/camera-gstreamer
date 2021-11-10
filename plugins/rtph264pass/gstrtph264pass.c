/*
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the
 * GNU Lesser General Public License Version 2.1 (the "LGPL"), in
 * which case the following provisions apply instead of the ones
 * mentioned above:
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * SECTION:element-rtph264pass
 *
 * FIXME:Describe rtph264pass here.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v -m fakesrc ! rtph264pass ! fakesink silent=TRUE
 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <inttypes.h>

#include "gstrtph264pass.h"
GST_DEBUG_CATEGORY_STATIC(gst_rtph264pass_debug);
#define GST_CAT_DEFAULT gst_rtph264pass_debug

/* Filter signals and args */
enum {
    /* FILL ME */
    LAST_SIGNAL
};

enum {
    PROP_0,
    PROP_ENABLE
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory =
    GST_STATIC_PAD_TEMPLATE("sink", GST_PAD_SINK, GST_PAD_ALWAYS,
                            GST_STATIC_CAPS("application/x-rtp"));

// static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE("sink",
//                                                                    GST_PAD_SINK,
//                                                                    GST_PAD_ALWAYS,
//                                                                    GST_STATIC_CAPS("ANY"));

static GstStaticPadTemplate src_factory =
    GST_STATIC_PAD_TEMPLATE("src", GST_PAD_SRC, GST_PAD_ALWAYS,
                            GST_STATIC_CAPS("video/x-h264, "
                                            "stream-format=byte-stream"));

// static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE("src",
//                                                                   GST_PAD_SRC,
//                                                                   GST_PAD_ALWAYS,
//                                                                   GST_STATIC_CAPS("ANY"));

#define gst_rtph264pass_parent_class parent_class
G_DEFINE_TYPE(Gstrtph264pass, gst_rtph264pass, GST_TYPE_ELEMENT);

static void gst_rtph264pass_set_property(GObject *object, guint prop_id,
                                         const GValue *value, GParamSpec *pspec);
static void gst_rtph264pass_get_property(GObject *object, guint prop_id,
                                         GValue *value, GParamSpec *pspec);

static gboolean gst_rtph264pass_sink_event(GstPad *pad, GstObject *parent, GstEvent *event);
static GstFlowReturn gst_rtph264pass_chain(GstPad *pad, GstObject *parent, GstBuffer *buf);

/* GObject vmethod implementations */

/* initialize the rtph264pass's class */
static void
gst_rtph264pass_class_init(Gstrtph264passClass *klass) {
    GObjectClass *gobject_class;
    GstElementClass *gstelement_class;

    gobject_class = (GObjectClass *)klass;
    gstelement_class = (GstElementClass *)klass;

    gobject_class->set_property = gst_rtph264pass_set_property;
    gobject_class->get_property = gst_rtph264pass_get_property;

    gst_element_class_set_details_simple(gstelement_class,
                                         "rtph264pass",
                                         "FIXME:Generic",
                                         "FIXME:Generic Template Element",
                                         "Pavel Shvarchov <<pavel@dsp-ip.com>>");

    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&src_factory));
    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&sink_factory));
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void
gst_rtph264pass_init(Gstrtph264pass *filter) {
    filter->sinkpad = gst_pad_new_from_static_template(&sink_factory, "sink");
    gst_pad_set_event_function(filter->sinkpad,
                               GST_DEBUG_FUNCPTR(gst_rtph264pass_sink_event));
    gst_pad_set_chain_function(filter->sinkpad,
                               GST_DEBUG_FUNCPTR(gst_rtph264pass_chain));
    // GST_PAD_SET_PROXY_CAPS(filter->sinkpad);
    gst_element_add_pad(GST_ELEMENT(filter), filter->sinkpad);

    filter->srcpad = gst_pad_new_from_static_template(&src_factory, "src");
    // GST_PAD_SET_PROXY_CAPS(filter->srcpad);
    gst_element_add_pad(GST_ELEMENT(filter), filter->srcpad);
}

static void
gst_rtph264pass_set_property(GObject *object, guint prop_id,
                             const GValue *value, GParamSpec *pspec) {
    Gstrtph264pass *filter = GST_RTPH264PASS(object);

    switch (prop_id) {
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static void
gst_rtph264pass_get_property(GObject *object, guint prop_id,
                             GValue *value, GParamSpec *pspec) {
    Gstrtph264pass *filter = GST_RTPH264PASS(object);

    switch (prop_id) {
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

/* GstElement vmethod implementations */

/* this function handles sink events */
static gboolean
gst_rtph264pass_sink_event(GstPad *pad, GstObject *parent, GstEvent *event) {
    Gstrtph264pass *filter;
    gboolean ret;

    filter = GST_RTPH264PASS(parent);

    GST_LOG_OBJECT(filter, "Received %s event: %" GST_PTR_FORMAT,
                   GST_EVENT_TYPE_NAME(event), event);

    switch (GST_EVENT_TYPE(event)) {
        case GST_EVENT_CAPS: {
            GstCaps *caps;

            gst_event_parse_caps(event, &caps);
            /* do something with the caps */

            /* and forward */
            ret = gst_pad_event_default(pad, parent, event);
            break;
        }
        default:
            ret = gst_pad_event_default(pad, parent, event);
            break;
    }
    return ret;
}

static GstFlowReturn
gst_rtph264pass_chain(GstPad *pad, GstObject *parent, GstBuffer *buf) {
    Gstrtph264pass *filter = GST_RTPH264PASS(parent);
    GstRTPBuffer rtpbuffer = GST_RTP_BUFFER_INIT;
    gst_rtp_buffer_map(buf, GST_MAP_READ, &rtpbuffer);
    GstBuffer *pbuf = gst_rtp_buffer_get_payload_buffer(&rtpbuffer);
    gst_rtp_buffer_unmap(&rtpbuffer);
    pbuf->pts = buf->pts;

    // this is to clone the buffer and send it forwards.
    // GstMapInfo info;
    // gst_buffer_map(pbuf, &info, GST_MAP_READ);

    // GstBuffer *next = gst_buffer_new_allocate(NULL, info.size, NULL);
    // GstMapInfo next_info;

    // gst_buffer_map(next, &next_info, GST_MAP_READWRITE);
    // memcpy(next_info.data, info.data, info.size);

    // gst_buffer_unmap(next, &next_info);
    // gst_buffer_unmap(pbuf, &info);
    // next->pts = buf->pts;

    return gst_pad_push(filter->srcpad, pbuf);
    // return gst_pad_push(filter->srcpad, next);
}

/* entry point to initialize the plug-in
 * initialize the plug-in itself
 * register the element factories and other features
 */
static gboolean
rtph264pass_init(GstPlugin *rtph264pass) {
    /* debug category for fltering log messages
   *
   * exchange the string 'Template rtph264pass' with your description
   */
    GST_DEBUG_CATEGORY_INIT(gst_rtph264pass_debug, "rtph264pass",
                            0, "Template rtph264pass");

    return gst_element_register(rtph264pass, "rtph264pass", GST_RANK_NONE,
                                GST_TYPE_RTPH264PASS);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "myfirstrtph264pass"
#endif

/* gstreamer looks for this structure to register rtph264passs
 *
 * exchange the string 'Template rtph264pass' with your rtph264pass description
 */
GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    rtph264pass,
    "Template rtph264pass",
    rtph264pass_init,
    VERSION,
    "LGPL",
    "GStreamer",
    "http://gstreamer.net/")
