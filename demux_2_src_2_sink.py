#!/usr/bin/env python3

################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
################################################################################

import argparse
import sys
sys.path.append('../')

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GObject, Gst, GstRtspServer
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call

import pyds

PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_BICYCLE = 1
PGIE_CLASS_ID_PERSON = 2
PGIE_CLASS_ID_ROADSIGN = 3

def osd_sink_pad_buffer_probe(pad,info,u_data):
    frame_number=0
    #Intiallizing object counter with 0.
    obj_counter = {
        PGIE_CLASS_ID_VEHICLE:0,
        PGIE_CLASS_ID_PERSON:0,
        PGIE_CLASS_ID_BICYCLE:0,
        PGIE_CLASS_ID_ROADSIGN:0
    }
    num_rects=0

    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_number=frame_meta.frame_num
        num_rects = frame_meta.num_obj_meta
        l_obj=frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
            obj_counter[obj_meta.class_id] += 1
            try: 
                l_obj=l_obj.next
            except StopIteration:
                break

        # Acquiring a display meta object. The memory ownership remains in
        # the C code so downstream plugins can still access it. Otherwise
        # the garbage collector will claim it when this probe function exits.
        display_meta=pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]
        # Setting display text to be shown on screen
        # Note that the pyds module allocates a buffer for the string, and the
        # memory will not be claimed by the garbage collector.
        # Reading the display_text field here will return the C address of the
        # allocated string. Use pyds.get_string() to get the string content.
        py_nvosd_text_params.display_text = "Frame Number={} Number of Objects={} Vehicle_count={} Person_count={}".format(frame_number, num_rects, obj_counter[PGIE_CLASS_ID_VEHICLE], obj_counter[PGIE_CLASS_ID_PERSON])

        # Now set the offsets where the string should appear
        py_nvosd_text_params.x_offset = 10
        py_nvosd_text_params.y_offset = 12

        # Font , font-color and font-size
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 10
        # set(red, green, blue, alpha); set to White
        py_nvosd_text_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)

        # Text background color
        py_nvosd_text_params.set_bg_clr = 1
        # set(red, green, blue, alpha); set to Black
        py_nvosd_text_params.text_bg_clr.set(0.0, 0.0, 0.0, 1.0)
        # Using pyds.get_string() to get display_text as string
        print(pyds.get_string(py_nvosd_text_params.display_text))
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)
        try:
            l_frame=l_frame.next
        except StopIteration:
            break
			
    return Gst.PadProbeReturn.OK	


def cb_newpad(decodebin, decoder_src_pad,data):
    print("In cb_newpad\n")
    caps=decoder_src_pad.get_current_caps()
    gststruct=caps.get_structure(0)
    gstname=gststruct.get_name()
    source_bin=data
    features=caps.get_features(0)

    # Need to check if the pad created by the decodebin is for video and not
    # audio.
    print("gstname=",gstname)
    if(gstname.find("video")!=-1):
        # Link the decodebin pad only if decodebin has picked nvidia
        # decoder plugin nvdec_*. We do this by checking if the pad caps contain
        # NVMM memory features.
        print("features=",features)
        if features.contains("memory:NVMM"):
            # Get the source bin ghost pad
            bin_ghost_pad=source_bin.get_static_pad("src")
            if not bin_ghost_pad.set_target(decoder_src_pad):
                sys.stderr.write("Failed to link decoder src pad to source bin ghost pad\n")
        else:
            sys.stderr.write(" Error: Decodebin did not pick nvidia decoder plugin.\n")

def decodebin_child_added(child_proxy,Object,name,user_data):
    print("Decodebin child added:", name, "\n")
    if(name.find("decodebin") != -1):
        Object.connect("child-added",decodebin_child_added,user_data)   
    if(is_aarch64() and name.find("nvv4l2decoder") != -1):
        print("Seting bufapi_version\n")
        Object.set_property("bufapi-version",True)

def create_source_bin(index,uri):
    print("Creating source bin")

    # Create a source GstBin to abstract this bin's content from the rest of the
    # pipeline
    bin_name="source-bin-%02d" %index
    print(bin_name)
    nbin=Gst.Bin.new(bin_name)
    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")

    # Source element for reading from the uri.
    # We will use decodebin and let it figure out the container format of the
    # stream and the codec and plug the appropriate demux and decode plugins.
    uri_decode_bin=Gst.ElementFactory.make("uridecodebin", "uri-decode-bin")
    if not uri_decode_bin:
        sys.stderr.write(" Unable to create uri decode bin \n")
    # We set the input uri to the source element
    uri_decode_bin.set_property("uri",uri)
    # Connect to the "pad-added" signal of the decodebin which generates a
    # callback once a new pad for raw data has beed created by the decodebin
    uri_decode_bin.connect("pad-added",cb_newpad,nbin)
    uri_decode_bin.connect("child-added",decodebin_child_added,nbin)

    # We need to create a ghost pad for the source bin which will act as a proxy
    # for the video decoder src pad. The ghost pad will not have a target right
    # now. Once the decode bin creates the video decoder and generates the
    # cb_newpad callback, we will set the ghost pad target to the video decoder
    # src pad.
    Gst.Bin.add(nbin,uri_decode_bin)
    bin_pad=nbin.add_pad(Gst.GhostPad.new_no_target("src",Gst.PadDirection.SRC))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin


def main(args):
    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    pipeline = Gst.Pipeline()
    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")
    
    """
    # Source element for reading from the file
    print("Creating Source \n ")
    source = Gst.ElementFactory.make("filesrc", "file-source")
    if not source:
        sys.stderr.write(" Unable to create Source \n")
    
    # Since the data format in the input file is elementary h264 stream,
    # we need a h264parser
    print("Creating H264Parser \n")
    h264parser = Gst.ElementFactory.make("h264parse", "h264-parser")
    if not h264parser:
        sys.stderr.write(" Unable to create h264 parser \n")
    
    # Use nvdec_h264 for hardware accelerated decode on GPU
    print("Creating Decoder \n")
    decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
    if not decoder:
        sys.stderr.write(" Unable to create Nvv4l2 Decoder \n")
    """

    # Create nvstreammux instance to form batches from one or more sources.
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")
    
    pipeline.add(streammux)
    
    streamdemux = Gst.ElementFactory.make("nvstreamdemux", "Stream-demuxer")
    if not streamdemux:
        sys.stderr.write(" Unable to create NvStreamdeMux \n")
    
    for i, sp in enumerate(stream_path):
        # i = 0
        print("Creating source_bin ",i," \n ")
        source_bin=create_source_bin(i, sp)
        if not source_bin:
            sys.stderr.write("Unable to create source bin \n")
        pipeline.add(source_bin)
        padname="sink_%u" %i
        sinkpad= streammux.get_request_pad(padname) 
        if not sinkpad:
            sys.stderr.write("Unable to create sink pad bin \n")
        srcpad=source_bin.get_static_pad("src")
        if not srcpad:
            sys.stderr.write("Unable to create src pad bin \n")
        srcpad.link(sinkpad)

    # queue1=Gst.ElementFactory.make("queue", "queue1")

    # Use nvinfer to run inferencing on decoder's output,
    # behaviour of inferencing is set through config file
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")
    
    # Use convertor to convert from NV12 to RGBA as required by nvosd
    nvvidconv1 = Gst.ElementFactory.make("nvvideoconvert", "convertor1")
    nvvidconv2 = Gst.ElementFactory.make("nvvideoconvert", "convertor2")
    if not nvvidconv1:
        sys.stderr.write(" Unable to create nvvidconv \n")
    
    # Create OSD to draw on the converted RGBA buffer
    nvosd1 = Gst.ElementFactory.make("nvdsosd", "onscreendisplay1")
    nvosd2 = Gst.ElementFactory.make("nvdsosd", "onscreendisplay2")
    if not nvosd1:
        sys.stderr.write(" Unable to create nvosd \n")
    nvvidconv_postosd1 = Gst.ElementFactory.make("nvvideoconvert", "convertor_postosd1")
    nvvidconv_postosd2 = Gst.ElementFactory.make("nvvideoconvert", "convertor_postosd2")
    if not nvvidconv_postosd1:
        sys.stderr.write(" Unable to create nvvidconv_postosd \n")
    
    # Create a caps filter
    caps1 = Gst.ElementFactory.make("capsfilter", "filter1")
    caps2 = Gst.ElementFactory.make("capsfilter", "filter2")
    caps1.set_property("caps", Gst.Caps.from_string("video/x-raw(memory:NVMM), format=I420"))
    caps2.set_property("caps", Gst.Caps.from_string("video/x-raw(memory:NVMM), format=I420"))
    
    # Make the encoder
    if codec == "H264":
        encoder1 = Gst.ElementFactory.make("nvv4l2h264enc", "encoder1")
        encoder2 = Gst.ElementFactory.make("nvv4l2h264enc", "encoder2")
        print("Creating H264 Encoder")
    elif codec == "H265":
        encoder1 = Gst.ElementFactory.make("nvv4l2h265enc", "encoder1")
        encoder2 = Gst.ElementFactory.make("nvv4l2h265enc", "encoder2")
        print("Creating H265 Encoder")
    if not encoder1:
        sys.stderr.write(" Unable to create encoder")
    encoder1.set_property('bitrate', bitrate)
    encoder2.set_property('bitrate', bitrate)
    if is_aarch64():
        encoder1.set_property('preset-level', 1)
        encoder2.set_property('preset-level', 1)
        encoder1.set_property('insert-sps-pps', 1)
        encoder2.set_property('insert-sps-pps', 1)
        encoder1.set_property('bufapi-version', 1)
        encoder2.set_property('bufapi-version', 1)
    
    # Make the payload-encode video into RTP packets
    if codec == "H264":
        rtppay1 = Gst.ElementFactory.make("rtph264pay", "rtppay1")
        rtppay2= Gst.ElementFactory.make("rtph264pay", "rtppay2")
        print("Creating H264 rtppay")
    elif codec == "H265":
        rtppay1 = Gst.ElementFactory.make("rtph265pay", "rtppay1")
        rtppay2 = Gst.ElementFactory.make("rtph265pay", "rtppay2")
        print("Creating H265 rtppay")
    if not rtppay1:
        sys.stderr.write(" Unable to create rtppay")
    
    # Make the UDP sink
    updsink_port_num1 = 5400
    updsink_port_num2 = 5401
    sink1 = Gst.ElementFactory.make("udpsink", "udpsink1")
    sink2 = Gst.ElementFactory.make("udpsink", "udpsink2")
    if not sink1:
        sys.stderr.write(" Unable to create udpsink")
    
    sink1.set_property('host', '127.0.0.1')
    sink2.set_property('host', '127.0.0.1')
    sink1.set_property('port', updsink_port_num1)
    sink2.set_property('port', updsink_port_num2)
    sink1.set_property('async', False)
    sink2.set_property('async', False)
    sink1.set_property('sync', 1)
    sink2.set_property('sync', 1)
   
    is_live = 0
    if is_live:
        print("Atleast one of the sources is live")
        streammux.set_property('live-source', 1)
    print("Playing file %s " %stream_path)
    #source.set_property('location', stream_path)
    streammux.set_property('width', 1920)
    streammux.set_property('height', 1080)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)
    
    pgie.set_property('config-file-path', "dstest1_pgie_config.txt")
    
    print("Adding elements to Pipeline \n")
    #pipeline.add(source)
    #pipeline.add(h264parser)
    #pipeline.add(decoder)
    #pipeline.add(streammux)
    pipeline.add(pgie)
    pipeline.add(streamdemux)
    pipeline.add(nvvidconv1)
    pipeline.add(nvvidconv2)
    pipeline.add(nvosd1)
    pipeline.add(nvosd2)
    pipeline.add(nvvidconv_postosd1)
    pipeline.add(nvvidconv_postosd2)
    pipeline.add(caps1)
    pipeline.add(caps2)
    pipeline.add(encoder1)
    pipeline.add(encoder2)
    pipeline.add(rtppay1)
    pipeline.add(rtppay2)
    pipeline.add(sink1)
    pipeline.add(sink2)

    # Link the elements together:
    # file-source -> h264-parser -> nvh264-decoder ->
    # nvinfer -> nvvidconv -> nvosd -> nvvidconv_postosd -> 
    # caps -> encoder -> rtppay -> udpsink
    
    print("Linking elements in the Pipeline \n")
    """
    source.link(h264parser)
    h264parser.link(decoder)
    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        sys.stderr.write(" Unable to get the sink pad of streammux \n")
    
    srcpad = decoder.get_static_pad("src")
    if not srcpad:
        sys.stderr.write(" Unable to get source pad of decoder \n")
    
    srcpad.link(sinkpad)
    """
    
    nvvidconv = [nvvidconv1, nvvidconv2]
    nvosd = [nvosd1, nvosd2]
    nvvidconv_postosd = [nvvidconv_postosd1, nvvidconv_postosd2]
    caps = [caps1, caps2]
    encoder = [encoder1, encoder2]
    rtppay = [rtppay1, rtppay2]
    sink = [sink1, sink2]
    
    streammux.link(pgie)
    # pgie.link(nvvidconv)
    pgie.link(streamdemux)
    #######################
    for i in range(len(stream_path)):
        print("demux source", i, "\n")
        srcpad1 = streamdemux.get_request_pad("src_%u"%i)
        if not srcpad1:
            sys.stderr.write(" Unable to get the src pad of streamdemux \n")
        sinkpad1 = nvvidconv[i].get_static_pad("sink")
        if not sinkpad1:
            sys.stderr.write(" Unable to get sink pad of nvvidconv \n")
        srcpad1.link(sinkpad1)
        #######################
        
        nvvidconv[i].link(nvosd[i])
        nvosd[i].link(nvvidconv_postosd[i])
        nvvidconv_postosd[i].link(caps[i])
        caps[i].link(encoder[i])
        encoder[i].link(rtppay[i])
        rtppay[i].link(sink[i])
    
    # create an event loop and feed gstreamer bus mesages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)
    
    # Start streaming
    rtsp_port_num = 22
    
    server = GstRtspServer.RTSPServer.new()
    server.props.service = "%d" % rtsp_port_num
    server.attach(None)
    
    factory1 = GstRtspServer.RTSPMediaFactory.new()
    factory2 = GstRtspServer.RTSPMediaFactory.new()
    factory1.set_launch( "( udpsrc name=pay0 port=%d buffer-size=524288 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=(string)%s, payload=96 \" )" % (updsink_port_num1, codec))
    factory2.set_launch( "( udpsrc name=pay0 port=%d buffer-size=524288 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=(string)%s, payload=96 \" )" % (updsink_port_num2, codec))
    factory1.set_shared(True)
    factory2.set_shared(True)
    server.get_mount_points().add_factory("/ds-test1", factory1)
    server.get_mount_points().add_factory("/ds-test2", factory2)
    
    print("\n *** DeepStream: Launched RTSP Streaming at rtsp://localhost:%d/ds-test ***\n\n" % rtsp_port_num)
    
    # Lets add probe to get informed of the meta data generated, we add probe to
    # the sink pad of the osd element, since by that time, the buffer would have
    # had got all the metadata.
    osdsinkpad1 = nvosd[0].get_static_pad("sink")
    osdsinkpad2 = nvosd[1].get_static_pad("sink")
    if not osdsinkpad1:
        sys.stderr.write(" Unable to get sink pad of nvosd \n")
    
    osdsinkpad1.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)
    osdsinkpad2.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)
    
    # start play back and listen to events
    print("Starting pipeline \n")
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass
    # cleanup
    pipeline.set_state(Gst.State.NULL)

def parse_args():
    parser = argparse.ArgumentParser(description='RTSP Output Sample Application Help ')
    #parser.add_argument("-i", "--input",
    #              help="Path to input H264 elementry stream", required=True)
    parser.add_argument("-c", "--codec", default="H264",
                  help="RTSP Streaming Codec H264/H265 , default=H264", choices=['H264','H265'])
    parser.add_argument("-b", "--bitrate", default=4000000,
                  help="Set the encoding bitrate ", type=int)
    # Check input arguments
    #if len(sys.argv)==1:
    #    parser.print_help(sys.stderr)
    #    sys.exit(1)
    args = parser.parse_args()
    global codec
    global bitrate
    global stream_path
    codec = args.codec
    bitrate = args.bitrate
    stream_path = ['rtmp://58.200.131.2:1935/livetv/cctv1', 'rtmp://58.200.131.2:1935/livetv/cctv2']
    return 0

if __name__ == '__main__':
    parse_args()
    sys.exit(main(sys.argv))

