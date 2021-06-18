import sys
sys.path.append('../')
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
import pyds
import tracker
import time
from datetime import datetime
import cv2
import numpy as np


def create_sink_bin(index):

    print("Creating source bin")
    bin_name="source-bin-%02d" %index
    print(bin_name)
    nbin=Gst.Bin.new(bin_name)
    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")
    if is_aarch64():
        transform = Gst.ElementFactory.make("nvegltransform", "nvegl-transform")
    print("Creating EGLSink \n")
    sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
    Gst.Bin.add(nbin,transform)
    Gst.Bin.add(nbin,sink)
    transform.link(sink)

    transformPad = Gst.Element.get_static_pad (transform, "sink")
    print(transformPad)
    bin_pad=nbin.add_pad(Gst.GhostPad.new("sink",transformPad))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin


def main(args):
    # Check input arguments
    if len(args) != 2:
        sys.stderr.write("usage: %s <media file or uri>\n" % args[0])
        sys.exit(1)

    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    pipeline = Gst.Pipeline()

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")
    source = Gst.ElementFactory.make("filesrc", "file-source") 
    h264parser = Gst.ElementFactory.make("h264parse", "h264-parser") 
    decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer") 
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    # if is_aarch64():
    #     transform = Gst.ElementFactory.make("nvegltransform", "nvegl-transform")
    # print("Creating EGLSink \n")
    # sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
    print("Playing file %s " %args[1])
    source.set_property('location', args[1])
    streammux.set_property('width', 1280)
    streammux.set_property('height', 720)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)
    pipeline.add(source)
    pipeline.add(h264parser)
    pipeline.add(decoder)
    pipeline.add(streammux)
    pipeline.add(nvvidconv)
    

    print("Linking elements in the Pipeline \n")
    source.link(h264parser)
    h264parser.link(decoder)

    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        sys.stderr.write(" Unable to get the sink pad of streammux \n")
    srcpad = decoder.get_static_pad("src")
    if not srcpad:
        sys.stderr.write(" Unable to get source pad of decoder \n")
    srcpad.link(sinkpad)

    streammux.link(nvvidconv)

    sink_bin = create_sink_bin(1)
    pipeline.add(sink_bin)
    srcpad = nvvidconv.get_static_pad("src") 
    print(srcpad)
    sinkpad = sink_bin.get_static_pad("sink")
    print(sinkpad)
    srcpad.link(sinkpad)
    # if is_aarch64():
    #     nvvidconv.link(transform)
    #     transform.link(sink)
    # else:
    #     nvvidconv.link(sink)

    # create an event loop and feed gstreamer bus mesages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)


    # start play back and listen to events
    print("Starting pipeline \n")
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass
    # cleanup
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    sys.exit(main(sys.argv))

