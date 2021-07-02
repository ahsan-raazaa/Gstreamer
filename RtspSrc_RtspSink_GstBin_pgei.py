import sys
sys.path.append('../')
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GObject, Gst, GstRtspServer
from gi.repository import GObject, Gst, GLib
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
import pyds
import tracker
import time
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
from common.FPS import GETFPS
from datetime import datetime
import cv2
import numpy as np

Gst.debug_set_active(True)
Gst.debug_set_default_threshold(3)
OSD_PROCESS_MODE= 0
OSD_DISPLAY_TEXT= 0
pipeline = None
streammux = None
streamdemux = None
sink_bin_4 = None
source_bin_4 = None
server_list = None
SrcBin_list = None
SinkBin_list = None
server_id_list = None

#///////////////////////////////////////////////////////////////////////////////////
fps_streams={}

g_source_bin_list = None

MAX_DISPLAY_LEN=64
PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_BICYCLE = 1
PGIE_CLASS_ID_PERSON = 2
PGIE_CLASS_ID_ROADSIGN = 3
MUXER_OUTPUT_WIDTH=1280
MUXER_OUTPUT_HEIGHT=720
MUXER_BATCH_TIMEOUT_USEC=4000000
TILED_OUTPUT_WIDTH=1280
TILED_OUTPUT_HEIGHT=720
GST_CAPS_FEATURES_NVMM="memory:NVMM"
OSD_PROCESS_MODE= 0
OSD_DISPLAY_TEXT= 0
pgie_classes_str= ["Vehicle", "TwoWheeler", "Person","RoadSign"]

classes = ["Vehicle", "TwoWheeler", "Person","RoadSign"]

#tracker is off when 0 and on when 1
tracker_status = 1

trackers_list = []
Roi_points_list = []
tracker_width  = 1080
tracker_height = 720
boxes_dic = {}
pointsROI10 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI11 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI12 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI13 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI14 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI15 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI16 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI2 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI3 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI4 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI5 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI6 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI7 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI8 = [168, 400, 1100, 400, 1100, 418, 168, 418]
pointsROI9 = [168, 400, 395, 403, 394, 418, 168, 423]
pointsROI =  [250, 300, 1250, 300, 1250, 318, 250, 318]


Roi_points_list.append(pointsROI)
Roi_points_list.append(pointsROI2)
Roi_points_list.append(pointsROI3)
Roi_points_list.append(pointsROI4)
Roi_points_list.append(pointsROI5)
Roi_points_list.append(pointsROI6)
Roi_points_list.append(pointsROI7)
Roi_points_list.append(pointsROI8)
Roi_points_list.append(pointsROI9)
Roi_points_list.append(pointsROI10)
Roi_points_list.append(pointsROI11)
Roi_points_list.append(pointsROI12)
Roi_points_list.append(pointsROI13)
Roi_points_list.append(pointsROI14)
Roi_points_list.append(pointsROI15)
Roi_points_list.append(pointsROI16)

num_classes = len(classes)

def tiler_src_pad_buffer_probe(pad,info,u_data):
    frame_number=0
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

        '''
        print("Frame Number is ", frame_meta.frame_num)
        print("Source id is ", frame_meta.source_id)
        print("Batch id is ", frame_meta.batch_id)
        print("Source Frame Width ", frame_meta.source_frame_width)
        print("Source Frame Height ", frame_meta.source_frame_height)
        print("Num object meta ", frame_meta.num_obj_meta)
        '''
        frame_number=frame_meta.frame_num
        l_obj=frame_meta.obj_meta_list
        bboxes   = []
        classids = []
        num_rects = frame_meta.num_obj_meta
        obj_counter = {
        PGIE_CLASS_ID_VEHICLE:0,
        PGIE_CLASS_ID_PERSON:0,
        PGIE_CLASS_ID_BICYCLE:0,
        PGIE_CLASS_ID_ROADSIGN:0
        }
        while l_obj is not None:
            try: 
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
            obj_counter[obj_meta.class_id] += 1
            
            bos = np.array([int(obj_meta.rect_params.left),
            int(obj_meta.rect_params.top),
            int(obj_meta.rect_params.left + obj_meta.rect_params.width),
            int(obj_meta.rect_params.top + obj_meta.rect_params.height)])
            bboxes.append(bos.astype("int"))

            classids.append(obj_meta.class_id)

            try: 
                l_obj=l_obj.next
            except StopIteration:
                break
                
        boxes_dic[frame_meta.source_id] = bboxes
        frameTime = int(datetime.now().timestamp())

        if tracker_status == 1:
                counter,counts,trackers=trackers_list[frame_meta.source_id].detectandkalmtrack(boxes_dic[frame_meta.source_id], classids, frameTime=frameTime)
                for index,values in counts.items():
                        counters=list(values.keys())
                        #print(f' This {frame_meta.source_id + 1} Tracker {classes[index]} -- {counters[0]} ----{str(values["inCount"])} ------ {counters[1]}---- {str(values["outCount"])}')
    
        """display_meta=pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        display_meta.num_labels = 1
        py_nvosd_text_params = display_meta.text_params[0]
        py_nvosd_text_params.display_text = "Frame Number={} Number of Objects={} Vehicle_count={} Person_count={}".format(frame_number, num_rects, vehicle_count, person)
        py_nvosd_text_params.x_offset = 10;
        py_nvosd_text_params.y_offset = 12;
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 10
        py_nvosd_text_params.font_params.font_color.red = 1.0
        py_nvosd_text_params.font_params.font_color.green = 1.0
        py_nvosd_text_params.font_params.font_color.blue = 1.0
        py_nvosd_text_params.font_params.font_color.alpha = 1.0
        py_nvosd_text_params.set_bg_clr = 1
        py_nvosd_text_params.text_bg_clr.red = 0.0
        py_nvosd_text_params.text_bg_clr.green = 0.0
        py_nvosd_text_params.text_bg_clr.blue = 0.0
        py_nvosd_text_params.text_bg_clr.alpha = 1.0
        #print("Frame Number=", frame_number, "Number of Objects=",num_rects,"Vehicle_count=",vehicle_count,"Person_count=",person)
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)"""
        #print("Frame Number=", frame_number, "Number of Objects=",num_rects,"Vehicle_count=",obj_counter[PGIE_CLASS_ID_VEHICLE],"Person_count=",obj_counter[PGIE_CLASS_ID_PERSON])
        
        framenumber = 0  
        display_meta=pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        x1,y1,x2,y2,x3,y3,x4,y4 = Roi_points_list[frame_meta.source_id]
        line_params = display_meta.line_params[framenumber]
        framenumber+=1
        line_params.x1 = x1
        line_params.y1 = y1
        line_params.x2 = x2
        line_params.y2 = y2
        line_params.line_width = 4
        line_params.line_color.red = 1.0
        line_params.line_color.green = 1.0
        line_params.line_color.blue = 0.0
        line_params.line_color.alpha = 0.7
        display_meta.num_lines = display_meta.num_lines + 1

        line_params = display_meta.line_params[framenumber]
        framenumber+=1
        line_params.x1 = x2
        line_params.y1 = y2
        line_params.x2 = x3
        line_params.y2 = y3                
        line_params.line_width = 4
        line_params.line_color.red = 1.0
        line_params.line_color.green = 1.0
        line_params.line_color.blue = 0.0
        line_params.line_color.alpha = 0.7
        display_meta.num_lines = display_meta.num_lines + 1

        line_params = display_meta.line_params[framenumber]
        framenumber+=1
        line_params.x1 = x3
        line_params.y1 = y3
        line_params.x2 = x4
        line_params.y2 = y4
        line_params.line_width = 4
        line_params.line_color.red = 1.0
        line_params.line_color.green = 1.0
        line_params.line_color.blue = 0.0
        line_params.line_color.alpha = 0.7
        display_meta.num_lines = display_meta.num_lines + 1

        line_params = display_meta.line_params[framenumber]
        framenumber+=1
        line_params.x1 = x4
        line_params.y1 = y4
        line_params.x2 = x1
        line_params.y2 = y1
        line_params.line_width = 4
        line_params.line_color.red = 1.0
        line_params.line_color.green = 1.0
        line_params.line_color.blue = 0.0
        line_params.line_color.alpha = 0.7
        display_meta.num_lines = display_meta.num_lines + 1
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

        # Get frame rate through this probe
        fps_streams["stream{0}".format(frame_meta.pad_index)].get_fps()
        try:
            l_frame=l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK





def create_source_bin(index,uri):

    print("Creating source bin")
    # Create a source GstBin to abstract this bin's content from the rest of the
    # pipeline
    bin_name="source-bin-%02d" %index
    rtsp_name = "rtspsrc-bin-%02d" %index
    print(bin_name)
    nbin=Gst.Bin.new(bin_name)
    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")

    # Source element for reading from the uri.
    # We will use decodebin and let it figure out the container format of the
    # stream and the codec and plug the appropriate demux and decode plugins.
    rtsp_bin= Gst.ElementFactory.make("rtspsrc", rtsp_name)
    if not rtsp_bin:
        sys.stderr.write(" Unable to create uri decode bin \n")
    # We set the input uri to the source element
    rtsp_bin.set_property("location",uri)
    rtsp_bin.set_property("drop-on-latency","FALSE")

    queue1 =  Gst.ElementFactory.make("queue", "queue1")
    queue2 =  Gst.ElementFactory.make("queue", "queue2")
    queue3 =  Gst.ElementFactory.make("queue", "queue3")

    Gst.Bin.add(nbin, queue1)
    Gst.Bin.add(nbin, queue2)
    Gst.Bin.add(nbin, queue3)
  
    jitterBuffer_name = "buffer" + str(index)
    jitterBuffer = Gst.ElementFactory.make("rtpjitterbuffer", jitterBuffer_name)
    #jitterBuffer.set_property("drop-on-latency", True)
    #jitterBuffer.set_property("latency", 0)

    Gst.Bin.add(nbin, jitterBuffer)
    depay =  Gst.ElementFactory.make("rtph264depay", "rtph264depay0")
    Gst.Bin.add(nbin,depay)
    h264parser =  Gst.ElementFactory.make("h264parse", "h264parse0")
    Gst.Bin.add(nbin,h264parser)
    decoder =  Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2decoder0")
    Gst.Bin.add(nbin,decoder)

    jitterBuffer.link(queue1)
    queue1.link(depay) 
    depay.link(queue2)
    queue2.link(h264parser)
    h264parser.link(queue3)
    queue3.link(decoder)   
    # depaySrcPad = Gst.Element.get_static_pad (depay, "src")
    # h264parserSinkPad = Gst.Element.get_static_pad (h264parser, "sink")
    # Gst.Pad.link(depaySrcPad, h264parserSinkPad)

    # h264parserSrcPad = Gst.Element.get_static_pad (h264parser, "src")
    # decoderSinkPad = Gst.Element.get_static_pad (decoder, "sink")
    # check = Gst.Pad.link(h264parserSrcPad, decoderSinkPad)

    # print("padLinked : "+str(check))

    decoderSrcPad = Gst.Element.get_static_pad (decoder, "src")

    # Connect to the "pad-added" signal of the decodebin which generates a
    # callback once a new pad for raw data has beed created by the decodebin
    rtsp_bin.connect("pad-added", on_rtspsrc_pad_added, jitterBuffer)
    #rtsp_bin.connect("child-added",decodebin_child_added,depay)

    # We need to create a ghost pad for the source bin which will act as a proxy
    # for the video decoder src pad. The ghost pad will not have a target right
    # now. Once the decode bin creates the video decoder and generates the
    # cb_newpad callback, we will set the ghost pad target to the video decoder
    # src pad.
    Gst.Bin.add(nbin,rtsp_bin)

    bin_pad=nbin.add_pad(Gst.GhostPad.new("src",decoderSrcPad))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin

def create_sink_bin(index):

    global server_list
    global server_id_list
    server_list = []
    server_id_list = []
    print("Creating sink bin")
    bin_name="sink-bin-%02d" %index
    print(bin_name)
    nbin=Gst.Bin.new(bin_name)

    queue1 =  Gst.ElementFactory.make("queue", "queue1")
    queue2 =  Gst.ElementFactory.make("queue", "queue2")
    queue3 =  Gst.ElementFactory.make("queue", "queue3")
    queue4 =  Gst.ElementFactory.make("queue", "queue4")
    queue5 =  Gst.ElementFactory.make("queue", "queue5")

    Gst.Bin.add(nbin, queue1)
    Gst.Bin.add(nbin, queue2)
    Gst.Bin.add(nbin, queue3)
    Gst.Bin.add(nbin, queue4)
    Gst.Bin.add(nbin, queue5)

    transform_name = "nvegl-transform" + str(index)
    sink_name = "nvvideo-renderer" + str(index)
    filesink_name = "file_sink" + str(index)
    nvvidconv_name = "convertor" + str(index)
    rtppay_name = "rtppay" + str(index)
    nvosd_name = "onscreendisplay" + str(index)
    post_nvvidconv_name =  "post_convertor" + str(index)

    if not nbin:
        sys.stderr.write(" Unable to create source bin \n")

    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", nvvidconv_name)

    
    nvosd =  Gst.ElementFactory.make("nvdsosd", nvosd_name)
    nvosd.set_property('process-mode',OSD_PROCESS_MODE)
    nvosd.set_property('display-text',OSD_DISPLAY_TEXT)

    post_nvvidconv = Gst.ElementFactory.make("nvvideoconvert", post_nvvidconv_name)


    encoder_name = "h264-encoder" + str(index)
    encoder = Gst.ElementFactory.make("nvv4l2h264enc", encoder_name)
    if not encoder:
        sys.stderr.write(" Unable to create encoder")
    encoder.set_property('bitrate', 2000000)
    if is_aarch64():
        encoder.set_property('preset-level', 1)
        encoder.set_property('insert-sps-pps', 1)
        encoder.set_property('bufapi-version', 1)

    rtppay = Gst.ElementFactory.make("rtph264pay", rtppay_name)
        
    name = "udpsink" + str(index)
    updsink_port_num = 5400 + index
    sink = Gst.ElementFactory.make("udpsink", name)
    if not sink:
        sys.stderr.write(" Unable to create udpsink")         
    sink.set_property('host', '127.0.0.1')
    sink.set_property('port', updsink_port_num)
    #sink.set_property('async', False)
    sink.set_property('sync', 0)
    #sink.set_property('qos', 0)
    
    # Start streaming
    rtsp_port_num = 8554 + index
    
    server = GstRtspServer.RTSPServer.new()
    server.props.service = "%d" % rtsp_port_num
    # server.attach(None)
    print(server)
    
    id = server.attach(None)
    server_id_list.append(id)
    print("SERVER : "+str(id))
    factory = GstRtspServer.RTSPMediaFactory.new()
    factory.set_launch( "( udpsrc name=pay0 port=%d buffer-size=524288 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=(string)%s, payload=96 \" )" % (updsink_port_num, "h264"))
    factory.set_shared(True)
    #factory.set_eos_shutdown(True)
    server.get_mount_points().add_factory("/ds-test", factory)
    
    print("\n *** DeepStream: Launched RTSP Streaming at rtsp://localhost:%d/ds-test ***\n\n" % rtsp_port_num)
    server_list.append(factory)
    Gst.Bin.add(nbin, nvvidconv)
    Gst.Bin.add(nbin, nvosd)
    Gst.Bin.add(nbin, post_nvvidconv)
    Gst.Bin.add(nbin, encoder)
    Gst.Bin.add(nbin, rtppay)
    Gst.Bin.add(nbin, sink)
    
    nvvidconv.link(queue1)
    queue1.link(nvosd)
    nvosd.link(queue2)
    queue2.link(post_nvvidconv)
    post_nvvidconv.link(queue3)
    queue3.link(encoder)
    encoder.link(queue4)
    queue4.link(rtppay)
    rtppay.link(queue5)
    queue5.link(sink)

    nvvidconvPad = Gst.Element.get_static_pad (nvvidconv, "sink")
    print(nvvidconvPad)
    bin_pad=nbin.add_pad(Gst.GhostPad.new("sink",nvvidconvPad))
    if not bin_pad:
        sys.stderr.write(" Failed to add ghost pad in source bin \n")
        return None
    return nbin

def pausePipeline(index):
    global SrcBin_list
    #/////////////////////////////////////////////////////////////////
    state_return = SrcBin_list[index].set_state(Gst.State.READY)
    if state_return == Gst.StateChangeReturn.SUCCESS:
            print("EOS SRC BIN STATE CHANGE SUCCESS\n")
    GObject.timeout_add_seconds(5, startPipeline, index)
    #/////////////////////////////////////////////////////////////////
    

def startPipeline(index):
    print("Start Pipeline")
    global SrcBin_list
    global SinkBin_list
    state_return = SrcBin_list[index].set_state(Gst.State.PLAYING)

    if state_return == Gst.StateChangeReturn.SUCCESS:
        print("SOURCE STATE CHANGE SUCCESS\n")
    elif state_return == Gst.StateChangeReturn.FAILURE:
        print("STATE CHANGE FAILURE\n")
    elif state_return == Gst.StateChangeReturn.NO_PREROLL:
        print("STATE CHANGE NO PREROLL\n")

    return False
    # state_return = SinkBin_list[index].set_state(Gst.State.PLAYING)

    # if state_return == Gst.StateChangeReturn.SUCCESS:
    #     print("SINK STATE CHANGE SUCCESS\n")
    # elif state_return == Gst.StateChangeReturn.FAILURE:
    #     print("STATE CHANGE FAILURE\n")
    # elif state_return == Gst.StateChangeReturn.NO_PREROLL:
    #     print("STATE CHANGE NO PREROLL\n")
 
def on_rtspsrc_pad_added(rtspsrc, pad, data):  
        print ("Dynamic pad created, linking source/depay\n")
        source_bin=data
        sinkpad = Gst.Element.get_static_pad (data, "sink")
        check = Gst.Pad.link(pad, sinkpad)

def stop_release_source(source_id):
    global sink_bin_4
    global streammux
    global pipeline
    global source_bin_4
    global server
    global SrcBin_list
    global SinkBin_list
    global server_list
    global server_id_list
    #Attempt to change status of source to be released 
    state_return = SrcBin_list[source_id].set_state(Gst.State.NULL)
    state_return = SinkBin_list[source_id].set_state(Gst.State.NULL)
    #GLib.Source.remove(server_id_list[0])
    #GObject.Object.unref(server_list[0])
    #server_list[0].unref()
    if state_return == Gst.StateChangeReturn.SUCCESS:
        print("STATE CHANGE SUCCESS\n")
        pad_name = "sink_%u" % source_id
        print(pad_name)
        #Retrieve sink pad to be released
        sinkpad = streammux.get_static_pad(pad_name)
        #Send flush stop event to the sink pad, then release from the streammux
        sinkpad.send_event(Gst.Event.new_flush_stop(False))
        streammux.release_request_pad(sinkpad)
        print("STATE CHANGE SUCCESS\n")
        #Remove the source bin from the pipeline
        pipeline.remove(SrcBin_list[0])
        pipeline.remove(SinkBin_list[0])
    
    elif state_return == Gst.StateChangeReturn.FAILURE:
        print("STATE CHANGE FAILURE\n")
    add_sources(0)
    # elif state_return == Gst.StateChangeReturn.ASYNC:
    #     state_return = g_source_bin_list[source_id].get_state(Gst.CLOCK_TIME_NONE)
    #     pad_name = "sink_%u" % source_id
    #     print(pad_name)
    #     sinkpad = streammux.get_static_pad(pad_name)
    #     sinkpad.send_event(Gst.Event.new_flush_stop(False))
    #     streammux.release_request_pad(sinkpad)
    #     print("STATE CHANGE ASYNC\n")
    #     pipeline.remove(g_source_bin_list[source_id])
    #     source_id -= 1
    #     g_num_sources -= 1

def add_sources(index):
    # global sink_bin_4
    # global source_bin_4
    print("add_sources")
    global pipeline
    global SrcBin_list
    
    source_id = index
    i = source_id
    uri = r"rtsp://admin:pakistan1947@192.168.2.54:554/Facit/media.smp"
    source_bin = create_source_bin(source_id, uri)
    if (not source_bin):
        sys.stderr.write("Failed to create source bin. Exiting.")
        exit(1)
    #source_bin_4 = source_bin
    pipeline.add(source_bin)
    padname = "sink_%u" %i
    sinkpad = streammux.get_request_pad(padname) 
    if not sinkpad:
        sys.stderr.write("Unable to create sink pad bin \n")
    srcpad = source_bin.get_static_pad("src")
    if not srcpad:
        sys.stderr.write("Unable to create src pad bin \n")
    srcpad.link(sinkpad)

    print("demux source", i, "\n")
    srcpad = streamdemux.get_static_pad("src_%u"%4)
    if not srcpad:
        sys.stderr.write(" Unable to get the src pad of streamdemux \n")
        exit(1)
    sink_bin = create_sink_bin(i)
    #sink_bin_4 = sink_bin
    print(sink_bin)
    pipeline.add(sink_bin)
    sinkpad = sink_bin.get_static_pad("sink")
    if not sinkpad:
        sys.stderr.write(" Unable to get sink pad of nvvidconv \n")
    srcpad.link(sinkpad)

    state_return = source_bin.set_state(Gst.State.PLAYING)
    state_return = sink_bin.set_state(Gst.State.PLAYING)
    if state_return == Gst.StateChangeReturn.SUCCESS:
        print("STATE CHANGE SUCCESS\n")
    elif state_return == Gst.StateChangeReturn.FAILURE:
        print("STATE CHANGE FAILURE\n")
    elif state_return == Gst.StateChangeReturn.NO_PREROLL:
        print("STATE CHANGE NO PREROLL\n")

    #GObject.timeout_add_seconds(60, stop_release_source, 4)
    return False

def bus_call(bus, message, loop):
    global g_eos_list
    global pipeline
    global SrcBin_list

    t = message.type
    if t == Gst.MessageType.EOS:
        sys.stdout.write("End-of-stream\n")
        loop.quit()
    elif t==Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        sys.stderr.write("Warning: %s: %s\n" % (err, debug))
        print("WARNING: "+debug)
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s\n" % (err, debug))
        if "No supported authentication protocol was found" in debug:
            print("IN")
            if "rtspsrc-bin-01" in str(err):
                pausePipeline(1)
            if "rtspsrc-bin-01" in debug:
                pausePipeline(1)
            if "rtspsrc-bin-00" in str(err):
                pausePipeline(0)
            if "rtspsrc-bin-00" in debug:
                pausePipeline(0)
                
        if "Not Found (404)" in debug:
            print("IN")
            if "rtspsrc-bin-01" in str(err):
                pausePipeline(1)
            if "rtspsrc-bin-01" in debug:
                pausePipeline(1)
            if "rtspsrc-bin-00" in str(err):
                pausePipeline(0)
            if "rtspsrc-bin-00" in debug:
                pausePipeline(0)


        #loop.quit()
    elif t == Gst.MessageType.ELEMENT:
        struct = message.get_structure()
        #Check for stream-eos message
        if struct is not None and struct.has_name("stream-eos"):
            parsed, stream_id = struct.get_uint("stream-id")
            if parsed:
                #Set eos status of stream to True, to be deleted in delete-sources
                print("Got EOS from stream %d" % stream_id)
                pausePipeline(stream_id)
                #GObject.timeout_add_seconds(60, stop_release_source, 0)
                #stop_release_source(stream_id)          
    return True


def main(args):
    global pipeline
    global streammux
    global streamdemux
    global SrcBin_list
    global SinkBin_list
    # Check input arguments
    if len(args) < 2:
        sys.stderr.write("usage: %s <media file or uri>\n" % args[0])
        sys.exit(1)

    for i in range(0,len(args)-1):
        fps_streams["stream{0}".format(i)]=GETFPS(i)
    number_sources=len(args)-1
    
    #////////////////////////////////////////////////
    if(tracker_status == 1):    
        for i in range(number_sources):
            print(f" sources {i}")
            x1,y1,x2,y2,x3,y3,x4,y4 = Roi_points_list[i]
            merge = tracker.DetectandTrack(x1,y1,x2,y2,x3,y3,x4,y4, tracker_width, tracker_height ,classes, direction=0)
            trackers_list.append(merge)
    #////////////////////////////////////////////////

    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    pipeline = Gst.Pipeline()

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")

    print("Creating Pgie \n ")
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")

    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    streamdemux = Gst.ElementFactory.make("nvstreamdemux", "Stream-demuxer")
    
    pipeline.add(streammux)
    pipeline.add(streamdemux)
    SrcBin_list = []
    for i in range(number_sources):
        print("Creating source_bin ",i," \n ")
        uri_name=args[i+1]

        source_bin=create_source_bin(i, uri_name)
        SrcBin_list.append(source_bin)
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

    streammux.set_property('width', 800)
    streammux.set_property('height', 600)
    streammux.set_property('batch-size', number_sources)
    streammux.set_property('batched-push-timeout', 200000)
    streammux.set_property('live-source', 1)

    pgie.set_property('config-file-path', "config_infer_primary.txt")
    pgie_batch_size=pgie.get_property("batch-size")
    if(pgie_batch_size != number_sources):
        print("WARNING: Overriding infer-config batch-size",pgie_batch_size," with number of sources ", number_sources," \n")
        pgie.set_property("batch-size",number_sources)

    print("Adding elements to Pipeline \n")
    pipeline.add(pgie)

    queue1 = Gst.ElementFactory.make("queue","queue1")
    queue2 = Gst.ElementFactory.make("queue","queue2")

    pipeline.add(queue1)
    pipeline.add(queue2)

    streammux.link(queue1)
    queue1.link(pgie)
    pgie.link(queue2)
    queue2.link(streamdemux)

    SinkBin_list = []
    for i in range(number_sources):
        print("demux source", i, "\n")
        
        srcpad = streamdemux.get_request_pad("src_%u"%i)
        if not srcpad:
            sys.stderr.write(" Unable to get the src pad of streamdemux \n")
        sink_bin = create_sink_bin(i)
        SinkBin_list.append(sink_bin)
        pipeline.add(sink_bin)
        sinkpad = sink_bin.get_static_pad("sink")
        if not sinkpad:
            sys.stderr.write(" Unable to get sink pad of nvvidconv \n")
        srcpad.link(sinkpad)

    srcpad = streamdemux.get_request_pad("src_%u"%4)
    if not srcpad:
        sys.stderr.write(" Unable to get the src pad of streamdemux \n") 

    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)



    tiler_src_pad=pgie.get_static_pad("src")
    if not tiler_src_pad:
        sys.stderr.write(" Unable to get src pad \n")
    else:
        tiler_src_pad.add_probe(Gst.PadProbeType.BUFFER, tiler_src_pad_buffer_probe, 0)

    # start play back and listen to events
    print("Starting pipeline \n")
    pipeline.set_state(Gst.State.PLAYING)

    #GObject.timeout_add_seconds(60, stop_release_source, 0)

    try:
        loop.run()
    except:
        pass
    # cleanup
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    sys.exit(main(sys.argv))