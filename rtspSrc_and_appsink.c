
#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <gst/app/gstappsink.h>
#include <gst/allocators/gstdmabuf.h>
#include <gst/app/gstappsrc.h>

#include <glib.h>
#include <termios.h> 
#include <iostream>
#include <cstdlib>

#include <opencv2/opencv.hpp>
using namespace cv;


GstElement *pipeline;


int _getch() 
{ 
    int ch; 
    struct termios old; 
    struct termios current; 
    tcgetattr(0, &old); 

    current = old; 
    current.c_lflag &= ~ICANON; 
    current.c_lflag &= ~ECHO; 

    tcsetattr(0, TCSANOW, &current); 
    ch = getchar(); 
    tcsetattr(0, TCSANOW, &old); 

    return ch; 
}


GstFlowReturn new_sample0(GstAppSink *appsink, gpointer data) 
{

    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));

    if (sample != NULL)
    {
        GstCaps *caps = gst_sample_get_caps(sample);
        GstBuffer *buffer = gst_sample_get_buffer(sample);

        int width, height;
        GstStructure* structure = gst_caps_get_structure (caps, 0);
        gst_structure_get_int(structure, "height", &height);
        gst_structure_get_int(structure, "width", &width);
        int size = gst_buffer_get_size(buffer);

        printf("%d %d %d\n", width, height, size);

        GstMapInfo map;
        gst_buffer_map (buffer, &map, GST_MAP_READ);

        
        uint8_t *nv12DataBuffer = new uint8_t[size];
        memcpy(nv12DataBuffer, (uint8_t*)map.data, size);
      
        Mat img = Mat(Size(width, height*3/2), CV_8UC1, nv12DataBuffer);
        
        Mat img2;
        cvtColor(img, img2, COLOR_YUV2BGR_NV12 );

        imshow("window", img2);
        waitKey(1);

        delete[] nv12DataBuffer;

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref (sample);

    }

    return GST_FLOW_OK;
}



static void on_pad_added (GstElement *element, GstPad *pad, gpointer data)
{
    GstElement *source=gst_bin_get_by_name (GST_BIN(pipeline), "rtsp-source");
    GstElement *depay=gst_bin_get_by_name (GST_BIN(pipeline), "depay");

    gst_element_link(source, depay);

    gst_object_unref (GST_OBJECT (source));
    gst_object_unref (GST_OBJECT (depay));
}


int main ()
{
    GstElement *source, *depay, *parse, *decoder,  *filter1, *conv, *filter2, *sink;

    gst_init (NULL, NULL);


    pipeline = gst_pipeline_new ("player");
    source   = gst_element_factory_make ("rtspsrc", "rtsp-source");
    depay    = gst_element_factory_make ("rtph264depay", "depay");
    parse    = gst_element_factory_make ("h264parse", "parser");
    decoder  = gst_element_factory_make ("nvv4l2decoder", "decoder"); 
    conv = gst_element_factory_make("nvvidconv", "conv"); 
    sink     = gst_element_factory_make ("appsink", "sink");
    filter1 = gst_element_factory_make ("capsfilter", "video_filter1");
    filter2 = gst_element_factory_make ("capsfilter", "video_filter2");

    if (!pipeline || !source || !depay || !parse || !decoder || !filter1 ||!conv || !filter2 || !sink ) {
    printf("One element could not be created. Exiting.\n");
    return 1;
    }


    GstCaps    *caps1, *caps2;
    caps1 = gst_caps_from_string ("video/x-raw(memory:NVMM)"); 
    caps2 = gst_caps_from_string ("video/x-raw"); 
    g_object_set (G_OBJECT (filter1), "caps", caps1, NULL);
    g_object_set (G_OBJECT (filter2), "caps", caps2, NULL);
    gst_caps_unref (caps1);
    gst_caps_unref (caps2);


    g_object_set (G_OBJECT (source), "location", "rtsp ip address", NULL);

    g_object_set(G_OBJECT(sink), "emit-signals", true, NULL);
    g_object_set(G_OBJECT(sink), "async", false, "sync", false, "max-lateness", 0, NULL);
    g_object_set(G_OBJECT (source), "latency", 0, NULL);

    gst_bin_add_many (GST_BIN (pipeline), source, depay, parse, decoder, filter1, conv, filter2, sink, NULL);
    gst_element_link_many (depay, parse, decoder, filter1, conv, filter2, sink, NULL);


    g_signal_connect (source, "pad-added", G_CALLBACK (on_pad_added), NULL);
    if ( g_signal_connect(sink, "new-sample", G_CALLBACK(new_sample0), NULL) <= 0 )
    {
        std::cout << "Connects a GCallback function to a signal new-sample" << std::endl;
        return 1;
    }


    printf ("Now playing\n");
    
    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    printf ("Running...\n");

    while(1)
    {
        int key = _getch();

        if (key=='q') break;
    }

    printf ("Returned, stopping playback\n");
    gst_element_set_state (pipeline, GST_STATE_NULL);

    printf ("Deleting pipeline\n");
    gst_object_unref (GST_OBJECT (pipeline));
    gst_deinit();

    return 0;
}
