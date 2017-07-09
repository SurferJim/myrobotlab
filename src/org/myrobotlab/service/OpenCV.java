/**
 *                    
 * @author greg (at) myrobotlab.org
 *  
 * This file is part of MyRobotLab (http://myrobotlab.org).
 *
 * MyRobotLab is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version (subject to the "Classpath" exception
 * as provided in the LICENSE.txt file that accompanied this code).
 *
 * MyRobotLab is distributed in the hope that it will be useful or fun,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * All libraries in thirdParty bundle are subject to their own license
 * requirements - please refer to http://myrobotlab.org/libraries for 
 * details.
 * 
 * Enjoy !
 * 
 * */

package org.myrobotlab.service;

import static org.bytedeco.javacpp.opencv_core.cvPoint;
import static org.bytedeco.javacpp.opencv_imgproc.cvPutText;
/*
 static wild card imports for quickly finding static functions in eclipse
 */
//import static org.bytedeco.javacpp.opencv_calib3d.*;
//import static org.bytedeco.javacpp.opencv_contrib.*;
//import static org.bytedeco.javacpp.opencv_core.*;
//FIXME !!! no java.awt !!!! 
import java.awt.Dimension;
import java.awt.Rectangle;
//import static org.bytedeco.javacpp.opencv_gpu.*;
//import static org.bytedeco.javacpp.opencv_superres.*;
//import static org.bytedeco.javacpp.opencv_ts.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

import org.bytedeco.javacpp.opencv_core.CvPoint;
import org.bytedeco.javacpp.opencv_core.CvPoint2D32f;
import org.bytedeco.javacpp.opencv_core.CvScalar;
import org.bytedeco.javacpp.opencv_core.IplImage;
import org.bytedeco.javacpp.opencv_imgproc.CvFont;
import org.myrobotlab.framework.Instantiator;
import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.ServiceType;
import org.myrobotlab.image.ColoredPoint;
import org.myrobotlab.image.SerializableImage;
import org.myrobotlab.io.FileIO;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.Logging;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.opencv.BlockingQueueGrabber;
import org.myrobotlab.opencv.FilterWrapper;
import org.myrobotlab.opencv.OpenCVData;
import org.myrobotlab.opencv.OpenCVFilter;
import org.myrobotlab.opencv.OpenCVFilterDilate;
import org.myrobotlab.opencv.OpenCVFilterFaceDetect;
import org.myrobotlab.opencv.OpenCVFilterFaceDetect2;
import org.myrobotlab.opencv.OpenCVFilterInput;
import org.myrobotlab.opencv.OpenCVFilterPyramidDown;
import org.myrobotlab.opencv.OpenCVFilterRecorder;
import org.myrobotlab.opencv.Overlay;
import org.myrobotlab.reflection.Reflector;
// import org.myrobotlab.service.abstracts.AbstractVideoSource;
import org.myrobotlab.service.data.Point2Df;
import org.myrobotlab.service.interfaces.VideoProcessWorker;
import org.myrobotlab.service.interfaces.VideoProcessor;
import org.myrobotlab.service.interfaces.VideoSource;
import org.slf4j.Logger;

/*

import static org.bytedeco.javacpp.opencv_imgproc.*;
import static org.bytedeco.javacpp.opencv_calib3d.*;
import static org.bytedeco.javacpp.opencv_core.*;
import static org.bytedeco.javacpp.opencv_features2d.*;
import static org.bytedeco.javacpp.opencv_flann.*;
import static org.bytedeco.javacpp.opencv_highgui.*;
import static org.bytedeco.javacpp.opencv_imgcodecs.*;
import static org.bytedeco.javacpp.opencv_ml.*;
import static org.bytedeco.javacpp.opencv_objdetect.*;
import static org.bytedeco.javacpp.opencv_photo.*;
import static org.bytedeco.javacpp.opencv_shape.*;
import static org.bytedeco.javacpp.opencv_stitching.*;
import static org.bytedeco.javacpp.opencv_video.*;
import static org.bytedeco.javacpp.opencv_videostab.*;

*/

/**
 * 
 * OpenCV - This service provides webcam support and video image processing It
 * uses the JavaCV binding to the OpenCV library. OpenCV is a computer vision
 * library. You can create an OpenCV service and then add a pipeline of
 * OpenCVFilters to it to provide things like facial recognition, and
 * KLOpticalTracking
 * 
 * More Info about OpenCV : http://opencv.org/ JavaCV is maintained by Samuel
 * Audet : https://github.com/bytedeco/javacv
 * 
 * TODO - make VideoProcessing Interfaces which use SerializableImages
 * 
 */
public class OpenCV extends Service implements VideoSource, VideoProcessor {

  public static final String INPUT_KEY = "input";
  // FIXME - don't return BufferedImage return SerializableImage always !
  transient final static public String BACKGROUND = "background";

  transient public static final String FILTER_DETECTOR = "Detector";

  transient public static final String FILTER_DILATE = "Dilate";
  transient public static final String FILTER_ERODE = "Erode";
  transient public static final String FILTER_FACE_DETECT = "FaceDetect";
  transient public static final String FILTER_FIND_CONTOURS = "FindContours";
  transient public static final String FILTER_GOOD_FEATURES_TO_TRACK = "GoodFeaturesToTrack";
  // TODO - OpenCV constants / enums ? ... hmm not a big fan ...
  // FIXME - if these are constants they need to be part of the array of
  // possible filters ..
  transient public static final String FILTER_LK_OPTICAL_TRACK = "LKOpticalTrack";

  static final String FILTER_PKG_PREFIX = "org.myrobotlab.opencv.OpenCVFilter";
  transient public static final String FILTER_PYRAMID_DOWN = "PyramidDown";
  transient final static public String FOREGROUND = "foreground";
  // FIXME - make more simple
  transient public final static String INPUT_SOURCE_CAMERA = "camera";
  transient public final static String INPUT_SOURCE_IMAGE_DIRECTORY = "slideshow";
  transient public final static String INPUT_SOURCE_IMAGE_FILE = "imagefile";
  transient public final static String INPUT_SOURCE_MOVIE_FILE = "file";
  transient public final static String INPUT_SOURCE_NETWORK = "network";
  transient public final static String INPUT_SOURCE_PIPELINE = "pipeline";

  public final static Logger log = LoggerFactory.getLogger(OpenCV.class);
  transient final static public String PART = "part";

  static String POSSIBLE_FILTERS[] = { "AdaptiveThreshold", "AddAlpha", "AddMask", "Affine", "And", "AverageColor", "Canny", "ColorTrack", "Copy", "CreateHistogram", "Detector",
      "Dilate", "Erode", "FaceDetect", "FaceRecognizer", "Fauvist", "Ffmpeg", "FindContours", "Flip", "FloodFill", "FloorFinder", "GoodFeaturesToTrack", "Gray", "HoughLines2",
      "Hsv", "Input", "InRange", "KinectDepth", "KinectDepthMask", "KinectInterleave", "LKOpticalTrack", "Mask", "MatchTemplate", "MotionTemplate", "Mouse", "Not", "Output",
      FILTER_PYRAMID_DOWN, "PyramidUp", "RepetitiveAnd", "RepetitiveOr", "ResetImageRoi", "Resize", "SampleArray", "SampleImage", "SetImageROI", "SimpleBlobDetector", "Smooth",
      "Split", "State", "Surf", "Threshold", "Transpose" };

  private static final long serialVersionUID = 1L;

  transient public final static String SOURCE_KINECT_DEPTH = "SOURCE_KINECT_DEPTH";

  /**
   * This static method returns all the details of the class without it having
   * to be constructed. It has description, categories, dependencies, and peer
   * definitions.
   * 
   * @return ServiceType - returns all the data
   * 
   */
  static public ServiceType getMetaData() {

    ServiceType meta = new ServiceType(OpenCV.class.getCanonicalName());
    meta.addDescription("OpenCV (computer vision) service wrapping many of the functions and filters of OpenCV");
    meta.addCategory("video", "vision", "sensor");
    // meta.addPeer("streamer", "VideoStreamer", "video streaming service
    // for
    // webgui.");

    meta.sharePeer("streamer", "streamer", "VideoStreamer", "Shared Video Streamer");
    // meta.addDependency("org.bytedeco.javacpp","1.1");
    meta.addDependency("org.bytedeco.javacv", "1.3");
    meta.addDependency("pl.sarxos.webcam", "0.3.10");
    return meta;
  }

  static public String[] getPossibleFilters() {
    return POSSIBLE_FILTERS;
  }

  public static void main(String[] args) throws Exception {

    // TODO - Avoidance / Navigation Service
    // ground plane
    // http://stackoverflow.com/questions/6641055/obstacle-avoidance-with-stereo-vision
    // radio lab - map cells location cells yatta yatta
    // lkoptical disparity motion Time To Contact
    // https://www.google.com/search?aq=0&oq=opencv+obst&gcx=c&sourceid=chrome&ie=UTF-8&q=opencv+obstacle+avoidance
    //
    // WebGui webgui = (WebGui)Runtime.start("webgui", "WebGui");
    Runtime.start("gui", "SwingGui");

    org.apache.log4j.BasicConfigurator.configure();
    LoggingFactory.getInstance().setLevel(Level.INFO);

    // OpenCV opencvLeft = (OpenCV) Runtime.start("left", "OpenCV");
    // Runtime.start("right", "OpenCV");
    // opencvLeft.setFrameGrabberType("org.myrobotlab.opencv.SlideShowFrameGrabber");
    // opencvLeft.setInputSource(INPUT_SOURCE_IMAGE_DIRECTORY);
    // training images in this example must be same resolution as camera
    // video
    // stream.
    // OpenCVFilterTranspose tr = new OpenCVFilterTranspose("tr");
    // opencv.addFilter(tr);

    OpenCV opencv = (OpenCV) Runtime.start("opencv", "OpenCV");
    // Runtime.start("right", "OpenCV");
    // opencv.setFrameGrabberType("org.myrobotlab.opencv.SarxosFrameGrabber");
    // opencv.setFrameGrabberType("org.myrobotlab.opencv.MJpegFrameGrabber");

    // opencv.setInputSource(INPUT_SOURCE_IMAGE_DIRECTORY);
    // opencv.setInputSource(INPUT_SOURCE_CAMERA);
    // opencv.setInputSource(INPUT_SOURCE_NETWORK);
    // opencv.setInputFileName("http://192.168.4.112:8080/?action=stream");
    // opencv.setInputFileName("http://192.168.4.112:8081/?action=stream");

    opencv.addFilter("facerec", "FaceRecognizer");

    // OpenCVFilterPyramidDown pyramid = new OpenCVFilterPyramidDown("pyramid");
    // opencv.addFilter(pyramid);

    // OpenCVFilterDilate dilate = new OpenCVFilterDilate();
    // opencv.addFilter(dilate);

    // OpenCVFilterFaceDetect2 facedetect2 = new
    // OpenCVFilterFaceDetect2("facedetect2");
    // opencv.addFilter(facedetect2);
    // OpenCVFilterFaceRecognizer("facerec");

    // String trainingDir = "C:\\training";
    // facerec.setTrainingDir(trainingDir);
    // facerec.train();

    // opencvLeft.addFilter(facerec);

    // VideoStreamer vs = (VideoStreamer)Runtime.start("vs",
    // "VideoStreamer");
    // vs.attach(opencv);
    // opencv.capture();
    // opencvLeft.capture();
    // opencvRight.capture();

    /*
     * OpenCVFilterFFmpeg ffmpeg = new OpenCVFilterFFmpeg("ffmpeg");
     * opencv.addFilter(ffmpeg);
     */

    // opencv.capture();
    // Thread.sleep(5000);
    // opencv.stopCapture();
    // Thread.sleep(5000);
    //
    //
    // opencv.capture();
    // Thread.sleep(5000);
    // opencv.stopCapture();
    // Thread.sleep(5000);

    boolean leave = true;
    if (leave) {
      return;
    }

    // opencv.removeFilters();
    // ffmpeg.stopRecording();
    // opencv.setCameraIndex(0);

    // opencv.setInputSource("file");
    // opencv.setInputFileName("c:/test.avi");
    // opencv.capture();
    // OpenCVFilterSURF surf = new OpenCVFilterSURF("surf");
    // String filename = "c:/dev/workspace.kmw/myrobotlab/lloyd.png";
    // String filename = "c:/dev/workspace.kmw/myrobotlab/kw.jpg";
    // surf.settings.setHessianThreshold(400);
    // surf.loadObjectImageFilename(filename);
    // opencv.addFilter(surf);

    // OpenCVFilterTranspose tr = new OpenCVFilterTranspose("tr");
    // opencv.addFilter(tr);
    /*
     * OpenCVFilterLKOpticalTrack lktrack = new
     * OpenCVFilterLKOpticalTrack("lktrack"); opencv.addFilter(lktrack);
     */

    // OpenCVFilterAffine affine = new OpenCVFilterAffine("left");
    // affine.setAngle(45);
    // opencv.addFilter(affine);
    // opencv.test();

    // opencv.capture();
    // opencv.captureFromImageFile("C:\\mrl\\myrobotlab\\image0.png");

    // Runtime.createAndStart("gui", "SwingGui");
    // opencv.test();
    /*
     * Runtime.createAndStart("gui", "SwingGui"); RemoteAdapter remote =
     * (RemoteAdapter) Runtime.start("ocvremote", "RemoteAdapter");
     * remote.connect("tcp://localhost:6767");
     * 
     * opencv.capture(); boolean leaveNow = true; if (leaveNow) return;
     */

    // final CvMat image1 = cvLoadImageM("C:/blob.jpg" , 0);
    //
    // SimpleBlobDetector o = new SimpleBlobDetector();
    // KeyPoint point = new KeyPoint();
    // o.detect(image1, point, null);
    //
    // System.out.println(point.toString());

  }

  transient Queue<OpenCVFilter> addFilterQueue = new ConcurrentLinkedQueue<OpenCVFilter>();
  // track the state of opencv. capturing true/false?
  // FIXME - there should be a bool isCapturing() - part of VideoCapture
  // interface !
  // additionally this should not be public but package scope protected (ie no
  // declaration)
  public boolean capturing = false;

  OpenCVData data;
  // filters
  transient Map<String, OpenCVFilter> filters = new LinkedHashMap<String, OpenCVFilter>();

  transient CvFont font = new CvFont();
  private int frameIndex;
  StringBuffer frameTitle = new StringBuffer();

  // data
  // transient public BlockingQueue<Object> blockingData = new
  // LinkedBlockingQueue<Object>();
  OpenCVData lastOpenCVData;

  /**
   * the last source key - used to set the next filter's default source
   */
  String lastSourceKey;
  // mask for each named filter
  transient public HashMap<String, IplImage> masks = new HashMap<String, IplImage>();

  HashMap<String, Overlay> overlays = new HashMap<String, Overlay>();

  boolean publishDisplay = true;

  boolean publishOpenCVData = true;

  String recordingSource = INPUT_KEY;

  transient Queue<String> removeFilterQueue = new ConcurrentLinkedQueue<String>();
  /* moreStuff */

  // selections
  // FIXME - mark transient transient - but keep serializable !!!
  OpenCVFilterInput selectedInputFilter = null;
  OpenCVFilterRecorder selectedRecorderFilter = null;

  private boolean showFrameNumbers;

  // display
  boolean showTimestamp = true;

  // TODO: a peer, but in the future , we should use WebGui and it's http
  // container for this
  // if possible.
  // GROG : .. perhaps just a filter in the pipeline could stream it via http
  transient public VideoStreamer streamer;

  public boolean streamerEnabled = true;

  public boolean undockDisplay = false;

  // processing
  final transient VideoProcessWorker worker;

  public OpenCV(String n) {
    super(n);
    worker = new VideoProcessWorker(this);

    // create a default "input" filter
    selectedInputFilter = new OpenCVFilterInput(INPUT_KEY);
    addFilter(selectedInputFilter);
  }

  /**
   * adds a filter if the video processing thread is not running it adds it
   * directly to the linked hashmap filter. If the video processing thread is
   * running it must add it to a thread safe queue to be processed by the the
   * other thread. This should make the system thread-safe
   * 
   * @param filter
   * @return
   */
  public OpenCVFilter addFilter(OpenCVFilter filter) {
    filter.sourceKey = lastSourceKey;
    if (capturing) {
      addFilterQueue.add(filter);
    } else {
      filters.put(filter.name, filter);
    }
    lastSourceKey = filter.name;
    filter.setVideoProcessor(this);
    broadcastState();
    return filter;
  }

  /**
   * add filter by name
   * 
   * @param filterName
   * @return
   */
  public OpenCVFilter addFilter(String filterName) {
    return addFilter(filterName, filterName);
  }

  /**
   * add filter by name and type
   * 
   * @param name
   * @param filterType
   * @return
   */
  public OpenCVFilter addFilter(String name, String filterType) {
    String type = FILTER_PKG_PREFIX + filterType;
    OpenCVFilter filter = (OpenCVFilter) Instantiator.getNewInstance(type, name);
    log.info("adding {} filter {}", type, name);
    addFilterQueue.add(filter);
    return addFilter(filter);
  }

  public void capture() {
    save();

    if (streamerEnabled) {
      streamer = (VideoStreamer) startPeer("streamer");
      streamer.attach(this);
    }
    // stopCapture(); // restart?
    startVideoProcessing();
    // GroG:FIXED I THINK
    // there's a nasty race condition,
    // so we sleep here for 500 milliseconds to make sure
    // the video stream is up and running before we publish our state.
    // sleep(500); - FIXME - fix race condition
    capturing = true; // FIXME <-- change to event publishCapturing(true)
    // invoke("publishCapturing", true);
    broadcastState(); // let everyone know
  }

  public void captureFromImageFile(String filename) {
    stopCapture();
    setFrameGrabberType("org.myrobotlab.opencv.ImageFileFrameGrabber");
    setInputSource(INPUT_SOURCE_IMAGE_FILE);
    setInputFileName(filename);
    capture();
  }

  public void captureFromResourceFile(String filename) throws IOException {
    FileIO.extractResource(filename, filename);
    captureFromImageFile(filename);
  }

  public void clearSources() {
    OpenCVData.clearSources();
  }

  public IplImage get(String key) {
    if (lastOpenCVData != null && lastOpenCVData.containsKey(key)) {
      return lastOpenCVData.get(key);
    }
    return null;
  }

  public Queue<OpenCVFilter> getAddFilterQueue() {
    return addFilterQueue;
  }

  public int getCameraIndex() {
    if (selectedInputFilter == null) {
      error("please select an input filter");
      return -1;
    } else {
      return selectedInputFilter.getCameraIndex();
    }
  }

  public SerializableImage getDisplay() {
    OpenCVData d = getOpenCVData();
    SerializableImage ret = new SerializableImage(d.getBufferedImage(), d.getSelectedFilterName());
    return ret;
  }

  public String getDisplayFilterName() {
    if (selectedInputFilter == null) {
      error("please select an input filter");
      return null;
    } else {
      return selectedInputFilter.getDisplayFilterName();
    }
  }

  public OpenCVData getFaceDetect() {
    OpenCVFilterFaceDetect fd = new OpenCVFilterFaceDetect();
    addFilter(fd);
    OpenCVData d = getOpenCVData();
    removeFilter(fd.name);
    return d;
  }

  public OpenCVFilter getFilter(String name) {
    return filters.get(name);
  }

  public List<OpenCVFilter> getFiltersCopy() {
    return new ArrayList<OpenCVFilter>(filters.values());
  }

  // utility single shot functions - interesting idea
  // shouldn't this return just the points ?
  public OpenCVData getGoodFeatures() {
    addFilter(FILTER_GOOD_FEATURES_TO_TRACK, FILTER_GOOD_FEATURES_TO_TRACK);
    OpenCVData d = getOpenCVData();
    removeFilter(FILTER_GOOD_FEATURES_TO_TRACK);
    return d;
  }

  public String getGrabberType() {
    if (selectedInputFilter == null) {
      error("please select an input filter");
      return null;
    } else {
      return selectedInputFilter.getGrabberType();
    }
  }

  public String getInputFile() {
    if (selectedInputFilter == null) {
      error("please select an input filter");
      return null;
    } else {
      return selectedInputFilter.getInputFile();
    }
  }

  public String getInputSource() {
    if (selectedInputFilter == null) {
      error("please select an input filter");
      return null;
    } else {
      return selectedInputFilter.getInputSource();
    }
  }

  public OpenCVData getOpenCVData() {
    return lastOpenCVData;
  }

  public String getPipelineSelected() {
    if (selectedInputFilter == null) {
      error("please select an input filter");
      return null;
    } else {
      return selectedInputFilter.getPipelineSelected();
    }
  }

  public Queue<String> getRemoveFilters() {
    return removeFilterQueue;
  }

  @Override
  public VideoProcessWorker getWorker() {
    return worker;
  }

  /**
   * Callback from the SwingGui to the appropriate filter funnel through here
   */
  public void invokeFilterMethod(String filterName, String method, Object... params) {
    OpenCVFilter filter = getFilter(filterName);
    if (filter != null) {
      Reflector.invokeMethod(filter, method, params);
    } else {
      log.error("invokeFilterMethod " + filterName + " does not exist");
    }
  }

  @Override
  public boolean isCapturing() {
    return capturing;
  }

  public boolean isStreamerEnabled() {
    return streamerEnabled;
  }

  /**
   * main video processing loop sources is a globally accessible VideoSources -
   * but is not threadsafe data is thread safe - at least the references to the
   * data are threadsafe even if the data might not be (although it "probably"
   * is :)
   * 
   * more importantly the references of data are synced with itself - so that
   * all references are from the same processing loop
   */
  @Override
  public void processVideo() {

    capturing = true;

    while (capturing) {
      try {

        /**
         * add or remove filters depending on the requests of the queues
         */

        // process filter add requests
        if (addFilterQueue.size() > 0) {
          for (int i = 0; i < addFilterQueue.size(); ++i) {
            OpenCVFilter f = addFilterQueue.poll();
            if (f == null) {
              continue;
            }
            if (f.sourceKey == null) {
              f.sourceKey = lastSourceKey;
              data.put(f.name);
              lastSourceKey = f.name;
            }
            filters.put(f.name, f);
          }
          addFilterQueue.clear();
          broadcastState(); // filters have changed
        }

        // process filter remove requests
        if (removeFilterQueue.size() > 0) {
          for (int i = 0; i < removeFilterQueue.size(); ++i) {
            String name = removeFilterQueue.poll();
            if (name == null) {
              continue;
            }
            if (filters.containsKey(name)) {
              filters.remove(name);
            }
          }
          removeFilterQueue.clear();
          broadcastState(); // filters have changed
        }

        /**
         * This is the creation of a new OpenCVData. References for serializable
         * data will be created new and added to in the pipeline. Internally
         * sources are static (non serializable) and continue to be added or
         * overwritten.
         * 
         * OpenCVData.sources is an index into what has previously processed
         */
        data = new OpenCVData(getName(), frameIndex);

        // process each filter
        for (String filterName : filters.keySet()) {
          OpenCVFilter filter = filters.get(filterName);
          if (Logging.performanceTiming)
            Logging.logTime(String.format("pre set-filter %s", filter.name));

          // set the selected filter
          data.setFilter(filter);

          // get the source image this filter is chained to
          // should be safe and correct if operating in this
          // service
          // pipeline to another service needs to use data not
          // sources
          IplImage image = data.get(filter.sourceKey);

          // pre process handles image size & channel changes
          filter.preProcess(frameIndex, image, data);
          if (Logging.performanceTiming)
            Logging.logTime(String.format("preProcess-filter %s", filter.name));

          image = filter.process(image, data);

          if (Logging.performanceTiming)
            Logging.logTime(String.format("process-filter %s", filter.name));

          // process the image - push into source as new output
          // other pipelines will pull it off the from the sources
          data.put(filter.name, image);

          // no display || merge display || fork display
          // currently there is no "display" in sources
          // i've got a user selection to display a particular
          // filter

          // FIXME - display becomes an attribute of the filter itself
          // -
          // FIXME - filters by default "display" when they are
          // selected and "un-display" when they are not
          // FIXME - however - filters could be left in the display
          // state if desired
          // if (publishDisplay && displayFilterName != null &&
          // displayFilterName.equals(filter.name)) {
          if (publishDisplay) {

            // data.setDisplayFilterName(displayFilterName);

            // The fact that I'm in a filter loop
            // and there is a display to publish means
            // i've got to process a filter's display
            // TODO - would be to have a set of displays if it's
            // needed
            // if displayFilter == null but we are told to
            // display - then display INPUT

            filter.display(image, data);

            // if display frame
            if (showFrameNumbers || showTimestamp) {

              frameTitle.setLength(0);

              if (showFrameNumbers) {
                frameTitle.append("frame ");
                frameTitle.append(frameIndex);
                frameTitle.append(" ");
              }

              if (showTimestamp) {
                frameTitle.append(System.currentTimeMillis());
              }
              // log.info("Adding text: " +
              // frameTitle.toString());
              cvPutText(image, frameTitle.toString(), cvPoint(20, 20), font, CvScalar.BLACK);
              for (Overlay overlay : overlays.values()) {
                // log.info("Overlay text:" + overlay.text);
                cvPutText(image, overlay.text, overlay.pos, overlay.font, overlay.color);
              }
            }

          } // end of display processing

        } // for each filter

        if (Logging.performanceTiming)
          Logging.logTime("filters done");

        // copy key references from sources to data
        // the references will presist and so will the data
        // for as long as the OpenCVData structure exists
        // Sources will contain new references to new data
        // next iteration
        // data.putAll(sources.getData()); not needed :)

        // has to be 2 tests for publishDisplay
        // one inside the filter loop - to set the display to a new
        // filter
        // and this one to publish - if it is left "unset" then the
        // input becomes the
        // display filter
        if (publishDisplay) {
          SerializableImage display = new SerializableImage(data.getDisplayBufferedImage(), data.getDisplayFilterName(), frameIndex);
          invoke("publishDisplay", display);
        }

        // publish accumulated data
        if (publishOpenCVData) {
          invoke("publishOpenCVData", data);
        }

        // this has to be before record as
        // record uses the queue - this has the "issue" if
        // the consumer does not pickup-it will get stale
        // if (blockingData.size() == 0) {
        // blockingData.add(data);
        // }

      } catch (Exception e) {
        Logging.logError(e);
        log.error("stopping capture");
        stopVideoProcessing();
      }

      if (Logging.performanceTiming)
        Logging.logTime("finished pass");
    } // while capturing

  }

  // when containers are published the <T>ypes are unknown to the publishing
  // function
  public ArrayList<?> publish(ArrayList<?> polygons) {
    return polygons;
  }

  public ColoredPoint[] publish(ColoredPoint[] points) {
    return points;
  }

  public CvPoint publish(CvPoint point) {
    return point;
  }

  // CPP interface does not use array - but hides implementation
  public CvPoint2D32f publish(CvPoint2D32f features) {
    return features;
  }

  public double[] publish(double[] data) {
    return data;
  }

  public Point2Df publish(Point2Df point) {
    return point;
  }

  public Rectangle publish(Rectangle rectangle) {
    return rectangle;
  }

  public String publish(String value) {
    return value;
  }

  public final void publishDisplay(Boolean b) {
    publishDisplay = b;
  }

  /**
   * FIXME - input needs to be OpenCVData THIS IS NOT USED ! VideoProcessor NOW
   * DOES OpenCVData - this will return NULL REMOVE !!
   */
  public final SerializableImage publishDisplay(SerializableImage img) {
    // lastDisplay = new SerializableImage(img, source);
    // return lastDisplay;
    return img;
  }

  /**
   * publishing method for filters - used internally
   * 
   * @return FilterWrapper solves the problem of multiple types being resolved
   *         in the setFilterState(FilterWrapper data) method
   */
  public FilterWrapper publishFilterState(FilterWrapper filterWrapper) {
    return filterWrapper;
  }

  /**
   * publishing method for filters - uses string parameter for remote invocation
   * 
   * @return FilterWrapper solves the problem of multiple types being resolved
   *         in the onFilterState(FilterWrapper data) callback
   */
  public FilterWrapper publishFilterState(String name) {
    OpenCVFilter filter = getFilter(name);
    if (filter != null) {
      return new FilterWrapper(name, filter);
    } else {
      log.error(String.format("publishFilterState %s does not exist ", name));
    }

    return null;
  }

  public IplImage publishIplImageTemplate(IplImage img) {
    return img;
  }

  public void publishNoRecognizedFace() {

  }

  // the big switch <input>
  // FIXME - test overloaded !!!!
  public void publishOpenCVData(boolean b) {
    publishOpenCVData = b;
  }

  /**
   * the publishing point of all OpenCV goodies ! type conversion is held off
   * until asked for - then its cached SMART ! :)
   * 
   * @param data - the data to be published
   * @return - the published data
   */
  public final OpenCVData publishOpenCVData(OpenCVData data) {
   /* GroG asks, "Is this necessary  ?" shouldnt the thing that publishes
      do it through the capture method and update the capturing appropriately ? */

    if (data != null) {
      capturing = true;
    } else {
      capturing = false;
    }
    return data;
  }

  public String publishRecognizedFace(String value) {
    return value;
  }

  public SerializableImage publishTemplate(String source, BufferedImage img, int frameIndex) {
    SerializableImage si = new SerializableImage(img, source, frameIndex);
    return si;
  }

  // FIXME - need to record on a specific record filter !!!
  // maintain a default selected record filter selectedRecorder
  @Deprecated
  public void recordOutput(Boolean b) {
    // recordOutput = b;
    // FIXME - default name of recorder filter is recorder
    // FIXME - need another method to set selectedRecorder
  }

  public String recordSingleFrame() {
    // WOOHOO Changed threads & thread safe !
    // OpenCVData d = videoProcessor.getLastData();
    OpenCVData d = getOpenCVData();
    /*
     * if (d == null) { log.error(
     * "could not record frame last OpenCVData is null"); return null; }
     */
    return d.writeDisplay();
    // return d.writeInput();
  }

  /**
   * remove a specific filter if not capturing remove it from the video
   * processor thread's filters if the video thread is capturing - must remove
   * it by adding it to a removeFilterQueue to be processed later
   * 
   * @param name
   */
  public void removeFilter(String name) {
    log.info("removing filter {} ", name);
    if (!capturing) {
      filters.remove(name);
    } else {
      removeFilterQueue.add(name);
    }
  }

  /**
   * remove filter by name if not capturing remove it from the video processor
   * thread's filters if the video thread is capturing - must remove it by
   * adding it to a removeFilterQueue to be processed later
   */
  public void removeFilters() {
    log.info("removeFilters");
    if (!capturing) {
      filters.clear();
    } else {
      List<OpenCVFilter> fl = getFiltersCopy();
      for (OpenCVFilter filter : fl) {
        removeFilter(filter.name);
      }
    }
  }

  public void setCameraIndex(Integer index) {
    if (selectedInputFilter == null) {
      error("please select an input filter");
    } else {
      selectedInputFilter.setCameraIndex(index);
    }
  }

  // FIXME setSelectedFilter - one global concept of currently selected filter
  // - not just for display
  // FIXME - rename to setSelectedFilter
  @Deprecated
  public void setDisplayFilter(String name) {
    // this.displayFilterName = name;
    // FIXME set the dispaly on the default selected filter
    // FIXME - change this to setSelectedFilter
  }

  /**
   * @param otherFilter
   *          - data from remote source
   * 
   *          This updates the filter with all the non-transient data in a
   *          remote copy through a reflective field update. If your filter has
   *          JNI members or pointer references it will break, mark all of
   *          these.
   */
  public void setFilterState(FilterWrapper otherFilter) {

    OpenCVFilter filter = getFilter(otherFilter.name);
    if (filter != null) {
      Service.copyShallowFrom(filter, otherFilter.filter);
    } else {
      error("setFilterState - could not find %s ", otherFilter.name);
    }

  }

  public void setFrameGrabberType(String grabberType) {
    if (selectedInputFilter == null) {
      error("please select an input filter");
    } else {
      selectedInputFilter.setFrameGrabberType(grabberType);
    }
  }

  public void setInputFileName(String inputFile) {
    if (selectedInputFilter == null) {
      error("please select an input filter");
    } else {
      selectedInputFilter.setInputFileName(inputFile);
    }
  }

  public void setInputSource(String inputSource) {
    if (selectedInputFilter == null) {
      error("please select an input filter");
    } else {
      selectedInputFilter.setInputSource(inputSource);
    }
  }

  public void setMask(String name, IplImage mask) {
    masks.put(name, mask);
  }

  /**
   * minimum time between processing frames - time unit is in milliseconds
   * 
   * @param time
   */
  public void setMinDelay(int time) {
    if (selectedInputFilter == null) {
      error("please select an input filter");
    } else {
      selectedInputFilter.setMinDelay(time);
    }
  }

  public void setPipeline(String pipeline) {
    if (selectedInputFilter == null) {
      error("please select an input filter");
    } else {
      selectedInputFilter.setPipeline(pipeline);
    }
  }

  public void setRecordingSource(String source) {
    this.recordingSource = source;
  }

  public void setStreamerEnabled(boolean streamerEnabled) {
    this.streamerEnabled = streamerEnabled;
  }

  public void showFrameNumbers(boolean b) {
    this.showFrameNumbers(b);
  }

  public void showTimestamp(boolean b) {
    this.showTimestamp = b;
  }

  public void stopCapture() {
    stopVideoProcessing();
    // FIXME -
    capturing = false;// change this to an event .. publishCapturing()
    // invoke("publishCapturing", false);
    broadcastState();
  }

  // FIXME - implement
  public void stopRecording(String filename) {
    // cvReleaseVideoWriter(outputFileStreams.get(filename).pointerByReference());
  }

  @Override
  public void stopService() {
    // shut down video
    stopVideoProcessing();
    // shut down service
    super.stopService();
  }

  public boolean undockDisplay(boolean b) {
    undockDisplay = b;
    broadcastState();
    return b;
  }

}
