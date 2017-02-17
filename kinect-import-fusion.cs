using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.Colors;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;
using SynchronizationContext = System.Threading.SynchronizationContext;

namespace KinectSamples
{
  public static class ColorUtils
  {
    public static Point3dCollection
    Point3dFromColoredPointCollection(
      IList<ColoredPoint3d> vecs
    )
    {
      var pts = new Point3dCollection();
      foreach (var vec in vecs)
      {
        pts.Add(new Point3d(vec.X, vec.Y, vec.Z));
      }
      return pts;
    }

    public static Point3dCollection
    Point3dFromVertCollection(
      ReadOnlyCollection<Vector3> vecs
    )
    {
      var pts = new Point3dCollection();
      foreach (var vec in vecs)
      {
        pts.Add(new Point3d(vec.X, vec.Z, -vec.Y));
      }
      return pts;
    }

    public static List<ColoredPoint3d>
    ColoredPoint3FromVertCollection(
      ReadOnlyCollection<Vector3> vecs, ReadOnlyCollection<int> cols
    )
    {
      Debug.Assert(vecs.Count == cols.Count);

      var pts = new List<ColoredPoint3d>();
      for (int i=0; i < vecs.Count; ++i)
      {
        var vec = vecs[i];
        var col = cols[i];
        pts.Add(
          new ColoredPoint3d(
            vec.X, vec.Z, -vec.Y,
            (col >> 16) & 255,
            (col >> 8) & 255,
            col & 255
          )
        );
      }
      return pts;
    }
  }

  public class KinectFusionColorJig : KinectPointCloudJig
  {
    // Constants

    private const ReconstructionProcessor ProcessorType =
      ReconstructionProcessor.Amp;
    private const int DeviceToUse = -1;
    private const bool AutoResetReconstructionWhenLost = false;
    private const int MaxTrackingErrors = 100;
    private const int ResetOnTimeStampSkippedMillisecondsGPU = 2000;
    private const int ResetOnTimeStampSkippedMillisecondsCPU = 6000;
    private const int RawDepthWidth = 512;
    private const int RawDepthHeight = 424;
    private const int RawColorWidth = 1920;
    private const int RawColorHeight = 1080;
    private const int RawDepthHeightWithSpecialRatio = 384;
    private const int FpsInterval = 5;
    private const int StatusBarInterval = 1;
    private const int RenderIntervalMilliseconds = 100;
    private const int ColorIntegrationInterval = 1;
    private const int DeltaFrameCalculationInterval = 2;
    private const int LineThickness = 2;
    private const float OriginCoordinateCrossAxisSize = 0.1f;
    private const int CameraPoseFinderProcessFrameCalcInterval = 5;
    private const int MinGoodTrackingFramesForCameraPoseFinder = 45;
    private const int
      MinGoodTrackingFramesForCameraPoseFinderAfterFailure = 200;
    private const float MaxAlignToReconEnergyForSuccess = 0.27f;
    private const float MinAlignToReconEnergyForSuccess = 0.005f;
    private const float MaxAlignPointCloudsEnergyForSuccess = 0.006f;
    private const float MinAlignPointCloudsEnergyForSuccess = 0.0f;
    private const int MaxCameraPoseFinderPoseTests = 5;
    private const float CameraPoseFinderDistThresholdReject = 1.0f;
    private const float CameraPoseFinderDistThresholdAccept = 0.1f;
    // 0=just copy, 1=3x3, 2=5x5, 3=7x7, here we create a 3x3 kernel
    private const int SmoothingKernelWidth = 1;
    // 4cm, could use up to around 0.1f;
    private const float SmoothingDistanceThreshold = 0.04f;
    // 0.15 - 0.3m per frame typical
    private const float MaxTranslationDeltaAlignPointClouds = 0.3f;
    // 10-20 degrees per frame typical
    private const float MaxRotationDeltaAlignPointClouds = 20.0f;
    private const int DownsampleFactor = 2;

    // Member variables

    private bool _autoResetReconstructionOnTimeSkip = false;
    private bool _captureColor = true;
    private bool _pauseIntegration;
    private bool _mirrorDepth = false;
    private int _depWidth = 0;
    private int _depHeight = 0;
    private int _depPixelCount = 0;
    private int _colWidth = 0;
    private int _colHeight = 0;
    private int _colPixelCount = 0;
    private int _downsampledWidth;
    private int _downsampledHeight;
    private int _trackingErrorCount = 0;
    private bool _trackingFailed;
    private bool _trackingHasFailedPreviously;
    private bool _cameraPoseFinderAvailable;
    private int _successfulFrameCount;
    private int _processedFrameCount = 0;
    private TimeSpan _lastFrameTimestamp;
    private DispatcherTimer _fpsTimer;
    private DateTime _lastFPSTimestamp = DateTime.UtcNow;
    private double _currentFps;
    private Queue<string> _statusMessageQueue = new Queue<string>();
    private MultiSourceFrameReader _reader;
    private ushort[] _depthImagePixels;
    private byte[] _colorImagePixels;
    private int[] _resampledColorImagePixels;
    private int[] _downsampledDeltaFromReferenceColorPixels;
    private ColorReconstruction _volume;
    private FusionFloatImageFrame _depthFloatFrame;
    private FusionFloatImageFrame _smoothDepthFloatFrame;
    private FusionColorImageFrame _resampledColorFrame;
    private FusionColorImageFrame _resampledColorFrameDepthAligned;
    private FusionFloatImageFrame _deltaFromReferenceFrame;
    private FusionColorImageFrame _shadedSurfaceFrame;
    private FusionColorImageFrame _shadedSurfaceNormalsFrame;
    private FusionPointCloudImageFrame _raycastPointCloudFrame;
    private FusionPointCloudImageFrame _depthPointCloudFrame;
    private FusionFloatImageFrame _downsampledDepthFloatFrame;
    private FusionFloatImageFrame _downsampledSmoothDepthFloatFrame;
    private FusionPointCloudImageFrame
      _downsampledRaycastPointCloudFrame;
    private FusionPointCloudImageFrame
      _downsampledDepthPointCloudFrame;
    private FusionColorImageFrame
      _downsampledDeltaFromReferenceFrameColorFrame;
    private float[] _depthFloatFrameDepthPixels;
    private float[] _deltaFromReferenceFrameFloatPixels;
    private int[] _depthFloatFramePixelsArgb;
    private int[] _deltaFromReferenceFramePixelsArgb;
    private ColorSpacePoint[] _colorCoordinates;
    private int[] _resampledColImgPixelsAlignedToDep;
    private float[] _downsampledDepthImagePixels;
    private CoordinateMapper _mapper;
    private float _alignmentEnergy;
    private Thread _workerThread = null;
    private ManualResetEvent _workerThreadStopEvent;
    private ManualResetEvent _depthReadyEvent;
    private ManualResetEvent _colorReadyEvent;
    private object _rawDataLock = new object();
    private object _volumeLock = new object();
    private bool _resetReconstruction = false;
    private bool _recreateReconstruction = false;
    private Matrix4 _worldToCameraTransform;
    private Matrix4 _defaultWorldToVolumeTransform;
    private float _minDepthClip =
      FusionDepthProcessor.DefaultMinimumDepth;
    private float _maxDepthClip =
      FusionDepthProcessor.DefaultMaximumDepth;
    private short _integrationWeight =
      FusionDepthProcessor.DefaultIntegrationWeight;
    private int _voxelsX = 384;
    private int _voxelsY = 384;
    private int _voxelsZ = 384;
    private bool _translateResetPoseByMinDepthThreshold = true;
    private Matrix4 _worldToBGRTransform = new Matrix4();
    private CameraPoseFinder _cameraPoseFinder;
    private bool _autoFindCameraPoseWhenLost = true;
    private TimeSpan _relativeTime;

    private Editor _ed;
    private SynchronizationContext _ctxt;
    private double _roomWidth;
    private double _roomLength;
    private double _roomHeight;
    private int _lowResStep;
    private bool _useMesh;
    private float _voxelsPerMeter;
    private short[] _voxels = null;

    private TextStyle _style;

    // Constructor

    public KinectFusionColorJig(
      Editor ed, SynchronizationContext ctxt,
      double width, double length, double height,
      double vpm, int step, bool useMesh
    )
    {
      _ed = ed;
      _ctxt = ctxt;

      _roomWidth = width;
      _roomLength = length;
      _roomHeight = height;
      _voxelsPerMeter = (float)vpm;
      _voxelsX = (int)(_roomWidth * _voxelsPerMeter);
      _voxelsY = (int)(_roomHeight * _voxelsPerMeter);
      _voxelsZ = (int)(_roomLength * _voxelsPerMeter);

      _lowResStep = step;
      _useMesh = useMesh;

      _vecs = new List<ColoredPoint3d>();
    }

    private void PostToAutoCAD(SendOrPostCallback cb)
    {
      _ctxt.Post(cb, null);
      System.Windows.Forms.Application.DoEvents();
    }

    public override bool StartSensor()
    {
      _kinect = KinectSensor.GetDefault();
      if (_kinect == null)
      {
        _ed.WriteMessage(
          "\nUnable to start Kinect sensor - " +
          "are you sure it's plugged in?"
        );

        return false;
      }

      // Get the coordinate mapper
      
      _mapper = _kinect.CoordinateMapper;

      // Open the sensor
      
      _kinect.Open();

      _reader =
        _kinect.OpenMultiSourceFrameReader(
          FrameSourceTypes.Depth | FrameSourceTypes.Color
        );

      var depthFrameDescription =
        _kinect.DepthFrameSource.FrameDescription;
      _depWidth = depthFrameDescription.Width;
      _depHeight = depthFrameDescription.Height;
      _depPixelCount = _depWidth * _depHeight;

      var colorFrameDescription =
        _kinect.ColorFrameSource.FrameDescription;
      _colWidth = colorFrameDescription.Width;
      _colHeight = colorFrameDescription.Height;
      _colPixelCount = _colWidth * _colHeight;

      // Start worker thread for depth processing
      
      StartWorkerThread();

      // Start fps timer
      
      _fpsTimer = new DispatcherTimer(DispatcherPriority.Send);
      _fpsTimer.Interval = new TimeSpan(0, 0, FpsInterval);
      _fpsTimer.Tick += FpsTimerTick;
      _fpsTimer.Start();

      // Set last fps timestamp as now

      _lastFPSTimestamp = DateTime.UtcNow;

      // Add an event handler to be called whenever depth and
      // color both have new data
      
      _reader.MultiSourceFrameArrived +=
        Reader_MultiSourceFrameArrived;

      // Allocate frames for Kinect Fusion now a sensor is present

      AllocateKinectFusionResources();

      // Set recreate reconstruction flag
      
      _recreateReconstruction = true;

      // Text style for our jig

      _style = new TextStyle();
      _style.Font =
        new FontDescriptor("Calibri", false, false, 0, 0);
      _style.TextSize = 14;

      return true;
    }

    // Handler for FPS timer tick

    private void FpsTimerTick(object sender, EventArgs e)
    {
      // Calculate time span from last calculation of FPS
      
      double intervalSeconds =
        (DateTime.UtcNow - _lastFPSTimestamp).TotalSeconds;

      // Calculate and show fps on status bar
      
      _currentFps =
        (double)(_processedFrameCount / intervalSeconds);

      // Reset frame counter

      _processedFrameCount = 0;
      _lastFPSTimestamp = DateTime.UtcNow;
    }

    // Reset FPS timer and counter

    private void ResetFps()
    {
      // Restart fps timer
      
      if (null != _fpsTimer)
      {
        _fpsTimer.Stop();
        _fpsTimer.Start();
      }

      // Reset frame counter
      
      _processedFrameCount = 0;
      _lastFPSTimestamp = DateTime.UtcNow;
    }

    // Start the work thread to process incoming depth data

    private void StartWorkerThread()
    {
      if (null == _workerThread)
      {
        // Initialize events
        
        _depthReadyEvent = new ManualResetEvent(false);
        _colorReadyEvent = new ManualResetEvent(false);
        _workerThreadStopEvent = new ManualResetEvent(false);

        // Create worker thread and start
        
        _workerThread = new Thread(WorkerThreadProc);
        _workerThread.Start();
      }
    }

    // Stop worker thread

    private void StopWorkerThread()
    {
      if (null != _workerThread)
      {
        // Set stop event to stop thread
        
        _workerThreadStopEvent.Set();

        // Wait for exit of thread
        
        _workerThread.Join();
      }
    }

    // Worker thread in which depth data is processed

    private void WorkerThreadProc()
    {
      var events =
        new WaitHandle[2]
        { _workerThreadStopEvent, _depthReadyEvent };
      
      while (true)
      {
        int index = WaitHandle.WaitAny(events);

        if (0 == index)
        {
          // Stop event has been set. Exit thread
          
          break;
        }

        // Reset depth ready event
        
        _depthReadyEvent.Reset();

        // Pass data to process
        
        Process();
      }
    }

    // Event handler for multiSourceFrame arrived event

    private void Reader_MultiSourceFrameArrived(
      object sender, MultiSourceFrameArrivedEventArgs e
    )
    {
      bool validDepth = false;
      bool validColor = false;

      var frameReference = e.FrameReference;

      MultiSourceFrame multiSourceFrame = null;
      DepthFrame depthFrame = null;
      ColorFrame colorFrame = null;

      try
      {
        multiSourceFrame = frameReference.AcquireFrame();

        if (multiSourceFrame != null)
        {
          // MultiSourceFrame is IDisposable

          lock (_rawDataLock)
          {
            var colorFrameReference =
              multiSourceFrame.ColorFrameReference;
            var depthFrameReference =
              multiSourceFrame.DepthFrameReference;

            colorFrame = colorFrameReference.AcquireFrame();
            depthFrame = depthFrameReference.AcquireFrame();

            if ((depthFrame != null) && (colorFrame != null))
            {
              // Save frame timestamp

              _relativeTime = depthFrame.RelativeTime;

              var colorFrameDescription =
                colorFrame.FrameDescription;
              int colorWidth = colorFrameDescription.Width;
              int colorHeight = colorFrameDescription.Height;

              if (
                (colorWidth * colorHeight * sizeof(int)) ==
                _colorImagePixels.Length
              )
              {
                colorFrame.CopyConvertedFrameDataToArray(
                  _colorImagePixels, ColorImageFormat.Bgra
                );

                validColor = true;
              }

              var depthFrameDescription =
                depthFrame.FrameDescription;
              int depthWidth = depthFrameDescription.Width;
              int depthHeight = depthFrameDescription.Height;

              if (
                (depthWidth * depthHeight) ==
                _depthImagePixels.Length
              )
              {
                depthFrame.CopyFrameDataToArray(_depthImagePixels);

                validDepth = true;
              }
            }
          }
        }
      }
      catch (System.Exception)
      {
        // Ignore if the frame is no longer available
      }
      finally
      {
        // MultiSourceFrame, DepthFrame, ColorFrame, BodyIndexFrame
        // are IDisposable
        
        if (depthFrame != null)
        {
          depthFrame.Dispose();
          depthFrame = null;
        }

        if (colorFrame != null)
        {
          colorFrame.Dispose();
          colorFrame = null;
        }

        if (multiSourceFrame != null)
        {
          multiSourceFrame = null;
        }
      }

      if (validDepth)
      {
        // Signal worker thread to process
        
        _depthReadyEvent.Set();
      }

      if (validColor)
      {
        // Signal worker thread to process
        
        _colorReadyEvent.Set();
      }
    }

    // The main Kinect Fusion process function

    private void Process()
    {
      if (_finished)
        return;

      if (_recreateReconstruction)
      {
        lock (_volumeLock)
        {
          _recreateReconstruction = false;
          RecreateReconstruction();
        }
      }

      if (_resetReconstruction)
      {
        _resetReconstruction = false;
        ResetReconstruction();
      }

      if (null != _volume)
      {
        try
        {
          // Check if camera pose finder is available
          
          _cameraPoseFinderAvailable = IsCameraPoseFinderAvailable();

          // Convert depth to float and render depth frame
          
          ProcessDepthData();

          // Track camera pose
          
          TrackCamera();

          // Only continue if we do not have tracking errors
          
          if (0 == _trackingErrorCount)
          {
            // Integrate depth

            bool colorAvailable = IntegrateData();

            // Update camera pose finder, adding key frames to
            // the database
            
            if (
              _autoFindCameraPoseWhenLost &&
              !_trackingHasFailedPreviously &&
              _successfulFrameCount >
                MinGoodTrackingFramesForCameraPoseFinder &&
              _processedFrameCount %
                CameraPoseFinderProcessFrameCalcInterval == 0
                && colorAvailable
            )
            {
              UpdateCameraPoseFinder();
            }
          }
        }
        catch (InvalidOperationException ex)
        {
          ShowStatusMessage(ex.Message);
        }
      }
    }

    // Is the camera pose finder initialized and running?

    private bool IsCameraPoseFinderAvailable()
    {
      return
        _autoFindCameraPoseWhenLost &&
        null != _cameraPoseFinder &&
        _cameraPoseFinder.GetStoredPoseCount() > 0;
    }

    // Process the depth input for camera tracking

    private void ProcessDepthData()
    {
      if (_finished)
        return;

      // To enable playback of a .xef file through Kinect Studio
      // and reset of the reconstruction if the .xef loops,
      // we test for when the frame timestamp has skipped a
      // large number. 
      // Note: this will potentially continually reset live
      // reconstructions on slow machines which cannot process
      // a live frame in less time than the reset threshold.
      // Increase the number of milliseconds if this is a problem.
      
      if (_autoResetReconstructionOnTimeSkip)
      {
        CheckResetTimeStamp(_relativeTime);
      }

      // Lock the depth operations
      
      lock (_rawDataLock)
      {
        _volume.DepthToDepthFloatFrame(
          _depthImagePixels,
          _depthFloatFrame,
          _minDepthClip,
          _maxDepthClip,
          _mirrorDepth
        );
      }
    }
    
    // Track camera pose by aligning depth float image with
    // reconstruction volume

    private bool TrackCameraAlignDepthFloatToReconstruction(
      bool calculateDeltaFrame, ref Matrix4 calculatedCameraPos
    )
    {
      bool trackingSucceeded = false;

      // Note that here we only calculate the
      // deltaFromReferenceFrame every DeltaFrameCalculationInterval
      // frames to reduce computation time
      
      if (calculateDeltaFrame)
      {
        trackingSucceeded =
          _volume.AlignDepthFloatToReconstruction(
            _depthFloatFrame,
            FusionDepthProcessor.DefaultAlignIterationCount,
            _deltaFromReferenceFrame,
            out _alignmentEnergy,
            _worldToCameraTransform
          );
      }
      else
      {
        // Don't bother getting the residual delta from
        // reference frame to cut computation time
        
        trackingSucceeded =
          _volume.AlignDepthFloatToReconstruction(
            _depthFloatFrame,
            FusionDepthProcessor.DefaultAlignIterationCount,
            null,
            out _alignmentEnergy,
            _worldToCameraTransform
          );
      }

      if (
        !trackingSucceeded ||
        _alignmentEnergy > MaxAlignToReconEnergyForSuccess ||
        (_alignmentEnergy <= MinAlignToReconEnergyForSuccess &&
           _successfulFrameCount > 0
        )
      )
      {
        trackingSucceeded = false;
      }
      else
      {
        // Tracking succeeded, get the updated camera pose

        calculatedCameraPos =
          _volume.GetCurrentWorldToCameraTransform();
      }

      return trackingSucceeded;
    }

    // Track camera pose using AlignPointClouds

    private bool TrackCameraAlignPointClouds(
      ref bool calculateDeltaFrame, ref Matrix4 calculatedCameraPose
    )
    {
      bool trackingSucceeded = false;

      DownsampleDepthFrameNearestNeighbor(
        _downsampledDepthFloatFrame, DownsampleFactor
      );

      // Smooth the depth frame
      
      _volume.SmoothDepthFloatFrame(
        _downsampledDepthFloatFrame,
        _downsampledSmoothDepthFloatFrame,
        SmoothingKernelWidth,
        SmoothingDistanceThreshold
      );

      // Calculate point cloud from the smoothed frame
      
      FusionDepthProcessor.DepthFloatFrameToPointCloud(
        _downsampledSmoothDepthFloatFrame,
        _downsampledDepthPointCloudFrame
      );

      // Get the saved pose view by raycasting the volume from
      // the current camera pose
      
      _volume.CalculatePointCloud(
        _downsampledRaycastPointCloudFrame,
        calculatedCameraPose
      );

      Matrix4 initialPose = calculatedCameraPose;

      // Note that here we only calculate the
      // deltaFromReferenceFrame every
      // DeltaFrameCalculationInterval frames to reduce
      // computation time
      
      if (calculateDeltaFrame)
      {
        trackingSucceeded =
          FusionDepthProcessor.AlignPointClouds(
            _downsampledRaycastPointCloudFrame,
            _downsampledDepthPointCloudFrame,
            FusionDepthProcessor.DefaultAlignIterationCount,
            _downsampledDeltaFromReferenceFrameColorFrame,
            ref calculatedCameraPose
          );

        UpsampleColorDeltasFrameNearestNeighbor(DownsampleFactor);

        // Set calculateDeltaFrame to false as we are
        // rendering it here
        
        calculateDeltaFrame = false;
      }
      else
      {
        // Don't bother getting the residual delta from reference
        // frame to cut computation time
        
        trackingSucceeded =
          FusionDepthProcessor.AlignPointClouds(
            _downsampledRaycastPointCloudFrame,
            _downsampledDepthPointCloudFrame,
            FusionDepthProcessor.DefaultAlignIterationCount,
            null,
            ref calculatedCameraPose
          );
      }

      if (trackingSucceeded)
      {
        bool failed =
          CameraTransformFailed(
            initialPose,
            calculatedCameraPose,
            MaxTranslationDeltaAlignPointClouds,
            MaxRotationDeltaAlignPointClouds
          );

        if (failed)
        {
          trackingSucceeded = false;
        }
      }

      return trackingSucceeded;
    }

    public static bool CameraTransformFailed(
      Matrix4 initial, Matrix4 final,
      float maxTrans, float maxRotDegrees
    )
    {
      // Check if the transform is too far out to be reasonable 
      
      float deltaTrans = maxTrans;
      float angDeg = maxRotDegrees;
      float deltaRot = (angDeg * (float)Math.PI) / 180.0f;

      // Calculate the deltas
      
      float[] eulerInit = RotationMatrixToEulerFloatArray(initial);
      float[] eulerFinal = RotationMatrixToEulerFloatArray(final);

      float[] transInit = ExtractTranslationFloatArray(initial);
      float[] transFinal = ExtractTranslationFloatArray(final);

      bool failRot = false;
      bool failTrans = false;

      float[] eulerDeltas = new float[3];
      float[] transDeltas = new float[3];

      for (int i = 0; i < 3; ++i)
      {
        // Handle when one angle is near PI, and the other near -PI

        if (
          eulerInit[i] >= Math.PI - deltaRot &&
          eulerFinal[i] < deltaRot - Math.PI
        )
        {
          eulerInit[i] -= (float)(Math.PI * 2);
        }
        else if (
          eulerFinal[i] >= Math.PI - deltaRot &&
          eulerInit[i] < deltaRot - Math.PI
        )
        {
          eulerFinal[i] -= (float)(Math.PI * 2);
        }

        eulerDeltas[i] = eulerInit[i] - eulerFinal[i];
        transDeltas[i] = transInit[i] - transFinal[i];

        if (Math.Abs(eulerDeltas[i]) > deltaRot)
        {
          failRot = true;
          break;
        }

        if (Math.Abs(transDeltas[i]) > deltaTrans)
        {
          failTrans = true;
          break;
        }
      }

      return failRot || failTrans;
    }

    // Extract 3x3 rotation matrix from the Matrix4 4x4 transform
    // then convert to Euler angles.

    public static float[] RotationMatrixToEulerFloatArray(
      Matrix4 transform
    )
    {
      float[] rotation = new float[3];

      float phi = (float)Math.Atan2(transform.M23, transform.M33);
      float theta = (float)Math.Asin(-transform.M13);
      float psi = (float)Math.Atan2(transform.M12, transform.M11);

      // This is rotation about x,y,z, or pitch, yaw, roll
      // respectively
      
      rotation[0] = phi;
      rotation[1] = theta;
      rotation[2] = psi;

      return rotation;
    }

    // Extract translation from the Matrix4 transform in M41,M42,M43

    public static float[] ExtractTranslationFloatArray(
      Matrix4 transform
    )
    {
      float[] translation = new float[3];

      translation[0] = transform.M41;
      translation[1] = transform.M42;
      translation[2] = transform.M43;

      return translation;
    }

    // Track the camera pose

    private void TrackCamera()
    {
      bool calculateDeltaFrame =
        _processedFrameCount % DeltaFrameCalculationInterval == 0;
      bool trackingSucceeded = false;

      // Get updated camera transform from image alignment
      
      Matrix4 calculatedCameraPos = _worldToCameraTransform;

      // Here we can either call
      // TrackCameraAlignDepthFloatToReconstruction or
      // TrackCameraAlignPointClouds

      // The TrackCameraAlignPointClouds function typically
      // has higher performance with the camera pose finder 
      // due to its wider basin of convergence, enabling it
      // to more robustly regain tracking from nearby poses
      // suggested by the camera pose finder after tracking
      // is lost

      if (_autoFindCameraPoseWhenLost)
      {
        // Track using AlignPointClouds
        
        trackingSucceeded =
          TrackCameraAlignPointClouds(
            ref calculateDeltaFrame, ref calculatedCameraPos
          );
      }
      else
      {
        // Track using AlignDepthFloatToReconstruction
        
        trackingSucceeded =
          TrackCameraAlignDepthFloatToReconstruction(
            calculateDeltaFrame, ref calculatedCameraPos
          );
      }

      if (!trackingSucceeded && 0 != _successfulFrameCount)
      {
        SetTrackingFailed();

        if (!_cameraPoseFinderAvailable)
        {
          // Show tracking error on status bar
          
          ShowStatusMessageLowPriority("Camera tracking failed!");
        }
        else
        {
          // Here we try to find the correct camera pose, to
          // re-localize camera tracking.
          // We can call either the version using
          // AlignDepthFloatToReconstruction or the version
          // using AlignPointClouds, which typically has a
          // higher success rate.
          // trackingSucceeded =
          //   FindCameraPoseAlignDepthFloatToReconstruction();
          
          trackingSucceeded = FindCameraPoseAlignPointClouds();

          if (!trackingSucceeded)
          {
            // Show tracking error on status bar
            
            ShowStatusMessageLowPriority("Camera tracking failed!");
          }
        }
      }
      else
      {
        if (_trackingHasFailedPreviously)
        {
          ShowStatusMessageLowPriority(
            "Kinect Fusion camera tracking RECOVERED! " +
            " Residual energy=" +
            string.Format(
              CultureInfo.InvariantCulture,
              "{0:0.00000}",
              _alignmentEnergy
            )
          );
        }

        SetTrackingSucceeded();

        _worldToCameraTransform = calculatedCameraPos;
      }

      if (
        AutoResetReconstructionWhenLost &&
        !trackingSucceeded &&
        _trackingErrorCount >= MaxTrackingErrors
      )
      {
        // Bad tracking
        
        ShowStatusMessage("Automatically resetting volume.");

        // Automatically clear Volume and reset tracking on failure

        ResetReconstruction();
      }

      if (trackingSucceeded)
      {
        // Get our points on success

        try
        {
          _points =
            _useMesh ? GetPointCloud(true) : GetPointCloud2(true);
        }
        catch (InvalidOperationException) { }

        // Increase processed frame counter

        _processedFrameCount++;
      }
    }

    // Set variables if camera tracking succeeded

    private void SetTrackingFailed()
    {
      // Clear successful frame count and increment the track
      // error count
      
      _trackingFailed = true;
      _trackingHasFailedPreviously = true;
      _trackingErrorCount++;
      _successfulFrameCount = 0;
    }

    // Set variables if camera tracking succeeded

    private void SetTrackingSucceeded()
    {
      // Clear track error count and increment the successful
      // frame count
      
      _trackingFailed = false;
      _trackingErrorCount = 0;
      _successfulFrameCount++;
    }

    // Reset tracking variables

    private void ResetTracking()
    {
      _trackingFailed = false;
      _trackingHasFailedPreviously = false;
      _trackingErrorCount = 0;
      _successfulFrameCount = 0;

      // Reset pause and signal that the integration resumed
      
      _pauseIntegration = false;

      if (null != _cameraPoseFinder)
      {
        _cameraPoseFinder.ResetCameraPoseFinder();
      }
    }

    // Process input color image to make it equal in size to the
    // depth image

    private unsafe void ProcessColorForCameraPoseFinder()
    {
      if (
        null == _resampledColorFrame ||
        null == _downsampledDepthImagePixels
      )
      {
        throw new ArgumentException("inputs null");
      }

      if (
        _depWidth != RawDepthWidth ||
        _depHeight != RawDepthHeight ||
        _colWidth != RawColorWidth ||
        _colHeight != RawColorHeight
      )
      {
        return;
      }

      float factor = RawColorWidth / RawDepthHeightWithSpecialRatio;
      const int FilledZeroMargin =
        (RawDepthHeight - RawDepthHeightWithSpecialRatio) / 2;

      // Here we make use of unsafe code to just copy the whole
      // pixel as an int for performance reasons, as we do
      // not need access to the individual rgba components.

      fixed (byte* ptrColorPixels = _colorImagePixels)
      {
        int* rawColorPixels = (int*)ptrColorPixels;

        Parallel.For(
          FilledZeroMargin,
          _depHeight - FilledZeroMargin,
          y =>
          {
            int destIndex = y * _depWidth;

            for (int x = 0; x < _depWidth; ++x, ++destIndex)
            {
              int srcX = (int)(x * factor);
              int srcY = (int)(y * factor);
              int sourceColorIndex = (srcY * _colWidth) + srcX;

              _resampledColorImagePixels[destIndex] =
                rawColorPixels[sourceColorIndex];
            }
          }
        );
      }

      _resampledColorFrame.CopyPixelDataFrom(
        _resampledColorImagePixels
      );
    }

    // This is used to set the reference frame

    private void SetReferenceFrame(Matrix4 pose)
    {
      // Get the saved pose view by raycasting the volume
      
      _volume.CalculatePointCloudAndDepth(
        _raycastPointCloudFrame, _smoothDepthFloatFrame, null, pose
      );

      // Set this as the reference frame for the next call
      // to AlignDepthFloatToReconstruction
      
      _volume.SetAlignDepthFloatToReconstructionReferenceFrame(
        _smoothDepthFloatFrame
      );
    }

    // Perform camera pose finding when tracking is lost using
    // AlignPointClouds. This is typically more successful than
    // FindCameraPoseAlignDepthFloatToReconstruction

    private bool FindCameraPoseAlignPointClouds()
    {
      if (!_cameraPoseFinderAvailable)
      {
        return false;
      }

      ProcessColorForCameraPoseFinder();

      var matchCandidates =
        _cameraPoseFinder.FindCameraPose(
          _depthFloatFrame,
          _resampledColorFrame
        );

      if (null == matchCandidates)
      {
        return false;
      }

      int poseCount = matchCandidates.GetPoseCount();
      float minDistance = matchCandidates.CalculateMinimumDistance();

      if (
        0 == poseCount ||
        minDistance >= CameraPoseFinderDistThresholdReject
      )
      {
        ShowStatusMessage("Could really use some more matches.");
        return false;
      }

      // Smooth the depth frame
      
      _volume.SmoothDepthFloatFrame(
        _depthFloatFrame, _smoothDepthFloatFrame,
        SmoothingKernelWidth, SmoothingDistanceThreshold
      );

      // Calculate point cloud from the smoothed frame
      
      FusionDepthProcessor.DepthFloatFrameToPointCloud(
        _smoothDepthFloatFrame, _depthPointCloudFrame
      );

      double smallestEnergy = double.MaxValue;
      int smallestEnergyNeighborIndex = -1;

      int bestNeighborIndex = -1;
      Matrix4 bestNeighborCameraPose = Matrix4.Identity;

      double bestNeighborAlignmentEnergy =
        MaxAlignPointCloudsEnergyForSuccess;

      // Run alignment with best matched poseCount
      // (i.e. k nearest neighbors (kNN))
      
      int maxTests =
        Math.Min(MaxCameraPoseFinderPoseTests, poseCount);

      var neighbors = matchCandidates.GetMatchPoses();

      for (int n = 0; n < maxTests; ++n)
      {
        // Run the camera tracking algorithm with the volume
        // this uses the raycast frame and pose to find a valid
        // camera pose by matching the raycast against the
        // input point cloud
        
        Matrix4 poseProposal = neighbors[n];

        // Get the saved pose view by raycasting the volume
        
        _volume.CalculatePointCloud(
          _raycastPointCloudFrame, poseProposal
        );

        bool success =
          _volume.AlignPointClouds(
            _raycastPointCloudFrame,
            _depthPointCloudFrame,
            FusionDepthProcessor.DefaultAlignIterationCount,
            _resampledColorFrame,
            out _alignmentEnergy,
            ref poseProposal
          );

        bool relocSuccess =
          success &&
          _alignmentEnergy < bestNeighborAlignmentEnergy &&
          _alignmentEnergy > MinAlignPointCloudsEnergyForSuccess;

        if (relocSuccess)
        {
          bestNeighborAlignmentEnergy = _alignmentEnergy;
          bestNeighborIndex = n;

          // This is after tracking succeeds, so should be a
          // more accurate pose to store...
          
          bestNeighborCameraPose = poseProposal;

          // Update the delta image
          
          _resampledColorFrame.CopyPixelDataTo(
            _deltaFromReferenceFramePixelsArgb
          );
        }

        // Find smallest energy neighbor independent of
        // tracking success
        
        if (_alignmentEnergy < smallestEnergy)
        {
          smallestEnergy = _alignmentEnergy;
          smallestEnergyNeighborIndex = n;
        }
      }

      matchCandidates.Dispose();

      // Use the neighbor with smallest residual alignment energy
      // At the cost of additional processing we could also use
      // kNN+Mean camera pose finding here by calculating the
      // mean pose of the best n matched poses and also testing
      // this to see if the residual alignment energy is less
      // than with kNN.
      
      if (bestNeighborIndex > -1)
      {
        _worldToCameraTransform = bestNeighborCameraPose;
        SetReferenceFrame(_worldToCameraTransform);

        // Tracking succeeded!
        
        SetTrackingSucceeded();

        ShowStatusMessageLowPriority(
          "Camera Pose Finder SUCCESS! Residual energy= " +
          string.Format(
            CultureInfo.InvariantCulture,
            "{0:0.00000}",
            bestNeighborAlignmentEnergy
          ) +
          ", " + poseCount +
          " frames stored, minimum distance=" + minDistance +
          ", best match index=" + bestNeighborIndex
        );

        return true;
      }
      else
      {
        _worldToCameraTransform =
          neighbors[smallestEnergyNeighborIndex];
        SetReferenceFrame(_worldToCameraTransform);

        // Camera pose finding failed - return the tracking
        // failed error code
        
        SetTrackingFailed();

        // Tracking Failed will be set again on the next
        // iteration in ProcessDepth
        
        ShowStatusMessageLowPriority(
          "Camera Pose Finder FAILED! Residual energy=" +
          string.Format(
            CultureInfo.InvariantCulture,
            "{0:0.00000}",
            smallestEnergy
          ) +
          ", " + poseCount +
          " frames stored, minimum distance=" + minDistance +
          ", best match index=" + smallestEnergyNeighborIndex
        );

        return false;
      }
    }

    // Perform camera pose finding when tracking is lost
    // using AlignDepthFloatToReconstruction.

    private bool FindCameraPoseAlignDepthFloatToReconstruction()
    {
      if (!_cameraPoseFinderAvailable)
      {
        return false;
      }

      ProcessColorForCameraPoseFinder();

      var matchCandidates =
        _cameraPoseFinder.FindCameraPose(
          _depthFloatFrame,
          _resampledColorFrame
        );

      if (null == matchCandidates)
      {
        return false;
      }

      int poseCount = matchCandidates.GetPoseCount();
      float minDistance = matchCandidates.CalculateMinimumDistance();

      if (
        0 == poseCount ||
        minDistance >= CameraPoseFinderDistThresholdReject
      )
      {
        ShowStatusMessage("Can't find enough camera pose matches!");
        return false;
      }

      double smallestEnergy = double.MaxValue;
      int smallestEnergyNeighborIndex = -1;

      int bestNeighborIndex = -1;
      Matrix4 bestNeighborCameraPose = Matrix4.Identity;

      double bestNeighborAlignmentEnergy =
        MaxAlignToReconEnergyForSuccess;

      // Run alignment with best matched poseCount
      // (i.e. k nearest neighbors (kNN))
      
      int maxTests =
        Math.Min(MaxCameraPoseFinderPoseTests, poseCount);

      var neighbors = matchCandidates.GetMatchPoses();

      for (int n = 0; n < maxTests; ++n)
      {
        // Run the camera tracking algorithm with the volume
        // this uses the raycast frame and pose to find a valid
        // camera pose by matching the depth against the volume
        
        SetReferenceFrame(neighbors[n]);

        bool success =
          _volume.AlignDepthFloatToReconstruction(
            _depthFloatFrame,
            FusionDepthProcessor.DefaultAlignIterationCount,
            _deltaFromReferenceFrame,
            out _alignmentEnergy,
            neighbors[n]
          );

        // Exclude very tiny alignment energy case which
        // is unlikely to happen in reality - this is more
        // likely a tracking error
        
        bool relocSuccess =
          success &&
          _alignmentEnergy < bestNeighborAlignmentEnergy && 
          _alignmentEnergy > MinAlignToReconEnergyForSuccess;

        if (relocSuccess)
        {
          bestNeighborAlignmentEnergy = _alignmentEnergy;
          bestNeighborIndex = n;

          // This is after tracking succeeds, so should be a
          // more accurate pose to store...
          
          bestNeighborCameraPose =
            _volume.GetCurrentWorldToCameraTransform();
        }

        // Find smallest energy neighbor independent of
        // tracking success
        
        if (_alignmentEnergy < smallestEnergy)
        {
          smallestEnergy = _alignmentEnergy;
          smallestEnergyNeighborIndex = n;
        }
      }

      matchCandidates.Dispose();

      // Use the neighbor with smallest residual alignment energy
      // At the cost of additional processing we could also use
      // kNN+Mean camera pose finding here by calculating the
      // mean pose of the best n matched poses and also testing
      // this to see if the residual alignment energy is less
      // than with kNN

      if (bestNeighborIndex > -1)
      {
        _worldToCameraTransform = bestNeighborCameraPose;
        SetReferenceFrame(_worldToCameraTransform);

        // Tracking succeeded!
        
        SetTrackingSucceeded();

        ShowStatusMessage(
          "Camera Pose Finder SUCCESS! Residual energy= " +
          bestNeighborAlignmentEnergy + ", " + poseCount +
          " frames stored, minimum distance=" + minDistance +
          ", best match index=" + bestNeighborIndex
        );

        return true;
      }
      else
      {
        _worldToCameraTransform =
          neighbors[smallestEnergyNeighborIndex];
        SetReferenceFrame(_worldToCameraTransform);

        // Camera pose finding failed - return the tracking
        // failed error code
        
        SetTrackingFailed();

        // Tracking Failed will be set again on the next
        // iteration in ProcessDepth
        
        ShowStatusMessage(
          "Camera Pose Finder FAILED! Residual energy=" +
          smallestEnergy + ", " + poseCount +
          " frames stored, minimum distance=" + minDistance +
          ", best match index=" + smallestEnergyNeighborIndex
        );

        return false;
      }
    }

    // Process the color and depth inputs, converting the color
    // into the depth space

    private unsafe void MapColorToDepth()
    {
      _mapper.MapDepthFrameToColorSpace(
        _depthImagePixels, _colorCoordinates
      );

      lock (_rawDataLock)
      {
        if (_mirrorDepth)
        {
          // Here we make use of unsafe code to just copy the
          // whole pixel as an int for performance reasons, as we
          // do not need access to the individual rgba components

          fixed (byte* ptrColorPixels = _colorImagePixels)
          {
            int* rawColorPixels = (int*)ptrColorPixels;

            Parallel.For(
              0,
              _depHeight,
              y =>
              {
                int destIdx = y * _depWidth;

                for (int x = 0; x < _depWidth; ++x, ++destIdx)
                {
                  // Calculate index into depth array
                  
                  int colorInDepthX =
                    (int)Math.Floor(
                      _colorCoordinates[destIdx].X + 0.5
                    );
                  int colorInDepthY =
                    (int)Math.Floor(
                      _colorCoordinates[destIdx].Y + 0.5
                    );

                  // Make sure the depth pixel maps to a
                  // valid point in color space
                  
                  if (
                    colorInDepthX >= 0 &&
                    colorInDepthX < _colWidth &&
                    colorInDepthY >= 0 &&
                    colorInDepthY < _colHeight &&
                    _depthImagePixels[destIdx] != 0
                  )
                  {
                    // Calculate index into color array
                    
                    int sourceColorIndex =
                      colorInDepthX + (colorInDepthY * _colWidth);

                    // Copy color pixel
                    
                    _resampledColImgPixelsAlignedToDep[destIdx] =
                      rawColorPixels[sourceColorIndex];
                  }
                  else
                  {
                    _resampledColImgPixelsAlignedToDep[destIdx] = 0;
                  }
                }
              }
            );
          }
        }
        else
        {
          // Here we make use of unsafe code to just copy the
          // whole pixel as an int for performance reasons, as we
          // do not need access to the individual rgba components

          fixed (byte* ptrColorPixels = _colorImagePixels)
          {
            int* rawColorPixels = (int*)ptrColorPixels;

            // Horizontal flip the color image as the standard
            // depth image is flipped internally in Kinect Fusion
            // to give a viewpoint as though from behind the Kinect
            // looking forward by default

            Parallel.For(
              0,
              _depHeight,
              y =>
              {
                int destIndex = y * _depWidth;
                
                // Horizontally mirrored
                
                int flipDestIdx = destIndex + (_depWidth - 1);

                for (
                  int x = 0;
                  x < _depWidth;
                  ++x, ++destIndex, --flipDestIdx
                )
                {
                  // Calculate index into depth array
                  
                  int colorInDepthX =
                    (int)Math.Floor(
                      _colorCoordinates[destIndex].X + 0.5
                    );
                  int colorInDepthY =
                    (int)Math.Floor(
                      _colorCoordinates[destIndex].Y + 0.5
                    );

                  // Make sure the depth pixel maps to a valid
                  // point in color space
                  
                  if (
                    colorInDepthX >= 0 &&
                    colorInDepthX < _colWidth &&
                    colorInDepthY >= 0 &&
                    colorInDepthY < _colHeight &&
                    _depthImagePixels[destIndex] != 0
                  )
                  {
                    // Calculate index into color array -
                    // this will perform a horizontal flip as well
                    
                    int sourceColorIndex =
                      colorInDepthX + (colorInDepthY * _colWidth);

                    // Copy color pixel
                    
                    _resampledColImgPixelsAlignedToDep[flipDestIdx] =
                      rawColorPixels[sourceColorIndex];
                  }
                  else
                  {
                    _resampledColImgPixelsAlignedToDep[flipDestIdx] =
                      0;
                  }
                }
              }
            );
          }
        }
      }

      _resampledColorFrameDepthAligned.CopyPixelDataFrom(
        _resampledColImgPixelsAlignedToDep
      );
    }

    // Perform volume depth data integration

    private bool IntegrateData()
    {
      // Color may opportunistically be available here - check
      
      bool colorAvailable = _colorReadyEvent.WaitOne(0);

      // Don't integrate depth data into the volume if:
      // 1) tracking failed
      // 2) camera pose finder is off and we have paused capture
      // 3) camera pose finder is on and we are still under the
      //    MinGoodTrackingFramesForCameraPoseFinderAfterFailure
      //    number of successful frames count.
      
      bool integrateData =
        !_trackingFailed && !_pauseIntegration &&
        (!_cameraPoseFinderAvailable ||
          (_cameraPoseFinderAvailable &&
            !(_trackingHasFailedPreviously &&
              _successfulFrameCount <
              MinGoodTrackingFramesForCameraPoseFinderAfterFailure
            )
          )
        );

      // Integrate the frame to volume
      
      if (integrateData)
      {
        bool integrateColor =
          _processedFrameCount % ColorIntegrationInterval == 0 &&
          colorAvailable;

        // Reset this flag as we are now integrating data again
        
        _trackingHasFailedPreviously = false;

        if (_captureColor && integrateColor)
        {
          // Pre-process color
          
          MapColorToDepth();

          // Integrate color and depth
          
          _volume.IntegrateFrame(
            _depthFloatFrame,
            _resampledColorFrameDepthAligned,
            _integrationWeight,
            FusionDepthProcessor.DefaultColorIntegrationOfAllAngles,
            _worldToCameraTransform
          );
        }
        else
        {
          // Just integrate depth

          _volume.IntegrateFrame(
            _depthFloatFrame,
            _integrationWeight,
            _worldToCameraTransform
          );
        }

        // Reset color ready event
        
        _colorReadyEvent.Reset();
      }

      return colorAvailable;
    }

    // Update the camera pose finder data.

    private void UpdateCameraPoseFinder()
    {
      if (
        null == _depthFloatFrame ||
        null == _resampledColorFrame ||
        null == _cameraPoseFinder
      )
      {
        return;
      }

      ProcessColorForCameraPoseFinder();

      bool poseHistoryTrimmed = false;
      bool addedPose = false;

      // This function will add the pose to the camera pose
      // finding database when the input frame's minimum
      // distance to the existing database is equal to or
      // above CameraPoseFinderDistanceThresholdAccept 
      // (i.e. indicating that the input has become dis-similar
      // to the existing database and a new frame 
      // should be captured). Note that the color and depth
      // frames must be the same size, however, the 
      // horizontal mirroring setting does not have to be
      // consistent between depth and color. It does have
      // to be consistent between camera pose finder database
      // creation and calling FindCameraPose though,
      // hence we always reset both the reconstruction and
      // database when changing the mirror depth setting

      _cameraPoseFinder.ProcessFrame(
        _depthFloatFrame,
        _resampledColorFrame,
        _worldToCameraTransform,
        CameraPoseFinderDistThresholdAccept,
        out addedPose,
        out poseHistoryTrimmed
      );

      if (addedPose)
      {
        ShowStatusMessageLowPriority(
          "Camera Pose Finder Added Frame! " +
          _cameraPoseFinder.GetStoredPoseCount() +
          " frames stored, minimum distance >= " +
          CameraPoseFinderDistThresholdAccept
        );
      }

      if (poseHistoryTrimmed)
      {
        ShowStatusMessage("Pose finder history full.");
      }
    }

    // Check if the gap between 2 frames has reached reset time
    // threshold. If yes, reset the reconstruction

    private void CheckResetTimeStamp(TimeSpan frameTimestamp)
    {
      if (!_lastFrameTimestamp.Equals(TimeSpan.Zero))
      {
        long timeThreshold =
          (ReconstructionProcessor.Amp == ProcessorType) ?
          ResetOnTimeStampSkippedMillisecondsGPU :
          ResetOnTimeStampSkippedMillisecondsCPU;

        // Calculate skipped milliseconds between 2 frames
        
        long skippedMilliseconds =
          (long)frameTimestamp.Subtract(_lastFrameTimestamp).
            Duration().TotalMilliseconds;

        if (skippedMilliseconds >= timeThreshold)
        {
          ShowStatusMessage("Resetting volume.");
          _resetReconstruction = true;
        }
      }

      // Set timestamp of last frame
      
      _lastFrameTimestamp = frameTimestamp;
    }

    // Reset reconstruction object to initial state

    private void ResetReconstruction()
    {
      if (null == _kinect)
      {
        return;
      }

      // Reset tracking error counter

      _trackingErrorCount = 0;

      // Set the world-view transform to identity, so the
      // world origin is the initial camera location
      
      _worldToCameraTransform = Matrix4.Identity;

      // Reset volume

      if (null != _volume)
      {
        try
        {
          // Translate the reconstruction volume location
          // away from the world origin by an amount equal
          // to the minimum depth threshold. This ensures
          // that some depth signal falls inside the volume.
          // If set false, the default world origin is set
          // to the center of the front face of the 
          // volume, which has the effect of locating the
          // volume directly in front of the initial camera
          // position with the +Z axis into the volume
          // along the initial camera direction of view

          if (_translateResetPoseByMinDepthThreshold)
          {
            Matrix4 worldToVolumeTransform =
              _defaultWorldToVolumeTransform;

            // Translate the volume in the Z axis by the
            // minDepthClip distance
            
            float minDist =
              (_minDepthClip < _maxDepthClip) ?
                _minDepthClip :
                _maxDepthClip;

            worldToVolumeTransform.M43 -= minDist * _voxelsPerMeter;

            _volume.ResetReconstruction(
              _worldToCameraTransform,
              worldToVolumeTransform
            );
          }
          else
          {
            _volume.ResetReconstruction(_worldToCameraTransform);
          }

          ResetTracking();
        }
        catch (InvalidOperationException)
        {
          ShowStatusMessage("Unable to reset volume.");
        }
      }

      // Update manual reset information to status bar

      ShowStatusMessage("Resetting volume.");
    }

    // Re-create the reconstruction object

    private bool RecreateReconstruction()
    {
      // Check if sensor has been initialized
      
      if (null == _kinect)
      {
        return false;
      }

      if (null != _volume)
      {
        _volume.Dispose();
        _volume = null;
      }

      try
      {
        var volParam =
          new ReconstructionParameters(
            _voxelsPerMeter, _voxelsX, _voxelsY, _voxelsZ
          );

        // Set the world-view transform to identity,
        // so the world origin is the initial camera location
        
        _worldToCameraTransform = Matrix4.Identity;

        _volume =
          ColorReconstruction.FusionCreateReconstruction(
            volParam, ProcessorType, DeviceToUse,
            _worldToCameraTransform
          );

        _defaultWorldToVolumeTransform =
          _volume.GetCurrentWorldToVolumeTransform();

        if (_translateResetPoseByMinDepthThreshold)
        {
          ResetReconstruction();
        }
        else
        {
          ResetTracking();
        }

        // Map world X axis to blue channel, Y axis to
        // green channel and Z axis to red channel,
        // normalizing each to the range [0, 1].
        // We also add a shift of 0.5 to both X,Y channels
        // as the world origin starts located at the center
        // of the front face of the volume,
        // hence we need to map negative x,y world vertex
        // locations to positive color values

        _worldToBGRTransform = Matrix4.Identity;
        _worldToBGRTransform.M11 = _voxelsPerMeter / _voxelsX;
        _worldToBGRTransform.M22 = _voxelsPerMeter / _voxelsY;
        _worldToBGRTransform.M33 = _voxelsPerMeter / _voxelsZ;
        _worldToBGRTransform.M41 = 0.5f;
        _worldToBGRTransform.M42 = 0.5f;
        _worldToBGRTransform.M44 = 1.0f;

        return true;
      }
      catch (ArgumentException)
      {
        _volume = null;
        ShowStatusMessage("Inappropriate volume resolution.");
      }
      catch (InvalidOperationException ex)
      {
        _volume = null;
        ShowStatusMessage(ex.Message);
      }
      catch (DllNotFoundException)
      {
        _volume = null;
        ShowStatusMessage("Can't find Kinect Fusion module.");
      }
      catch (OutOfMemoryException)
      {
        _volume = null;
        ShowStatusMessage("Out of memory.");
      }

      return false;
    }

    private void ShowStatusMessage(string message)
    {
      PostToAutoCAD(
        a => { _ed.WriteMessage("\n" + message); }
      );

      ResetFps();
    }

    // Show low priority messages in the status bar.
    // Low priority messages do not reset the fps counter,
    // and will only be displayed when high priority message
    // has at least StatusBarInterval seconds shown to the user.
    // Messages that comes in burst or appear extremely
    // frequently should be considered low priority.

    private void ShowStatusMessageLowPriority(string message)
    {
      PostToAutoCAD(
        a => { _ed.WriteMessage("\n" + message); }
      );
    }

    // Allocate the frame buffers and memory used in the
    // process for Kinect Fusion

    private void AllocateKinectFusionResources()
    {
      SafeDisposeFusionResources();

      // Allocate depth float frame

      _depthFloatFrame =
        new FusionFloatImageFrame(_depWidth, _depHeight);

      // Allocate color frame for color data from Kinect
      // mapped into depth frame
      
      _resampledColorFrameDepthAligned =
        new FusionColorImageFrame(_depWidth, _depHeight);

      // Allocate delta from reference frame
      
      _deltaFromReferenceFrame =
        new FusionFloatImageFrame(_depWidth, _depHeight);

      // Allocate point cloud frame
      
      _raycastPointCloudFrame =
        new FusionPointCloudImageFrame(_depWidth, _depHeight);

      // Allocate shaded surface frame

      _shadedSurfaceFrame =
        new FusionColorImageFrame(_depWidth, _depHeight);

      // Allocate shaded surface normals frame
      
      _shadedSurfaceNormalsFrame =
        new FusionColorImageFrame(_depWidth, _depHeight);

      // Allocate re-sampled color at depth image size
      
      _resampledColorFrame =
        new FusionColorImageFrame(_depWidth, _depHeight);

      // Allocate point cloud frame created from input depth
      
      _depthPointCloudFrame =
        new FusionPointCloudImageFrame(_depWidth, _depHeight);

      // Allocate smoothed depth float frame
      
      _smoothDepthFloatFrame =
        new FusionFloatImageFrame(_depWidth, _depHeight);

      _downsampledWidth = _depWidth / DownsampleFactor;
      _downsampledHeight = _depHeight / DownsampleFactor;

      // Allocate downsampled image frames
      
      _downsampledDepthFloatFrame =
        new FusionFloatImageFrame(
          _downsampledWidth, _downsampledHeight
        );

      _downsampledSmoothDepthFloatFrame =
        new FusionFloatImageFrame(
          _downsampledWidth, _downsampledHeight
        );

      _downsampledRaycastPointCloudFrame =
        new FusionPointCloudImageFrame(
          _downsampledWidth, _downsampledHeight
        );

      _downsampledDepthPointCloudFrame =
        new FusionPointCloudImageFrame(
          _downsampledWidth, _downsampledHeight
        );

      _downsampledDeltaFromReferenceFrameColorFrame =
        new FusionColorImageFrame(
          _downsampledWidth, _downsampledHeight
        );

      int depthImageSize = _depWidth * _depHeight;
      int colorImageByteSize = _colWidth * _colHeight * sizeof(int);
      int downsampledDepthImageSize =
        _downsampledWidth * _downsampledHeight;

      // Create local depth pixels buffer
      
      _depthImagePixels = new ushort[depthImageSize];

      // Create local color pixels buffer
      
      _colorImagePixels = new byte[colorImageByteSize];

      // Create local color pixels buffer re-sampled to depth size
      
      _resampledColorImagePixels = new int[depthImageSize];

      // Create float pixel array
      
      _depthFloatFrameDepthPixels = new float[depthImageSize];

      // Create float pixel array
      
      _deltaFromReferenceFrameFloatPixels =
        new float[depthImageSize];

      // Create colored pixel array of correct format
      
      _depthFloatFramePixelsArgb = new int[depthImageSize];

      // Create colored pixel array of correct format
      
      _deltaFromReferenceFramePixelsArgb = new int[depthImageSize];

      // Allocate the depth-color mapping points
      
      _colorCoordinates = new ColorSpacePoint[depthImageSize];

      // Allocate color points re-sampled to depth size
      // mapped into depth frame of reference
      
      _resampledColImgPixelsAlignedToDep = new int[depthImageSize];

      // Allocate downsampled image arrays
      
      _downsampledDepthImagePixels =
        new float[downsampledDepthImageSize];

      _downsampledDeltaFromReferenceColorPixels =
        new int[downsampledDepthImageSize];

      // Create a camera pose finder with default parameters
      
      _cameraPoseFinder =
        CameraPoseFinder.FusionCreateCameraPoseFinder(
          CameraPoseFinderParameters.Defaults
        );
    }

    // Dispose fusion resources safely

    private void SafeDisposeFusionResources()
    {
      if (null != _depthFloatFrame)
      {
        _depthFloatFrame.Dispose();
      }

      if (null != _resampledColorFrameDepthAligned)
      {
        _resampledColorFrameDepthAligned.Dispose();
      }

      if (null != _deltaFromReferenceFrame)
      {
        _deltaFromReferenceFrame.Dispose();
      }

      if (null != _raycastPointCloudFrame)
      {
        _raycastPointCloudFrame.Dispose();
      }

      if (null != _shadedSurfaceFrame)
      {
        _shadedSurfaceFrame.Dispose();
      }

      if (null != _shadedSurfaceNormalsFrame)
      {
        _shadedSurfaceNormalsFrame.Dispose();
      }

      if (null != _resampledColorFrame)
      {
        _resampledColorFrame.Dispose();
      }

      if (null != _depthPointCloudFrame)
      {
        _depthPointCloudFrame.Dispose();
      }

      if (null != _smoothDepthFloatFrame)
      {
        _smoothDepthFloatFrame.Dispose();
      }

      if (null != _downsampledDepthFloatFrame)
      {
        _downsampledDepthFloatFrame.Dispose();
      }

      if (null != _downsampledSmoothDepthFloatFrame)
      {
        _downsampledSmoothDepthFloatFrame.Dispose();
      }

      if (null != _downsampledRaycastPointCloudFrame)
      {
        _downsampledRaycastPointCloudFrame.Dispose();
      }

      if (null != _downsampledDepthPointCloudFrame)
      {
        _downsampledDepthPointCloudFrame.Dispose();
      }

      if (null != _downsampledDeltaFromReferenceFrameColorFrame)
      {
        _downsampledDeltaFromReferenceFrameColorFrame.Dispose();
      }

      if (null != _cameraPoseFinder)
      {
        _cameraPoseFinder.Dispose();
      }
    }

    // Copy a color frame to a fusion color image 

    private void CopyColorImageFrame(FusionColorImageFrame dest)
    {
      if (
        null == dest ||
        null == _colorImagePixels ||
        null == _resampledColorImagePixels
      )
      {
        throw new ArgumentException("dest == null");
      }

      if (dest.Width != _colWidth || dest.Height != _colHeight)
      {
        throw new ArgumentException("dest != color image size");
      }

      // Copy to intermediate buffer
      
      System.Buffer.BlockCopy(
        _colorImagePixels, 0, _resampledColorImagePixels,
        0, _colPixelCount * sizeof(int)
      );

      // Copy into Kinect Fusion image
      
      dest.CopyPixelDataFrom(_resampledColorImagePixels);
    }

    // Downsample depth pixels with nearest neighbor

    private unsafe void DownsampleDepthFrameNearestNeighbor(
      FusionFloatImageFrame dest, int factor
    )
    {
      if (null == dest || null == _downsampledDepthImagePixels)
      {
        throw new ArgumentException("inputs null");
      }

      if (
        !(2 == factor || 4 == factor || 8 == factor || 16 == factor)
      )
      {
        throw new ArgumentException("factor != 2, 4, 8 or 16");
      }

      int downsampleWidth = _depWidth / factor;
      int downsampleHeight = _depHeight / factor;

      if (
        dest.Width != downsampleWidth ||
        dest.Height != downsampleHeight
      )
      {
        throw new ArgumentException(
          "dest != downsampled image size"
        );
      }

      if (_mirrorDepth)
      {
        fixed (ushort* rawDepthPixelPtr = _depthImagePixels)
        {
          ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;

          Parallel.For(
            0,
            downsampleHeight,
            y =>
            {
              int destIndex = y * downsampleWidth;
              int sourceIndex = y * _depWidth * factor;

              for (
                int x = 0;
                x < downsampleWidth;
                ++x, ++destIndex, sourceIndex += factor
              )
              {
                // Copy depth value
                
                _downsampledDepthImagePixels[destIndex] =
                  (float)rawDepthPixels[sourceIndex] * 0.001f;
              }
            }
          );
        }
      }
      else
      {
        fixed (ushort* rawDepthPixelPtr = _depthImagePixels)
        {
          ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;

          // Horizontal flip the color image as the standard
          // depth image is flipped internally in Kinect Fusion
          // to give a viewpoint as though from behind the
          // Kinect looking forward by default.
          
          Parallel.For(
            0,
            downsampleHeight,
            y =>
            {
              int flippedDestIndex =
                (y * downsampleWidth) + (downsampleWidth - 1);
              int sourceIndex = y * _depWidth * factor;

              for (
                int x = 0;
                x < downsampleWidth;
                ++x, --flippedDestIndex, sourceIndex += factor
              )
              {
                // Copy depth value
                
                _downsampledDepthImagePixels[flippedDestIndex] =
                  (float)rawDepthPixels[sourceIndex] * 0.001f;
              }
            }
          );
        }
      }

      dest.CopyPixelDataFrom(_downsampledDepthImagePixels);
    }

    // Up sample color delta from reference frame with
    // nearest neighbor - replicates pixels

    private unsafe void UpsampleColorDeltasFrameNearestNeighbor(
      int factor
    )
    {
      if (
        null == _downsampledDeltaFromReferenceFrameColorFrame ||
        null == _downsampledDeltaFromReferenceColorPixels ||
        null == _deltaFromReferenceFramePixelsArgb
      )
      {
        throw new ArgumentException("inputs null");
      }

      if (
        !(2 == factor || 4 == factor || 8 == factor || 16 == factor)
      )
      {
        throw new ArgumentException("factor != 2, 4, 8 or 16");
      }

      int upsampleWidth = _downsampledWidth * factor;
      int upsampleHeight = _downsampledHeight * factor;

      if (
        _depWidth != upsampleWidth || _depHeight != upsampleHeight
      )
      {
        throw new ArgumentException(
          "upsampled image size != depth size"
        );
      }

      _downsampledDeltaFromReferenceFrameColorFrame.CopyPixelDataTo(
        _downsampledDeltaFromReferenceColorPixels
      );

      int upsampleRowMultiplier = upsampleWidth * factor;

      // Here we make use of unsafe code to just copy the
      // whole pixel as an int for performance reasons, as we do
      // not need access to the individual rgba components
      
      fixed (
        int* rawColorPixelPtr =
          _downsampledDeltaFromReferenceColorPixels
      )
      {
        int* rawColorPixels = (int*)rawColorPixelPtr;

        // Note we run this only for the source image height
        // pixels to sparsely fill the destination with rows
        
        Parallel.For(
          0,
          _downsampledHeight,
          y =>
          {
            int destIndex = y * upsampleRowMultiplier;
            int sourceColorIndex = y * _downsampledWidth;

            for (
              int x = 0; x < upsampleWidth; ++x, ++sourceColorIndex
            )
            {
              int color = rawColorPixels[sourceColorIndex];

              // Replicate pixels horizontally
              
              for (int s = 0; s < factor; ++s, ++destIndex)
              {
                // Copy color pixel
                
                _deltaFromReferenceFramePixelsArgb[destIndex] =
                  color;
              }
            }
          }
        );
      }

      int sizeOfInt = sizeof(int);
      int rowByteSize = _downsampledHeight * sizeOfInt;

      // Duplicate the remaining rows with memcpy

      for (int y = 0; y < _downsampledHeight; ++y)
      {
        // Iterate only for the smaller number of rows
        
        int srcRowIndex = upsampleRowMultiplier * y;

        // Duplicate lines
        
        for (int r = 1; r < factor; ++r)
        {
          int index = upsampleWidth * ((y * factor) + r);

          System.Buffer.BlockCopy(
            _deltaFromReferenceFramePixelsArgb,
            srcRowIndex * sizeOfInt,
            _deltaFromReferenceFramePixelsArgb,
            index * sizeOfInt, rowByteSize
          );
        }
      }
    }

    protected override SamplerStatus SamplerData()
    {
      if (_vecs.Count > 0)
      {
        _points.Clear();

        foreach (var vec in _vecs)
        {
          _points.Add(
            new Point3d(vec.X, vec.Y, vec.Z)
          );
        }
      }

      ForceMessage();

      return SamplerStatus.OK;
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      var wg = draw.Geometry;

      // Push our transforms onto the stack

      wg.PushOrientationTransform(
        OrientationBehavior.Screen
      );

      wg.PushPositionTransform(
        PositionBehavior.Screen,
        new Point2d(30, 30)
      );

      // Draw our screen-fixed text

      wg.Text(
        new Point3d(0, 0, 0),  // Position
        new Vector3d(0, 0, 1), // Normal
        new Vector3d(1, 0, 0), // Direction
        String.Format("{0:F1} FPS", _currentFps), // Text
        true,                  // Rawness
        _style                 // TextStyle
      );

      // Remember to pop our transforms off the stack

      wg.PopModelTransform();
      wg.PopModelTransform();

      return base.WorldDrawData(draw);
    }

    public ColorMesh GetMesh()
    {
      return _volume.CalculateMesh(1);
    }

    // Get a point cloud from the vertices of a mesh
    // (would be better to access the volume info directly)

    public Point3dCollection GetPointCloud(bool lowRes = false)
    {
      using (var m = _volume.CalculateMesh(lowRes ? _lowResStep : 1))
      {
        return
          m != null ?
            ColorUtils.Point3dFromVertCollection(
              m.GetVertices()
            ) :
            new Point3dCollection();
      }
    }

    public List<ColoredPoint3d> GetColoredPointCloud(int step)
    {
      using (var m = _volume.CalculateMesh(step))
      {
        return ColorUtils.ColoredPoint3FromVertCollection(
          m.GetVertices(), m.GetColors()
        );
      }
    }

    // Get a point cloud from the volume directly

    public Point3dCollection GetPointCloud2(bool lowRes = false)
    {
      var step = lowRes ? _lowResStep : 1;
      var res = (double)(_voxelsPerMeter / step);
      var destResX = (int)(_roomWidth * res);
      var destResY = (int)(_roomHeight * res);
      var destResZ = (int)(_roomLength * res);
      var destRes = destResX * destResY * destResZ;
      var offx = _roomWidth / -2.0;
      var offy = _roomHeight / -2.0;
      var offz = 0.0; //_roomLength / -2.0;

      var pts = new Point3dCollection();

      if (_voxels == null || _voxels.Length != destRes)
      {
        _voxels = new short[destRes];
      }

      try
      {
        _volume.ExportVolumeBlock(
          0, 0, 0, destResX, destResY, destResZ, step, _voxels
        );

        var pitch = destResX;
        var slice = destResY * pitch;

        for (int x = 0; x < destResX; ++x)
        {
          for (int y = 0; y < destResY; ++y)
          {
            for (int z = 0; z < destResZ; ++z)
            {
              var vox = (int)(_voxels[z * slice + y * pitch + x]);
              var v = (double)((vox | 0xFFFF) / 0xFFFF);
              if (v > 0.0)
              {
                pts.Add(
                  new Point3d(
                    x / res + offx,
                    z / res + offz,
                    -(y / res + offy)
                  )
                );
              }
            }
          }
        }
      }
      catch { }
      return pts;
    }

    public List<ColoredPoint3d> GetColoredPointCloud2(int step)
    {
      var res = (double)(_voxelsPerMeter / step);
      var destResX = (int)(_roomWidth * res);
      var destResY = (int)(_roomHeight * res);
      var destResZ = (int)(_roomLength * res);
      var destRes = destResX * destResY * destResZ;
      var offx = _roomWidth / -2.0;
      var offy = _roomHeight / -2.0;
      var offz = 0.0; //_roomLength / -2.0;

      if (_voxels == null || _voxels.Length != destRes)
      {
        _voxels = new short[destRes];
      }
      var colors = new int[destRes];

      var pts = new List<ColoredPoint3d>();

      try
      {
        _volume.ExportVolumeBlock(
          0, 0, 0, destResX, destResY, destResZ, step, _voxels,
          colors
        );

        var pitch = destResX;
        var slice = destResY * pitch;

        for (int x = 0; x < destResX; ++x)
        {
          for (int y = 0; y < destResY; ++y)
          {
            for (int z = 0; z < destResZ; ++z)
            {
              var idx = z * slice + y * pitch + x;
              var vox = (int)_voxels[idx];
              var v = (double)((vox | 0xFFFF) / 0xFFFF);
              if (v > 0.0)
              {
                int col = colors[idx];
                pts.Add(
                  new ColoredPoint3d(
                    x / res + offx,
                    z / res + offz,
                    -(y / res + offy),
                    (col >> 16) & 255,
                    (col >> 8) & 255,
                    col & 255
                  )
                );
              }
            }
          }
        }
      }
      catch { }
      return pts;
    }

    // Experimental command to generate a mesh rather than a
    // point cloud

    internal void CreateAndAddMesh(Document doc, int voxelStep)
    {
      if (_volume == null)
        return;

      // Calculate the mesh with the requested voxel step and
      // extract the vertices and face indices

      var mesh = _volume.CalculateMesh(voxelStep);
      var verts = mesh.GetVertices();
      var cols = mesh.GetColors();
      var indices = mesh.GetTriangleIndexes();

      // Get our points to create the SubDMesh

      var points = new Point3dCollection();
      foreach (var vert in verts)
      {
        points.Add(new Point3d(vert.X, vert.Z, -vert.Y));
      }

      // And the corresponding colors

      var colors = new EntityColor[verts.Count];      
      for (int i = 0; i < cols.Count; ++i)
      {
        var col = cols[i];
        colors[i] =
          new EntityColor(
            (byte)((col >> 16) & 255),
            (byte)((col >> 8) & 255),
            (byte)(col & 255)
          );
      }

      // And the face indices, of course

      var faces = new Int32Collection();
      for (int i = 0; i < indices.Count; i += 3)
      {
        faces.Add(3);
        faces.Add(indices[i]);
        faces.Add(indices[i + 1]);
        faces.Add(indices[i + 2]);
      }

      // Create the SubDMesh and add it to modelspace

      var sdm = new SubDMesh();
      try
      {
        sdm.SetSubDMesh(points, faces, 0);
      }
      catch (Autodesk.AutoCAD.Runtime.Exception)
      {
        _ed.WriteMessage("\nCould not create mesh.");
        sdm.Dispose();
        return;
      }

      using (var tr = doc.TransactionManager.StartTransaction())
      {
        var bt =
          (BlockTable)tr.GetObject(
            doc.Database.BlockTableId, OpenMode.ForRead
          );
        var btr =
          (BlockTableRecord)tr.GetObject(
            bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite
          );

        btr.AppendEntity(sdm);
        tr.AddNewlyCreatedDBObject(sdm, true);

        sdm.VertexColorArray = colors;
  
        tr.Commit();
      }
    }

    public void CleanUp()
    {
      SafeDisposeFusionResources();

      if (null != _style)
      {
        _style.Dispose();
        _style = null;
      }
    }
  }

  public class KinectFusionColorCommands
  {
    private const int RoomWidth = 4;
    private const int RoomHeight = 2;
    private const int RoomLength = 2;
    private const int VoxelsPerMeter = 128;
    private const int LowResStep = 4;
    private const bool UseMesh = true;

    private double _roomWidth = RoomWidth;
    private double _roomLength = RoomLength;
    private double _roomHeight = RoomHeight;
    private double _voxelsPerMeter = VoxelsPerMeter;
    private int _lowResStep = LowResStep;
    private bool _useMesh = UseMesh;

    [CommandMethod("ADNPLUGINS", "KINFUS", CommandFlags.Modal)]
    public void ImportFromKinectFusionWithColor()
    {
      var doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      var db = doc.Database;
      var ed = doc.Editor;

      var kinect = KinectSensor.GetDefault();      
      if (kinect == null)
      {
        ed.WriteMessage(
          "\nUnable to start Kinect sensor - " +
          "are you sure it's plugged in?"
        );

        return;
      }


      // Ask the user for double information

      var pdo = new PromptDoubleOptions("\nEnter width of volume");
      pdo.AllowNegative = false;
      pdo.AllowZero = false;
      pdo.DefaultValue = _roomWidth;
      pdo.UseDefaultValue = true;

      var pdr = ed.GetDouble(pdo);
      if (pdr.Status != PromptStatus.OK)
        return;

      _roomWidth = pdr.Value;

      pdo.Message = "\nEnter length of volume";
      pdo.DefaultValue = _roomLength;
      pdr = ed.GetDouble(pdo);
      if (pdr.Status != PromptStatus.OK)
        return;

      _roomLength = pdr.Value;

      pdo.Message = "\nEnter height of volume";
      pdo.DefaultValue = _roomHeight;
      pdr = ed.GetDouble(pdo);
      if (pdr.Status != PromptStatus.OK)
        return;

      _roomHeight = pdr.Value;

      // Ask the user for integer information

      pdo.Message = "\nEnter voxels per meter";
      pdo.AllowNegative = false;
      pdo.AllowZero = false;
      pdo.DefaultValue = _voxelsPerMeter;
      pdo.UseDefaultValue = true;

      pdr = ed.GetDouble(pdo);
      if (pdr.Status != PromptStatus.OK)
        return;

      _voxelsPerMeter = pdr.Value;

      var pio =
        new PromptIntegerOptions("\nLow resolution sampling");
      pio.DefaultValue = _lowResStep;

      var pir = ed.GetInteger(pio);
      if (pir.Status != PromptStatus.OK)
        return;

      _lowResStep = pir.Value;

      // Ask the user for keyword information

      var pko =
        new PromptKeywordOptions(
          "\nUse a mesh object to calculate points?"
        );
      pko.AllowNone = true;
      pko.Keywords.Add("Yes");
      pko.Keywords.Add("No");
      pko.Keywords.Default = _useMesh ? "Yes" : "No";

      var pkr = ed.GetKeywords(pko);
      if (pkr.Status != PromptStatus.OK)
        return;

      _useMesh = (pkr.StringResult == "Yes");

      // Create a form to set the sync context properly

      using (var f1 = new Form1())
      {
        var ctxt = SynchronizationContext.Current;
        if (ctxt == null)
        {
          throw
            new System.Exception(
              "Current sync context is null."
            );
        }

        // Create our jig

        var kj =
          new KinectFusionColorJig(
            ed, ctxt,
            _roomWidth, _roomLength, _roomHeight,
            _voxelsPerMeter, _lowResStep, _useMesh
          );

        if (!kj.StartSensor())
        {
          kj.StopSensor();
          kj.CleanUp();
          return;
        }

        var pr = ed.Drag(kj);
        if (pr.Status != PromptStatus.OK && !kj.Finished)
        {
          kj.StopSensor();
          kj.CleanUp();
          return;
        }

        kj.PauseSensor();

        try
        {
          ed.WriteMessage(
            "\nCapture complete: examining points...\n"
          );

          System.Windows.Forms.Application.DoEvents();

          var voxelStep = 1;
          bool loop = false, cancel = false;
          List<ColoredPoint3d> pts;

          do
          {
            loop = false;

            pts =
              _useMesh ?
              kj.GetColoredPointCloud(voxelStep) :
              kj.GetColoredPointCloud2(voxelStep);

            ed.WriteMessage(
              "Extracted mesh data: {0} vertices with" +
              " voxel step of {1}.\n",
              pts.Count, voxelStep
            );

            var pio2 =
              new PromptIntegerOptions(
                "Enter new voxel step or accept default to continue"
              );
            pio2.AllowNegative = false;
            pio2.AllowZero = false;
            pio2.DefaultValue = voxelStep;

            var pir2 = ed.GetInteger(pio2);
            if (pir2.Status != PromptStatus.OK)
            {
              cancel = true;
            }
            else if (
              pir2.Status == PromptStatus.OK &&
              pir2.Value != voxelStep
            )
            {
              voxelStep = pir2.Value;
              loop = true;
            }
          } while (loop);

          if (!cancel)
          {
            pko.Message =
              "\nGenerate a mesh instead of a point cloud?";
            pko.Keywords.Default = "No";

            pkr = ed.GetKeywords(pko);

            if (pkr.Status != PromptStatus.OK)
            {
              kj.StopSensor();
              kj.CleanUp();
              return;
            }

            if (pkr.StringResult == "Yes")
            {
              kj.CreateAndAddMesh(doc, voxelStep);
            }
            else
            {
              kj.WriteAndImportPointCloud(doc, pts);
            }
          }
        }
        catch (System.Exception ex)
        {
          ed.WriteMessage("\nException: {0}", ex.Message);
        }
        kj.StopSensor();
        kj.CleanUp();
      }
    }
  }
}
