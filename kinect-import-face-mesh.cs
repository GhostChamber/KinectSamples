using System;
using System.Collections.Generic;
using System.Globalization;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.Colors;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;

namespace KinectSamples
{
  public class KinectFaceMeshJig : KinectJig
  {
    private BodyFrameSource _bodySource = null;
    private BodyFrameReader _bodyReader = null;
    private HighDefinitionFaceFrameSource _hdFaceFrameSource = null;
    private HighDefinitionFaceFrameReader _hdFaceFrameReader = null;
    private FaceAlignment _currentFaceAlignment = null;
    private FaceModel _currentFaceModel = null;
    private FaceModelBuilder _faceModelBuilder = null;
    private Microsoft.Kinect.Body _currentTrackedBody = null;
    private ulong _currentTrackingId = 0;
    private string _currentBuilderStatus = string.Empty;
    private string _currentCollectionStatus = string.Empty;

    private IReadOnlyList<uint> _triangleIndices;
    private bool _displayFaceMesh;

    private Point3dCollection _polygonPoints, _polygonPositions;
    private UInt32Collection _numPolygonPositions, _numPolygonPoints;
    EntityColorCollection _outlineColors, _fillColors;
    private LinetypeCollection _outlineTypes;
    private TransparencyCollection _fillOpacities;

    private TextStyle _style;

    public KinectFaceMeshJig()
    {
      _displayFaceMesh = false;
      _polygonPoints = new Point3dCollection();
      _polygonPositions = new Point3dCollection();
      _numPolygonPositions = new UInt32Collection();
      _numPolygonPoints = new UInt32Collection();
      _outlineColors = new EntityColorCollection();
      _fillColors = new EntityColorCollection();
      _outlineTypes = new LinetypeCollection();
      _fillOpacities = new TransparencyCollection();
    }

    public override bool StartSensor()
    {
      _bodySource = _kinect.BodyFrameSource;
      _bodyReader = _bodySource.OpenReader();
      _bodyReader.FrameArrived += BodyReader_FrameArrived;

      _hdFaceFrameSource =
        new HighDefinitionFaceFrameSource(_kinect);
      _hdFaceFrameSource.TrackingIdLost +=
        HdFaceSource_TrackingIdLost;

      _hdFaceFrameReader =
        _hdFaceFrameSource.OpenReader();
      _hdFaceFrameReader.FrameArrived +=
        HdFaceReader_FrameArrived;

      _currentFaceModel = new FaceModel();
      _currentFaceAlignment = new FaceAlignment();

      InitializeMesh();
      UpdateMesh();

      // Text style for our jig

      _style = new TextStyle();
      _style.Font =
        new FontDescriptor("standard.shx", false, false, 0, 0);
      _style.TextSize = 10;

      var res = base.StartSensor();
      if (res)
      {
        if (_faceModelBuilder != null)
        {
          _faceModelBuilder.Dispose();
        }
        _faceModelBuilder =
          _hdFaceFrameSource.OpenModelBuilder(
            FaceModelBuilderAttributes.None
          );
        _faceModelBuilder.BeginFaceDataCollection();
        _faceModelBuilder.CollectionCompleted +=
          HdFaceBuilder_CollectionCompleted;
      }
      return res;
    }

    public override void StopSensor()
    {
      if (_faceModelBuilder != null)
      {
        _faceModelBuilder.Dispose();
        _faceModelBuilder = null;
      }

      base.StopSensor();
    }

    // Returns the length of a vector from origin

    private static double VectorLength(CameraSpacePoint point)
    {
      var result =
        Math.Pow(point.X, 2) +
        Math.Pow(point.Y, 2) +
        Math.Pow(point.Z, 2);

      result = Math.Sqrt(result);

      return result;
    }

    // Finds the closest body from the sensor if any

    private static Microsoft.Kinect.Body FindClosestBody(
      BodyFrame bodyFrame
    )
    {
      Microsoft.Kinect.Body result = null;
      double closestBodyDist = double.MaxValue;

      var bodies = new Microsoft.Kinect.Body[bodyFrame.BodyCount];

      bodyFrame.GetAndRefreshBodyData(bodies);

      foreach (var body in bodies)
      {
        if (body.IsTracked)
        {
          var curLoc = body.Joints[JointType.SpineBase].Position;
          var curDist = VectorLength(curLoc);

          if (result == null || curDist < closestBodyDist)
          {
            result = body;
            closestBodyDist = curDist;
          }
        }
      }

      return result;
    }

    // Find if there is a body tracked with the given trackingId

    private static Microsoft.Kinect.Body FindBodyWithTrackingId(
      BodyFrame bodyFrame, ulong trackingId
    )
    {
      Microsoft.Kinect.Body result = null;

      var bodies = new Microsoft.Kinect.Body[bodyFrame.BodyCount];
        
      bodyFrame.GetAndRefreshBodyData(bodies);

      foreach (var body in bodies)
      {
        if (body.IsTracked)
        {
          if (body.TrackingId == trackingId)
          {
            result = body;
            break;
          }
        }
      }

      return result;
    }

    // Gets the current collection status

    private static string GetCollectionStatusText(
      FaceModelBuilderCollectionStatus status
    )
    {
      string res = string.Empty;

      if (
        (status &
           FaceModelBuilderCollectionStatus.FrontViewFramesNeeded
        ) != 0)
      {
        res = "FrontViewFramesNeeded";
        return res;
      }

      if (
        (status &
          FaceModelBuilderCollectionStatus.LeftViewsNeeded
        ) != 0
      )
      {
        res = "LeftViewsNeeded";
        return res;
      }

      if (
        (status &
          FaceModelBuilderCollectionStatus.RightViewsNeeded
        ) != 0
      )
      {
        res = "RightViewsNeeded";
        return res;
      }

      if (
        (status &
          FaceModelBuilderCollectionStatus.TiltedUpViewsNeeded
        ) != 0
      )
      {
        res = "TiltedUpViewsNeeded";
        return res;
      }

      if (
        (status &
          FaceModelBuilderCollectionStatus.Complete
        ) != 0
      )
      {
        res = "Complete";
        return res;
      }

      if (
        (status &
          FaceModelBuilderCollectionStatus.MoreFramesNeeded
        ) != 0
      )
      {
        res = "TiltedUpViewsNeeded";
        return res;
      }

      return res;
    }

    // Initializes a 3D mesh to deform every frame
    
    private void InitializeMesh()
    {
      var vertices =
        _currentFaceModel.CalculateVerticesForAlignment(
          _currentFaceAlignment
        );

      _triangleIndices = _currentFaceModel.TriangleIndices;

      for (int i = 0; i < _triangleIndices.Count; i += 3)
      {
        // Set the initial points to be blank
        // (will be updated immediately)

        for (int j = 0; j < 3; j++)
        {
          _polygonPoints.Add(Point3d.Origin);
        }

        // We want a 0 offset for each polygon

        _polygonPositions.Add(Point3d.Origin);

        // Although each has 3 sides, each has different lengths
        // so we treat them all seperately rather than instanced

        _numPolygonPositions.Add(1);
        
        // Each will have three vertices

        _numPolygonPoints.Add(3);

        // Let's go with a lighter grey outline for each face

        _outlineColors.Add(
          new EntityColor(ColorMethod.ByAci, 8)
        );

        // And a darker grey fill

        _fillColors.Add(
          new EntityColor(ColorMethod.ByAci, 9)
        );
        
        // Solid linestyle for the outline

        _outlineTypes.Add(Linetype.Solid);
        
        // And full opacity

        _fillOpacities.Add(new Transparency(255));
      }
    }

    // Sends the new deformed mesh to be drawn

    private void UpdateMesh()
    {
      var vertices =
        _currentFaceModel.CalculateVerticesForAlignment(
          _currentFaceAlignment
        );

      // Generate a set of points for display

      for (
        int faceIdx = 0; faceIdx < _polygonPositions.Count; ++faceIdx
      )
      {
        var i = faceIdx * 3;

        for (int j = 0; j < 3; j++)
        {
          var vert = vertices[(int)_triangleIndices[i + j]];
          _polygonPoints[i + j] =
            new Point3d(vert.X, vert.Y, -vert.Z);
        }
      }
      _displayFaceMesh = true;
    }

    // This event fires when a BodyFrame is ready for consumption

    private void BodyReader_FrameArrived(
      object sender, BodyFrameArrivedEventArgs e
    )
    {
      CheckOnBuilderStatus();

      var frameReference = e.FrameReference;
      using (var frame = frameReference.AcquireFrame())
      {
        if (frame == null)
        {
          // We might miss the chance to acquire the frame,
          // it will be null if it's missed
          
          return;
        }

        if (_currentTrackedBody != null)
        {
          _currentTrackedBody =
            FindBodyWithTrackingId(frame, _currentTrackingId);

          if (_currentTrackedBody != null)
          {
            return;
          }
        }

        var selectedBody = FindClosestBody(frame);

        if (selectedBody == null)
        {
          return;
        }

        _currentTrackedBody = selectedBody;
        _currentTrackingId = selectedBody.TrackingId;

        _hdFaceFrameSource.TrackingId = _currentTrackingId;
      }
    }

    // This event is fired when a tracking is lost for a
    // body tracked by HDFace Tracker

    private void HdFaceSource_TrackingIdLost(
      object sender, TrackingIdLostEventArgs e
    )
    {
      var lostTrackingID = e.TrackingId;

      if (_currentTrackingId == lostTrackingID)
      {
        _currentTrackingId = 0;
        _currentTrackedBody = null;
        if (_faceModelBuilder != null)
        {
          _faceModelBuilder.Dispose();
          _faceModelBuilder = null;
        }

        _hdFaceFrameSource.TrackingId = 0;
      }
    }

    // This event is fired when a new HDFace frame is
    // ready for consumption

    private void HdFaceReader_FrameArrived(
      object sender, HighDefinitionFaceFrameArrivedEventArgs e
    )
    {
      using (var frame = e.FrameReference.AcquireFrame())
      {
        // We might miss the chance to acquire the frame;
        // it will be null if it's missed.
        // Also ignore this frame if face tracking failed.
        
        if (frame == null || !frame.IsFaceTracked)
        {
          return;
        }

        frame.GetAndRefreshFaceAlignmentResult(
          _currentFaceAlignment
        );

        UpdateMesh();
      }
    }

    // This event fires when the face capture operation is completed

    private void HdFaceBuilder_CollectionCompleted(
      object sender, FaceModelBuilderCollectionCompletedEventArgs e
    )
    {
      var modelData = e.ModelData;

      _currentFaceModel = modelData.ProduceFaceModel();

      _faceModelBuilder.Dispose();
      _faceModelBuilder = null;

      _currentBuilderStatus = "Capture Complete";
      _currentCollectionStatus = string.Empty;
    }

    // Check the face model builder status

    private void CheckOnBuilderStatus()
    {
      if (_faceModelBuilder == null)
      {
        return;
      }

      var captureStatus = _faceModelBuilder.CaptureStatus;
      _currentBuilderStatus = captureStatus.ToString();

      var collectionStatus = _faceModelBuilder.CollectionStatus;
      _currentCollectionStatus =
        GetCollectionStatusText(collectionStatus);
    }

    protected override SamplerStatus SamplerData()
    {
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

      const double spacing = 30;

      string line1 =
        string.Format(
          CultureInfo.CurrentCulture,
          "Status:   {0}",
          _currentBuilderStatus
        );
      string line2 =
        string.Format(
          CultureInfo.CurrentCulture,
          "         {0}",
          _currentCollectionStatus
        );
      string line3 =
        string.Format(
          CultureInfo.CurrentCulture,
          "Tracking: {0}",
          _currentTrackingId
        );

      wg.Text(
        new Point3d(0, spacing * 2, 0),  // Position
        new Vector3d(0, 0, 1),           // Normal
        new Vector3d(1, 0, 0),           // Direction
        line1,                           // Text
        true,                            // Rawness
        _style                           // TextStyle
      );

      wg.Text(
        new Point3d(0, spacing, 0),      // Position
        new Vector3d(0, 0, 1),           // Normal
        new Vector3d(1, 0, 0),           // Direction
        line2,                           // Text
        true,                            // Rawness
        _style                           // TextStyle
      );

      wg.Text(
        new Point3d(0, 0, 0),            // Position
        new Vector3d(0, 0, 1),           // Normal
        new Vector3d(1, 0, 0),           // Direction
        line3,                           // Text
        true,                            // Rawness
        _style                           // TextStyle
      );

      // Remember to pop our transforms off the stack

      wg.PopModelTransform();
      wg.PopModelTransform();

      // Draw the face

      if (_displayFaceMesh && _polygonPoints.Count > 0)
      {
        draw.Geometry.PolyPolygon(
          _numPolygonPositions,
          _polygonPositions,
          _numPolygonPoints,
          _polygonPoints,
          _outlineColors,
          _outlineTypes,
          _fillColors,
          _fillOpacities
        );
      }

      return true;
    }

    public Entity CreateMesh()
    {
      // Get the data from the current model

      var vertices =
        _currentFaceModel.CalculateVerticesForAlignment(
          _currentFaceAlignment
        );

      var indices = _currentFaceModel.TriangleIndices;

      // Copy the vertices to our point array

      var verts = new Point3dCollection();
      for (int i = 0; i < vertices.Count; ++i)
      {
        var vert = vertices[i];
        verts.Add(new Point3d(vert.X, vert.Y, -vert.Z));
      }

      // Get our faces in the right format

      var faces = new Int32Collection();
      for (int i = 0; i < indices.Count; i += 3)
      {
        faces.Add(3);
        faces.Add((int)indices[i]);
        faces.Add((int)(indices[i + 1]));
        faces.Add((int)(indices[i + 2]));
      }
 
      // Create and return the SubDMesh

      var sdm = new SubDMesh();
      sdm.SetSubDMesh(verts, faces, 0);

      return sdm;
    }
  }

  public class KinectFaceCommands
  {
    [CommandMethod("ADNPLUGINS", "KINFACE", CommandFlags.Modal)]
    public void ImportFaceFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      KinectFaceMeshJig kj = new KinectFaceMeshJig();

      kj.InitializeSpeech();

      if (!kj.StartSensor())
      {
        ed.WriteMessage(
          "\nUnable to start Kinect sensor - " +
          "are you sure it's plugged in?"
        );
        return;
      }

      var pr = ed.Drag(kj);

      if (pr.Status != PromptStatus.OK && !kj.Finished)
      {
        kj.StopSensor();
        return;
      }

      kj.StopSensor();

      var ent = kj.CreateMesh();
      if (ent != null)
      {
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

          btr.AppendEntity(ent);
          tr.AddNewlyCreatedDBObject(ent, true);
          
          tr.Commit();
        }
      }
    }
  }
}