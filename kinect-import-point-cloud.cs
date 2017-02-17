using System.Collections.Generic;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;

namespace KinectSamples
{
  public class KinectPointCloudJig : KinectJig
  {
    // A list of points captured by the sensor
    // (for eventual export)

    protected List<ColoredPoint3d> _vecs;

    public List<ColoredPoint3d> Vectors
    {
      get { return _vecs; }
    }

    // A list of points to be displayed
    // (we use this for the jig)

    protected Point3dCollection _points;

    public KinectPointCloudJig()
    {
      _points = new Point3dCollection();
    }

    public void UpdatePointCloud()
    {
      _vecs = GeneratePointCloud(1, true);
    }

    protected override SamplerStatus SamplerData()
    {
      // Generate a point cloud

      try
      {
        // Use a user-defined sampling the points for the jig

        var frame = _frameReader.AcquireLatestFrame();
        ProcessMultiSourceFrame(frame);

        _vecs = GeneratePointCloud(Sampling);

        // Extract the points for display in the jig

        _points.Clear();

        if (_vecs != null)
        {
          foreach (var vec in _vecs)
          {
            _points.Add(
              new Point3d(vec.X, vec.Y, vec.Z)
            );
          }
        }
      }
      catch (System.Exception ex)
      {
        Application.DocumentManager.MdiActiveDocument.Editor.WriteMessage(
          "\nException: {0}", ex.Message
        );
      }

      ForceMessage();

      return SamplerStatus.OK;
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      // This simply draws our points

      if (_points != null && _points.Count > 0)
        draw.Geometry.Polypoint(_points, null, null);

      return true;
    }
  }

  public class KinectPointCloudCommands
  {
    [CommandMethod("ADNPLUGINS", "KINECT", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      var doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      var ed = doc.Editor;

      var kj = new KinectPointCloudJig();

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

      // Generate a final point cloud with color before stopping
      // the sensor

      kj.UpdatePointCloud();
      kj.StopSensor();

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }
  }
}