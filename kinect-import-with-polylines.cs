using System.Collections.Generic;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;

namespace KinectSamples
{
  public class KinectPolyJig : KinectPointCloudJig
  {
    // A transaction and database to add polylines

    private Transaction _tr;
    private Document _doc;

    // A list of vertices to draw between
    // (we use this for the final polyline creation)

    private Point3dCollection _vertices;

    // The most recent vertex being captured/drawn

    private Point3d _curPt;
    private Entity _cursor;

    // A list of line segments being collected
    // (pass these as AcGe objects as they may
    // get created on a background thread)

    private List<LineSegment3d> _lineSegs;

    // The database lines we use for temporary
    // graphics (that need disposing afterwards)

    private DBObjectCollection _lines;

    // Flags to indicate Kinect gesture modes

    private bool _drawing;     // Drawing mode active

    public KinectPolyJig(Document doc, Transaction tr)
    {
      // Initialise the various members

      _doc = doc;
      _tr = tr;
      _vertices = new Point3dCollection();
      _lineSegs = new List<LineSegment3d>();
      _lines = new DBObjectCollection();
      _cursor = null;
      _drawing = false;
    }

    protected override SamplerStatus SamplerData()
    {
      if (!Finished && _skeletons != null)
      {
        foreach (var data in _skeletons)
        {
          if (data.IsTracked)
          {
            var leftHip =
              PointFromVector(
                data.Joints[JointType.HipLeft].Position, false
              );
            var leftHand =
              PointFromVector(
                data.Joints[JointType.HandLeft].Position, false
              );
            var rightHand =
              PointFromVector(
                data.Joints[JointType.HandRight].Position, false
              );

            _drawing = (leftHand.Z < leftHip.Z);

            if (
              leftHand.DistanceTo(Point3d.Origin) > 0 &&
              rightHand.DistanceTo(Point3d.Origin) > 0 &&
              leftHand.DistanceTo(rightHand) < 0.1)
            {
              _drawing = false;
              Finished = true;
            }

            if (_drawing)
            {
              // If we have at least one prior vertex...

              if (_vertices.Count > 0)
              {
                // ... connect them together with
                // a temp LineSegment3d

                var lastVert = _vertices[_vertices.Count - 1];
                if (lastVert.DistanceTo(rightHand) >
                    Tolerance.Global.EqualPoint)
                {
                  _lineSegs.Add(
                    new LineSegment3d(lastVert, rightHand)
                  );
                }
              }

              // Add the new vertex to our list

              _vertices.Add(rightHand);
            }
            break;
          }
        }
      }

      if (!_drawing && _lines.Count > 0)
      {
        AddPolylines();
      }

      // Generate a point cloud

      var res = SamplerStatus.Cancel;

      try
      {
        res = base.SamplerData();
      }
      catch { }

      return res;
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      if (!base.WorldDrawData(draw))
        return false;

      var ctm = TransientManager.CurrentTransientManager;
      var ints = new IntegerCollection();

      while (_lineSegs.Count > 0)
      {
        // Get the line segment and remove it from the list

        var ls = _lineSegs[0];
        _lineSegs.RemoveAt(0);

        // Create an equivalent, red, database line
        // (or yellow, if calibrating)

        var ln = new Line(ls.StartPoint, ls.EndPoint);
        ln.ColorIndex = 1;
        _lines.Add(ln);

        // Draw it as transient graphics

        ctm.AddTransient(
          ln, TransientDrawingMode.DirectShortTerm,
          128, ints
        );
      }

      if (_drawing)
      {
        if (_cursor == null)
        {
          if (_vertices.Count > 0)
          {
            // Clear our skeleton

            ClearTransients();

            _curPt = _vertices[_vertices.Count - 1];

            // Make our sphere 10cm in diameter (5cm radius)

            var sol = new Solid3d();
            sol.CreateSphere(0.05);
            _cursor = sol;
            _cursor.TransformBy(
              Matrix3d.Displacement(_curPt - Point3d.Origin)
            );

            _cursor.ColorIndex = 2;

            ctm.AddTransient(
              _cursor, TransientDrawingMode.DirectShortTerm,
              128, ints
            );
          }
        }
        else
        {
          if (_vertices.Count > 0)
          {
            var newPt = _vertices[_vertices.Count - 1];
            _cursor.TransformBy(
              Matrix3d.Displacement(newPt - _curPt)
            );
            _curPt = newPt;

            ctm.UpdateTransient(_cursor, ints);
          }
        }
      }
      else // !_drawing
      {
        if (_cursor != null)
        {
          ctm.EraseTransient(_cursor, ints);
          _cursor.Dispose();
          _cursor = null;
        }
      }

      return true;
    }

    public void AddPolylines()
    {
      ClearTransients();

      // Dispose of the database objects

      foreach (DBObject obj in _lines)
      {
        obj.Dispose();
      }
      _lines.Clear();

      // Create a true database-resident 3D polyline
      // (and let it be green)

      if (_vertices.Count > 1)
      {
        var btr =
          (BlockTableRecord)_tr.GetObject(
            _doc.Database.CurrentSpaceId,
            OpenMode.ForWrite
          );

        var pl =
          new Polyline3d(
            Poly3dType.SimplePoly, _vertices, false
          );
        pl.ColorIndex = 3;

        btr.AppendEntity(pl);
        _tr.AddNewlyCreatedDBObject(pl, true);
      }
      _vertices.Clear();
    }

    public void ClearTransients()
    {
      var ctm = TransientManager.CurrentTransientManager;

      // Erase the various transient graphics

      ctm.EraseTransients(
        TransientDrawingMode.DirectShortTerm, 128,
        new IntegerCollection()
      );
    }
  }

  public class KinectPolyCommands
  {
    [CommandMethod("ADNPLUGINS", "KINPOLY", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      var doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      var ed = doc.Editor;

      using (var tr = doc.TransactionManager.StartTransaction())
      {
        var kj = new KinectPolyJig(doc, tr);

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
          kj.ClearTransients();
          kj.StopSensor();
          return;
        }

        // Generate a final point cloud with color before stopping
        // the sensor

        kj.UpdatePointCloud();
        kj.StopSensor();

        kj.AddPolylines();
        tr.Commit();

        kj.WriteAndImportPointCloud(doc, kj.Vectors);
      }
    }
  }
}