using System;
using System.Timers;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;

namespace KinectSamples
{
  public class KinectSegmentedSolidsJig : KinectPointCloudJig
  {
    // Our transient solids (cursor sphere & tube) are yellow

    const short transSolColor = 2;

    // Our final solids will be green

    const short finalSolColor = 3;

    // A transaction and database to add solids

    private Transaction _tr;
    private Document _doc;

    // A list of vertices to draw between
    // (we use this for the final polyline creation)
    
    private Point3dCollection _vertices;
    private int _lastDrawnVertex;

    // Entities to create our solid

    private DBObjectCollection _created;

    // The radius of the profile circle to create

    private double _profRad;

    // The location at which to draw a sphere when resizing

    private Point3d _resizeLocation;

    // The approximate length of each swept segment
    // (as a multiple of the radius)

    private double _segFactor;

    // Flags to indicate Kinect gesture modes

    private bool _resizing;     // Drawing mode active
    private bool _drawing;      // Drawing mode active
    private bool _callibrating; // Initial few seconds to get in pos

    public bool Callibrating
    {
      get { return _callibrating; }
      set { _callibrating = value; }
    }

    public KinectSegmentedSolidsJig(
      Document doc, Transaction tr, double profRad, double factor
    )
    {
      // Initialise the various members

      _doc = doc;
      _tr = tr;
      _vertices = new Point3dCollection();
      _lastDrawnVertex = -1;
      _resizing = false;
      _drawing = false;
      _callibrating = true;
      _created = new DBObjectCollection();
      _profRad = profRad;
      _segFactor = factor;

      Words.Add("red");
      Words.Add("green");
      Words.Add("blue");
      Words.Add("yellow");
      Words.Add("pink");
      Words.Add("magenta");
      Words.Add("cyan");
    }

    protected override void OnFoundWord(WordFoundEventArgs e)
    {
      base.OnFoundWord(e);

      switch (e.Word)
      {
        case "RED":
          ColorIndex = 1;
          break;
        case "YELLOW":
          ColorIndex = 2;
          break;
        case "GREEN":
          ColorIndex = 3;
          break;
        case "CYAN":
          ColorIndex = 4;
          break;
        case "BLUE":
          ColorIndex = 5;
          break;
        case "PINK":
        case "MAGENTA":
          ColorIndex = 6;
          break;
        default:
          break;
      }
    }

    public void Cleanup()
    {
      _vertices.Clear();

      foreach (DBObject obj in _created)
      {
        obj.Dispose();        
      }
      _created.Clear();
    }

    private bool GenerateTube(
      double profRad, Point3dCollection pts, out Solid3d sol
    )
    {
      bool readyToBreak;

      // Let's start by creating our spline path

      using (var path = new Spline(pts, 0, 0.0))
      {
        double pathLen = path.GetDistanceAtParameter(path.EndParam);
        readyToBreak = (pathLen > _profRad * _segFactor);

        // And our sweep profile

        var profile = new Circle(pts[0], pts[1] - pts[0], profRad);
        using (profile)
        {
          // Then our sweep options

          var sob = new SweepOptionsBuilder();

          // Align the entity to sweep to the path

          sob.Align =
            SweepOptionsAlignOption.AlignSweepEntityToPath;

          // The base point is the start of the path

          sob.BasePoint = path.StartPoint;

          // The profile will rotate to follow the path

          sob.Bank = true;
          using (var sweepOpts = sob.ToSweepOptions())
          {
            sol = new Solid3d();
            sol.ColorIndex = ColorIndex;

            // Sweep our profile along our path

            sol.CreateSweptSolid(profile, path, sweepOpts);
          }
        }
      }
      _lastDrawnVertex = pts.Count - 1;

      return readyToBreak;
    }

    protected override SamplerStatus SamplerData()
    {
      if (!_callibrating && !Finished && _skeletons != null)
      {
        foreach (var data in _skeletons)
        {
          if (data.IsTracked)
          {
            var leftElbow =
              PointFromVector(
                data.Joints[JointType.ElbowLeft].Position, false
              );
            var leftHand =
              PointFromVector(
                data.Joints[JointType.HandLeft].Position, false
              );
            var rightHand =
              PointFromVector(
                data.Joints[JointType.HandRight].Position, false
              );
            var rightThumb =
              PointFromVector(
                data.Joints[JointType.ThumbRight].Position, false
              );

            if (
              leftHand.DistanceTo(Point3d.Origin) > 0 &&
              rightHand.DistanceTo(Point3d.Origin) > 0 &&
              leftHand.DistanceTo(rightHand) < 0.03
            )
            {
              // Hands are less than 3cm from each other

              _drawing = false;
              _resizing = false;
              Finished = true;
            }
            else
            {
              // Hands are within 10cm of each other vertically
              // and both hands are above the waist, so we resize
              // the profile radius

              _resizing =
                (leftHand.Z > leftElbow.Z &&
                  rightHand.Z > leftElbow.Z &&
                  Math.Abs(leftHand.Z - rightHand.Z) < 0.1
                );

              // If the left hand is below the waist, we draw

              _drawing = (leftHand.Z < leftElbow.Z);
            }

            if ((_resizing || _nearMode) && !_drawing)
            {
              // If resizing, set some data to help draw
              // a sphere where we're resizing

              var vec =
                (_nearMode ?
                  rightHand - rightThumb :
                  (leftHand - rightHand) / 2
                );
              _resizeLocation = rightHand + (_nearMode ? -vec : vec);
              _profRad = vec.Length;

              if (_nearMode)
              {
                _vertices.Add(_resizeLocation);
              }
            }

            if (_drawing)
            {
              // If we have at least one prior vertex...

              if (_vertices.Count > 0)
              {
                // ... check whether we're a certain distance
                // away from the last one before adding it (this
                // smooths off the jitters of adding every point)

                var lastVert = _vertices[_vertices.Count - 1];
                if (
                  lastVert.DistanceTo(rightHand) > _profRad * 3
                )
                {
                  // Add the new vertex to our list

                  _vertices.Add(rightHand);
                }
              }
              else
              {
                // Add the first vertex to our list

                _vertices.Add(rightHand);
              }
            }
            break;
          }
        }
      }
      
      // If not finished, but stopped drawing, add the
      // geometry that was previously drawn to the database

      if (
        !_drawing && (_created.Count > 0 || _vertices.Count > 0)
      )
      {
        AddSolidOrPath();
      }

      return base.SamplerData();
    }

    // Helper functions to extract/blank portions of our
    // vertex list (when we want to draw the beginning of it)

    private void ClearAllButLast(Point3dCollection pts, int n)
    {
      while (pts.Count > n)
      {
        pts.RemoveAt(0);
      }
      _lastDrawnVertex = -1;
    }

    private Point3dCollection GetAllButLast(
      Point3dCollection pts, int n
    )
    {
      var res = new Point3dCollection();
      for (int i = 0; i < pts.Count - n; i++)
      {
        res.Add(pts[i]);
      }
      return res;
    }

    protected override bool WorldDrawData(WorldDraw draw)
    {
      if (!base.WorldDrawData(draw))
        return false;

      short origCol = draw.SubEntityTraits.Color;

      if (
        (_resizing || _nearMode ) &&
        _resizeLocation.GetAsVector().Length > 0
      )
      {
        using (var sphere = new Solid3d())
        {
          try
          {
            sphere.CreateSphere(_profRad);

            if (sphere != null)
            {
              sphere.TransformBy(
                Matrix3d.Displacement(
                  _resizeLocation - Point3d.Origin
                )
              );

              // Draw the cursor

              draw.SubEntityTraits.Color = ColorIndex;
              sphere.WorldDraw(draw);
            }
          }
          catch (System.Exception ex)
          {
            _doc.Editor.WriteMessage(
              "\nException: {0} - {1}", ex.Message, ex.InnerException
            );

            ClearAllButLast(_vertices, 1);
          }
          finally
          {
            draw.SubEntityTraits.Color = origCol;
          }
        }
      }

      // If we're currently drawing...

      if (_drawing)
      {
        Solid3d sol = null;
        try
        {
          // If we have vertices that haven't yet been drawn...
 
          if (_vertices.Count > 1 //&&
              //_vertices.Count - 1 > _lastDrawnVertex
            )
          {
            // ... generate a tube

            if (GenerateTube(_profRad, _vertices, out sol))
            {
              // We now need to break the pipe...

              // If it was created, add it to our list to draw

              _created.Add(sol);
              sol = null;

              // Clear all but the last vertex to draw from
              // next time

              ClearAllButLast(_vertices, 1);
            }
          }
        }
        catch
        {
          // If the tube generation failed...

          if (sol != null)
          {
            sol.Dispose();
          }

          // Loop, creating the most recent successful tube we can

          bool succeeded = false;
          int n = 1;

          do
          {
            try
            {
              // Generate the previous, working tube using all
              // but the last points (if it fails, one more is
              // excluded per iteration, until we get a working
              // tube)

              GenerateTube(
                _profRad, GetAllButLast(_vertices, n++), out sol
              );

              _created.Add(sol);
              sol = null;
              succeeded = true;
            }
            catch { }
          }
          while (!succeeded && n < _vertices.Count);

          if (succeeded)
          {
            ClearAllButLast(_vertices, n - 1);

            if (_vertices.Count > 1)
            {
              try
              {
                // And generate a tube for the remaining vertices

                GenerateTube(_profRad, _vertices, out sol);
              }
              catch
              {
                succeeded = false;
              }
            }
          }
          
          if (!succeeded && sol != null)
          {
            ClearAllButLast(_vertices, 1);
            sol.Dispose();
            sol = null;
          }
        }

        // Draw our solid(s)

        draw.SubEntityTraits.Color = ColorIndex;

        foreach (DBObject obj in _created)
        {
          Entity ent = obj as Entity;
          if (ent != null)
          {
            try
            {
              ent.WorldDraw(draw);
            }
            catch
            {}
          }
        }

        if (sol != null)
        {
          try
          {
            sol.WorldDraw(draw);
          }
          catch
          { }
        }

        if (_vertices.Count > 0)
        {
          Point3d lastPt = _vertices[_vertices.Count - 1];
          
          // Create a cursor sphere

          using (Solid3d cursor = new Solid3d())
          {
            try
            {
              cursor.CreateSphere(_profRad);

              if (cursor != null)
              {
                cursor.TransformBy(
                  Matrix3d.Displacement(lastPt - Point3d.Origin)
                );

                // Draw the cursor

                draw.SubEntityTraits.Color = ColorIndex;

                cursor.WorldDraw(draw);
              }
            }
            catch { }
          }
        }

        if (sol != null)
        {
          sol.Dispose();
        }
      }

      draw.SubEntityTraits.Color = origCol;

      return true;
    }

    public void AddSolidOrPath()
    {
      Solid3d sol = null;
      try
      {
        GenerateTube(_profRad, _vertices, out sol);
      }
      catch
      {
        if (sol != null)
        {
          sol.Dispose();
          sol = null;
        }
      }

      if (_created.Count > 0 || sol != null)
      {
        if (sol != null)
        {
          _created.Add(sol);
        }

        var btr =
          (BlockTableRecord)_tr.GetObject(
            _doc.Database.CurrentSpaceId,
            OpenMode.ForWrite
          );

        foreach (DBObject obj in _created)
        {
          var ent = obj as Entity;
          if (ent != null)
          {
            //ent.ColorIndex = finalSolColor;

            btr.AppendEntity(ent);
            _tr.AddNewlyCreatedDBObject(ent, true);
          }
        }
        _created.Clear();
      }

      Cleanup();

      _vertices.Clear();
    }
  }

  public class KinectSegmentedSolidCommands
  {
    [CommandMethod("ADNPLUGINS", "KINEXT2", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      var doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      var ed = doc.Editor;

      using (var tr = doc.TransactionManager.StartTransaction())
      {
        // Pass in a default radius of 5cm and a segment length
        // of 10 times that

        var kj = new KinectSegmentedSolidsJig(doc, tr, 0.05, 10);

        // The Body data from KfW 2 gives fals positives, saying
        // the hands are close to one another. Set a timer to stop
        // the code from checking during an initial, 3 second
        // callibrating phase

        var tm = new Timer();
        tm.Interval = 3000;
        tm.Elapsed +=
          (s, e) =>
          {
             kj.Callibrating = false;
             tm.Stop();
          };
        tm.Start();

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
          kj.Cleanup();
          return;
        }

        // Generate a final point cloud with color before stopping
        // the sensor

        kj.UpdatePointCloud();
        kj.StopSensor();

        kj.AddSolidOrPath();
        tr.Commit();

        kj.WriteAndImportPointCloud(doc, kj.Vectors);
      }
    }
  }
}