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
  public class KinectCombinedJig : KinectPointCloudJig
  {
    // A list of lines representing our skeleton(s)

    private List<Line> _lines;

    public List<Line> Lines
    {
      get { return _lines; }
    }

    // Flags to make sure we don't end up both modifying
    // and accessing the _lines member at the same time

    private bool _drawing = false;
    private bool _capturing = false;
    private bool _done = false;

    public bool Done
    {
      get { return _done; }
      set { _done = value; }
    }
    
    public KinectCombinedJig()
    {
      // Initialise the various members

      _done = false;
      _drawing = false;
      _capturing = false;

      _lines = new List<Line>();

      try
      {
        _nearMode =
          (short)Application.GetSystemVariable("KINNEAR") == 1;
      }
      catch
      {
        _nearMode = false;
      }
    }

    private void AddLinesForSkeleton(
      List<Line> lines, Microsoft.Kinect.Body sk, int idx
    )
    {
      // Hard-code lists of connections between joints

      var links =
        new int[][]
        {
          // Head to left toe
          new int[] { 3, 2, 20, 1, 0, 12, 13, 14, 15 },
          // Hips to right toe
          new int[] { 0, 16, 17, 18, 19 },
          // Left hand to right hand
          new int[] { 21, 7, 6, 5, 4, 2, 8, 9, 10, 11, 23 },
          // Left thumb to palm
          new int[] { 22, 7 },
          // Right thumb to palm
          new int[] { 24, 11 }
        };

      // Populate an array of joints

      var joints = new Point3dCollection();
      for (int i = 0; i < sk.Joints.Count; i++)
      {
        joints.Add(
          PointFromVector(
            sk.Joints[(JointType)i].Position, false
          )
        );
      }

      // For each path of joints, create a sequence of lines

      int limit = sk.Joints.Count - 1;

      foreach (int[] link in links)
      {
        for (int i = 0; i < link.Length - 1; i++)
        {
          // Only add lines where links are within bounds
          // (check needed for seated mode)

          int first = link[i],
              second = link[i + 1];

          if (
            isValidJoint(first, limit) &&
            isValidJoint(second, limit)
          )
          {
            // Line from this vertex to the next

            var ln = new Line(joints[first], joints[second]);

            // Set the color to distinguish between skeletons

            ln.ColorIndex = idx;

            // Make tracked skeletons bolder

            ln.LineWeight =
              (sk.IsTracked?
                LineWeight.LineWeight050 :
                LineWeight.LineWeight000
              );

            lines.Add(ln);
          }
        }
      }
    }

    private bool isValidJoint(int id, int upperLimit)
    {
      // A joint is valid if it indexes into the collection
      // and if it's above the spine in near mode

      return id <= upperLimit && (!_nearMode || id > 1);
    }

    private void ClearLines()
    {
      // Dispose each of the lines and clear the list

      foreach (Line ln in _lines)
      {
        ln.Dispose();
      }
      _lines.Clear();
    }

    protected override SamplerStatus SamplerData()
    {
      var res = base.SamplerData();

      if (res != SamplerStatus.OK)
        return res;

      if (!_drawing && !_done)
      {
        _capturing = true;

        // Clear any previous lines

        ClearLines();

        // We'll colour the skeletons from yellow, onwards
        // (red is a bit dark)

        short col = 2;

        // Loop through each of the skeletons

        if (_skeletons != null)
        {
          foreach (var skel in _skeletons)
          {
            // Add skeleton vectors for tracked/positioned
            // skeletons

            AddLinesForSkeleton(_lines, skel, col++);
          }
        }
        _capturing = false;
      }

      return SamplerStatus.OK;
    }
    
    protected override bool WorldDrawData(WorldDraw draw)
    {
      if (!_capturing)
      {
        _drawing = true;

        // Draw each of our lines

        short oidx = draw.SubEntityTraits.Color;

        foreach (var ln in _lines)
        {
          // Set the colour and lineweight in the subentity
          // traits based on the original line

          if (ln != null)
          {
            draw.SubEntityTraits.Color = (short)ln.ColorIndex;
            draw.SubEntityTraits.LineWeight = ln.LineWeight;

            ln.WorldDraw(draw);
          }
        }

        draw.SubEntityTraits.Color = oidx;

        _drawing = false;
      }

      return base.WorldDrawData(draw);
    }
  }

  public class KinectCombinedCommands
  {
    [CommandMethod("ADNPLUGINS", "KINBOTH", CommandFlags.Modal)]
    public void ImportFromKinect()
    {
      Document doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      Editor ed = doc.Editor;

      KinectCombinedJig kj = new KinectCombinedJig();

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

      if (pr.Status != PromptStatus.OK)
      {
        kj.StopSensor();
        return;
      }

      kj.Done = true;

      // Generate a final point cloud with color before stopping
      // the sensor

      kj.UpdatePointCloud();
      kj.StopSensor();

      AddLines(doc, kj.Lines);

      kj.WriteAndImportPointCloud(doc, kj.Vectors);
    }

    public void AddLines(Document doc, List<Line> lines)
    {
      if (lines.Count > 0)
      {
        var tr =
          doc.TransactionManager.StartTransaction();
        using (tr)
        {
          var btr =
            (BlockTableRecord)tr.GetObject(
              doc.Database.CurrentSpaceId,
              OpenMode.ForWrite
            );

          foreach (Line ln in lines)
          {
            ln.ColorIndex = 1;
            btr.AppendEntity(ln);
            tr.AddNewlyCreatedDBObject(ln, true);
          }
          tr.Commit();
        }
      }
    }
  }
}