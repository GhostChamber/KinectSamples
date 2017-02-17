using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.GraphicsInterface;
using Autodesk.AutoCAD.Runtime;
using Microsoft.Kinect;
using Microsoft.Speech.Recognition;

namespace KinectSamples
{
  public class ColoredPoint3d
  {
    public ColoredPoint3d()
    {
    }

    public ColoredPoint3d(
      double x, double y, double z
    )
    {
      X = x; Y = y; Z = z;
    }

    public ColoredPoint3d(
      double x, double y, double z, int r, int g, int b
    )
    {
      X = x; Y = y; Z = z; R = r; G = g; B = b;
    }
    public double X, Y, Z;
    public int R, G, B;
  }

  public class WordFoundEventArgs : EventArgs
  {
    string _word = "";

    // Constructor

    public WordFoundEventArgs(string word)
    {
      _word = word;
    }

    // Property
    
    public string Word
    {
      get { return _word; }
    }
  }
  
  public abstract class KinectJig : DrawJig
  {
    // To stop the running jig by sending a cancel request

    [DllImport("accore.dll", CharSet = CharSet.Auto,
      CallingConvention = CallingConvention.Cdecl,
      EntryPoint = "?acedPostCommand@@YAHPEB_W@Z"
     )]
    extern static private int acedPostCommand(string strExpr);

    // We need our Kinect sensor

    protected KinectSensor _kinect = null;

    // We need a reader for depth and colour frames

    protected MultiSourceFrameReader _frameReader = null;

    // And audio array

    private AudioSource _audio = null;

    // Microsoft Speech recognition engine

    private SpeechRecognitionEngine _sre;

    // A word has been recognised

    private string _word = "";

    // With the data collected by the sensor

    protected ushort[] _depthPixels = null;
    protected int _depthWidth;
    protected byte[] _colorPixels = null;
    protected int _colorWidth;
    protected IList<Microsoft.Kinect.Body> _skeletons = null;

    // Flag and property for when we want to exit

    protected bool _finished;

    public bool Finished
    {
      get { return _finished; }
      set { _finished = value; }
    }

    // Selected color, for whatever usage makes sense
    // in child classes

    protected short _colorIndex;

    internal short ColorIndex
    {
      get { return _colorIndex; }
      set { _colorIndex = value; }
    }

    // Sampling value to reduce point set on capture

    private short _sampling;

    internal short Sampling
    {
      get { return _sampling; }
      set { _sampling = value; }
    }

    // Should speeh input be enabled

    private bool _speech;

    internal bool Speech
    {
      get { return _speech; }
      set { _speech = value; }
    }
    
    // Should near mode and seated skeleton tracking be enabled

    protected bool _nearMode;

    internal bool NearMode
    {
      get { return _nearMode; }
      set { _nearMode = value; }
    }

    // Audio support requires words to check

    protected List<string> Words = new List<string>();

    public event EventHandler<WordFoundEventArgs> FoundWord;

    protected virtual void OnFoundWord(WordFoundEventArgs e)
    {
      if (FoundWord != null)
      {
        FoundWord(this, e);
      }

      _word = e.Word;

      switch (_word)
      {
        case "CANCEL":
          CancelJig();
          break;
        case "FINISH":
          Finished = true;
          break;
        default:
          break;
      }
    }

    // Extents to filter points

    private static Extents3d? _ext = null;

    public static Extents3d? Extents
    {
      get { return _ext; }
      set { _ext = value; }
    }

    public KinectJig()
    {
      // Initialise the various members

      _colorIndex = 3;
      _finished = false;

      try
      {
        _sampling = (short)Application.GetSystemVariable("KINSAMP");
      }
      catch
      {
        _sampling = 50;
      }

      try
      {
        _speech =
          (short)Application.GetSystemVariable("KINSPEECH") == 1;
      }
      catch
      {
        _speech = false;
      }

      try
      {
        _nearMode =
          (short)Application.GetSystemVariable("KINNEAR") == 1;
      }
      catch
      {
        _nearMode = false;
      }

      _kinect = KinectSensor.GetDefault();

      Words.Add("finish");
      Words.Add("cancel");

      // Set values based on near-mode

      var nearMode = NearMode;
    }

    internal void ProcessMultiSourceFrame(MultiSourceFrame frame)
    {
      if (frame == null)
        return;

      if (frame.ColorFrameReference != null)
      {
        using (
          var colorFrame = frame.ColorFrameReference.AcquireFrame()
        )
        {
          if (colorFrame != null)
          {
            if (_colorPixels == null ||
                _colorPixels.Length !=
                  colorFrame.FrameDescription.LengthInPixels * 4
            )
            {
              _colorPixels =
                new byte[
                  colorFrame.FrameDescription.LengthInPixels * 4
                ];
              _colorWidth = colorFrame.FrameDescription.Width;
            }

            colorFrame.CopyConvertedFrameDataToArray(
              _colorPixels, ColorImageFormat.Bgra
            );
          }
        }
      }

      if (frame.DepthFrameReference != null)
      {
        using (
          var depthFrame = frame.DepthFrameReference.AcquireFrame()
        )
        {
          if (depthFrame != null)
          {
            if (_depthPixels == null ||
                _depthPixels.Length !=
                  depthFrame.FrameDescription.LengthInPixels
            )
            {
              _depthPixels =
                new ushort[
                  depthFrame.FrameDescription.LengthInPixels
                ];
              _depthWidth = depthFrame.FrameDescription.Width;
            }

            depthFrame.CopyFrameDataToArray(_depthPixels);
          }
        }
      }

      if (frame.BodyFrameReference != null)
      {
        using (
          var bodyFrame = frame.BodyFrameReference.AcquireFrame()
        )
        {
          if (bodyFrame != null)
          {
            if (_skeletons == null)
            {
              _skeletons =
                new Microsoft.Kinect.Body[
                  _kinect.BodyFrameSource.BodyCount
                ];
            }
            bodyFrame.GetAndRefreshBodyData(_skeletons);
          }
        }
      }
    }

    void OnSpeechHypothesized(
      object sender, SpeechHypothesizedEventArgs e
    )
    {
    }

    void OnSpeechRecognized(
      object sender, SpeechRecognizedEventArgs e
    )
    {
      // Ignore if we don't have a high degree of confidence

      if (e.Result.Confidence < 0.7)
        return;

      OnFoundWord(
        new WordFoundEventArgs(e.Result.Text.ToUpperInvariant())
      );
    }

    public virtual bool StartSensor()
    {
      if (_kinect != null)
      {
        _kinect.Open();

        _frameReader =
          _kinect.OpenMultiSourceFrameReader(
            FrameSourceTypes.Color |
            FrameSourceTypes.Depth |
            FrameSourceTypes.Body
          );

        // We need speech recognition started on a separate,
        // MTA thread

        if (Speech)
        {
          var t = new Thread(StartSpeech);
          t.Start();
        }

        return true;
      }
      return false;
    }

    private static RecognizerInfo GetKinectRecognizer()
    {
      Func<RecognizerInfo, bool> matchingFunc =
        r =>
        {
          string value;
          r.AdditionalInfo.TryGetValue("Kinect", out value);
          return
            "True".Equals(
              value,
              StringComparison.InvariantCultureIgnoreCase
            ) &&
            "en-US".Equals(
              r.Culture.Name,
              StringComparison.InvariantCultureIgnoreCase
            );
        };
      return
        SpeechRecognitionEngine.InstalledRecognizers().Where(
          matchingFunc
        ).FirstOrDefault();
    }

    public void InitializeSpeech()
    {
      var ed =
        Application.DocumentManager.MdiActiveDocument.Editor;

      if (!Speech || Words.Count == 0)
        return;

      // Create and setup our audio source

      _audio = _kinect.AudioSource;
      /*
      _audio.EchoCancellationMode =
        EchoCancellationMode.None;
      _audio.AutomaticGainControlEnabled = false;
      _audio.BeamAngleMode = BeamAngleMode.Automatic;
       */

      var ri = GetKinectRecognizer();
      if (ri == null)
      {
        ed.WriteMessage(
          "There was a problem initializing Speech Recognition. " +
          "Ensure you have the Microsoft Speech SDK installed " +
          "and configured."
        );
      }

      // Need to wait 4 seconds for device to be ready right after
      // initialization

      ed.WriteMessage("\n");

      int wait = 4;
      while (wait > 0)
      {
        ed.WriteMessage(
          "Kinect will be ready for speech recognition in {0} " +
          "second{1}.\n",
          wait--,
          wait == 0 ? "" : "s"
        );
        System.Windows.Forms.Application.DoEvents();
        Thread.Sleep(1000);
      }
      ed.WriteMessage("Kinect ready for speech recognition.\n");

      System.Windows.Forms.Application.DoEvents();

      try
      {
        _sre = new SpeechRecognitionEngine(ri.Id);
      }
      catch
      {
        ed.WriteMessage(
          "There was a problem initializing Speech Recognition. " +
          "Ensure you have the Microsoft Speech SDK installed " +
          "and configured."
        );
      }

      // Populate our word choices

      var words = new Choices();
      foreach (string word in Words)
      {
        words.Add(word);
      }

      // Create a GrammarBuilder from them

      var gb = new GrammarBuilder();
      gb.Culture = ri.Culture;
      gb.Append(words);

      // Create the actual Grammar instance, and then load it
      // into the speech recognizer

      var g = new Grammar(gb);
      _sre.LoadGrammar(g);

      // Attach our event handler for recognized commands
      // We won't worry about rejected or hypothesized callbacks

      _sre.SpeechHypothesized += OnSpeechHypothesized;
      _sre.SpeechRecognized += OnSpeechRecognized;
    }

    [MTAThread]
    private void StartSpeech()
    {
      if (_sre != null)
      {
        try
        {
          // Get the audio stream and pass it to the
          // speech recognition engine

          /*
          Stream kinectStream = _audio.Start();

          _sre.SetInputToAudioStream(
            kinectStream,
            new SpeechAudioFormatInfo(
              EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null
            )
          );
          _sre.RecognizeAsync(RecognizeMode.Multiple);
           */
        }
        catch
        {
          var ed =
            Application.DocumentManager.MdiActiveDocument.Editor;
          ed.WriteMessage(
            "There was a problem initializing the KinectAudioSource." +
            " Ensure you have the Kinect SDK installed correctly."
          );
        }
      }
    }

    public virtual void PauseSensor()
    {
      if (_kinect != null)
      {
        _finished = true;
      }
    }

    public virtual void StopSensor()
    {
      if (_kinect != null)
      {
        if (_audio != null)
        {
          _sre.RecognizeAsyncStop();
          //_audio.Stop();
          _audio = null;
        }

        _kinect.Close();
        _kinect = null;
      }
      _finished = true;
    }

    protected virtual SamplerStatus SamplerData()
    {
      return SamplerStatus.Cancel;
    }

    protected virtual bool WorldDrawData(WorldDraw draw)
    {
      return false;
    }

    protected override SamplerStatus Sampler(JigPrompts prompts)
    {
      if (!String.IsNullOrEmpty(_word))
      {
        var doc =
          Application.DocumentManager.MdiActiveDocument;
        if (doc != null)
        {
          doc.Editor.WriteMessage("\nWord recognised: {0}", _word);
          _word = "";
        }
      }

      // We don't really need a point, but we do need some
      // user input event to allow us to loop, processing
      // for the Kinect input

      var opts = new JigPromptPointOptions("\nClick to capture: ");
      opts.UserInputControls =
        UserInputControls.NullResponseAccepted;

      opts.Cursor = CursorType.Invisible;

      var ppr = prompts.AcquirePoint(opts);
      if (ppr.Status == PromptStatus.OK)
      {
        if (_finished)
        {
          CancelJig();
          return SamplerStatus.Cancel;
        }

        return SamplerData();
      }
      return SamplerStatus.Cancel;
    }

    protected override bool WorldDraw(WorldDraw draw)
    {
      return WorldDrawData(draw);
    }

    public void ForceMessage()
    {
      // Set the cursor without ectually moving it - enough to
      // generate a Windows message

      var pt = System.Windows.Forms.Cursor.Position;
      System.Windows.Forms.Cursor.Position =
        new System.Drawing.Point(pt.X, pt.Y);
    }

    public List<ColoredPoint3d> GeneratePointCloud(
      int sampling, bool useColor = false
    )
    {      
      return GeneratePointCloud(
        _kinect, _depthPixels, _colorPixels, sampling, useColor
      );
    }

    // Generate a point cloud from depth and RGB data

    internal List<ColoredPoint3d> GeneratePointCloud(
      KinectSensor kinect, ushort[] depth, byte[] color,
      int sampling, bool withColor = false
    )
    {
      if (depth == null || color == null)
        return null;

      // We will return a list of our ColoredPoint3d objects

      var res = new List<ColoredPoint3d>();

      // We now need a CoordinateMapper to map the points

      var cm = _kinect.CoordinateMapper;

      // Loop through the depth information - we process two
      // bytes at a time

      for (int i = 0; i < depth.Length; i += sampling)
      {
        // The x and y positions can be calculated using modulus
        // division from the array index

        var pt = new DepthSpacePoint();
        pt.X = i % _depthWidth;
        pt.Y = i / _depthWidth;

        var p = cm.MapDepthPointToCameraSpace(pt, depth[i]);

        // A zero value for Z means there is no usable depth for
        // that pixel

        if (p.Z > 0)
        {
          // Create a ColoredPoint3d to store our XYZ and RGB info
          // for a pixel

          var cv = new ColoredPoint3d();
          cv.X = p.X;
          cv.Y = p.Z;
          cv.Z = p.Y;

          // Only calculate the colour when it's needed

          if (withColor)
          {
            // Get the colour indices for that particular depth
            // pixel

            var cip = cm.MapDepthPointToColorSpace(pt, depth[i]); 

            // Extract the RGB data from the appropriate place
            // in the colour data

            long colIndex =
              (long)(4 * (Math.Floor(Math.Abs(cip.X)) +
                         (Math.Floor(Math.Abs(cip.Y)) * _colorWidth))
                    );
            bool inside = colIndex < color.LongLength;
            cv.B = (byte)(inside ? color[colIndex] : 0);
            cv.G = (byte)(inside ? color[colIndex + 1] : 0);
            cv.R = (byte)(inside ? color[colIndex + 2] : 0);
          }
          else
          {
            // If we don't need colour information, just set each
            // pixel to white

            cv.B = 255;
            cv.G = 255;
            cv.R = 255;
          }

          // Add our pixel data to the list to return

          res.Add(cv);
        }
      }

      // Apply a bounding box filter, if one is defined

      if (_ext.HasValue)
      {
        // Use LINQ to get the points within the
        // bounding box

        var vecSet =
          from ColoredPoint3d vec in res
          where
            vec.X > _ext.Value.MinPoint.X &&
            vec.X < _ext.Value.MaxPoint.X &&
            vec.Y > _ext.Value.MinPoint.Y &&
            vec.Y < _ext.Value.MaxPoint.Y &&
            vec.Z > _ext.Value.MinPoint.Z &&
            vec.Z < _ext.Value.MaxPoint.Z
          select vec;

        // Convert our IEnumerable<> into a List<>

        res = vecSet.ToList<ColoredPoint3d>();
      }

      return res;
    }

    // Save the provided point cloud to a specific file

    protected virtual void ExportPointCloud(
      List<ColoredPoint3d> vecs, string filename
    )
    {
      const double scale = 1.0 / 39.3701;

      if (vecs.Count > 0)
      {
        using (var sw = new StreamWriter(filename))
        {
          // For each pixel, write a line to the text file:
          // X, Y, Z, R, G, B

          foreach (var pt in vecs)
          {
            sw.WriteLine(
              "{0}, {1}, {2}, {3}, {4}, {5}",
              pt.X * scale, pt.Y * scale, pt.Z * scale,
              pt.R, pt.G, pt.B
            );
          }
        }
      }
    }

    // Translate from Skeleton Space to WCS

    internal static Point3d PointFromVector(
      CameraSpacePoint p, bool flip = true
    )
    {
      // Rather than just return a point, we're effectively
      // transforming it to the drawing space: flipping the
      // Y and Z axes (which makes it consistent with the
      // point cloud, and makes sure Z is actually up - from
      // the Kinect's perspective Y is up), and reversing
      // the X axis (which is the result of choosing UseDepth
      // rather than UseDepthAndPlayerIndex)

      return new Point3d(flip ? -p.X : p.X, p.Z, p.Y);
    }

    // Cancel the running jig

    internal static void CancelJig()
    {
      acedPostCommand(""); //CANCELCMD
    }

    // Write the provided point cloud to file, then chain
    // the commands needed to import it into AutoCAD

    public void WriteAndImportPointCloud(
      Document doc, List<ColoredPoint3d> vecs
    )
    {
      var ed = doc.Editor;

      if (vecs == null || vecs.Count == 0)
      {
        ed.WriteMessage("\nNo points were found.");
        return;
      }

      // We'll store most local files in the temp folder.
      // We get a temp filename, delete the file and
      // use the name for our folder

      /**/
      string localPath = Path.GetTempFileName();
      File.Delete(localPath);
      Directory.CreateDirectory(localPath);
      localPath += "\\";

      // Paths for our temporary files

      string txtPath = localPath + "points.xyz";
      /**/

      // Our PCG file will be stored under My Documents

      string outputPath =
        Environment.GetFolderPath(
          Environment.SpecialFolder.MyDocuments
        ) + "\\Kinect Point Clouds\\";

      if (!Directory.Exists(outputPath))
        Directory.CreateDirectory(outputPath);

      // We'll use the title as a base filename for the PCG,
      // but will use an incremented integer to get an unused
      // filename

      int cnt = 0;
      string xyzPath;
      do
      {
        xyzPath =
          outputPath + "Kinect" +
          (cnt == 0 ? "" : cnt.ToString()) + ".xyz";
        cnt++;
      }
      while (File.Exists(xyzPath));
      
      cnt = 0;
      string pcgPath;
      do
      {
        pcgPath =
          outputPath + "Kinect" +
          (cnt == 0 ? "" : cnt.ToString()) + ".pcg";
        cnt++;
      }
      while (File.Exists(pcgPath));

      // Export our point cloud from the jig

      ed.WriteMessage(
        "\nSaving TXT file of the captured points.\n"
      );

      ExportPointCloud(vecs, xyzPath);

      ed.WriteMessage("\nPoints saved to \"{0}\".", xyzPath);


      // The path for the AdPointCloudIndexer tool is the same as
      // for AutoCAD

      string exePath =
        Path.GetDirectoryName(
          System.Windows.Forms.Application.ExecutablePath
        ) + "\\";

      if (!File.Exists(exePath + "AdPointCloudIndexer.exe"))
      {
        ed.WriteMessage(
          "\nCould not find the AdPointCloudIndexer tool."
        );
        return;
      }

      // Use the AdPointCloudIndexer tool to create a .PCG from
      // our .XYZ file

      ed.WriteMessage(
        "\nIndexing the saved points.\n"
      );

      var psi =
        new ProcessStartInfo(
          exePath + "AdPointCloudIndexer",
          "-i \"" + xyzPath + "\" " +
          "-o \"" + pcgPath + "\" -RGB"
        );
      psi.CreateNoWindow = false;
      psi.WindowStyle = ProcessWindowStyle.Hidden;

      // Wait up to 20 seconds for the process to exit

      try
      {
        using (Process p = Process.Start(psi))
        {
          System.Windows.Forms.Application.DoEvents();
          p.WaitForExit();
        }
      }
      catch
      { }

      // If there's a problem, we return

      if (!File.Exists(pcgPath))
      {
        ed.WriteMessage("\nError indexing points.");
        return;
      }

      KinectCommands.CleanupTmpFiles(txtPath);

      string pcgLisp = pcgPath.Replace('\\', '/');

      // Attach the .PCG file

      doc.SendStringToExecute(
        "_.-VISUALSTYLES _C _Conceptual " +
        "_.UCSICON _OF " +
        "(command \"_.-POINTCLOUDATTACH\" \"" +
        pcgLisp + "\" \"0,0,0\" \"1\" \"0\")(princ) ",
        false, false, false
      );
      /**/
    }
  }

  public class KinectCommands
  {
    // Set the clipping volume for the current point cloud

    [CommandMethod("ADNPLUGINS", "KINBOUNDS2", CommandFlags.Modal)]
    public void SetBoundingBox()
    {
      var doc =
        Autodesk.AutoCAD.ApplicationServices.
          Application.DocumentManager.MdiActiveDocument;
      var ed = doc.Editor;

      // Ask the user to select an entity

      var peo =
        new PromptEntityOptions(
          "\nSelect entity to define bounding box"
        );
      peo.AllowNone = true;
      peo.Keywords.Add("None");
      peo.Keywords.Default = "None";

      var per = ed.GetEntity(peo);

      if (per.Status != PromptStatus.OK)
        return;

      // If "None" selected, clear the bounding box

      if (per.Status == PromptStatus.None ||
          per.StringResult == "None")
      {
        KinectJig.Extents = null;
        ed.WriteMessage("\nBounding box cleared.");
        return;
      }

      // Otherwise open the entity and gets its extents

      using (var tr = doc.TransactionManager.StartTransaction())
      {
        var ent =
          tr.GetObject(per.ObjectId, OpenMode.ForRead)
            as Entity;
        if (ent != null)
          KinectJig.Extents = ent.Bounds;

        ed.WriteMessage(
          "\nBounding box set to {0}", KinectJig.Extents
        );
        tr.Commit();
      }
    }

    // Remove any temporary files from the point cloud import

    internal static void CleanupTmpFiles(string txtPath)
    {
      if (File.Exists(txtPath))
        File.Delete(txtPath);
      Directory.Delete(
        Path.GetDirectoryName(txtPath)
      );
    }
  }
}
