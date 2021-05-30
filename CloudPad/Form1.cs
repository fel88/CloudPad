using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
//using MathNet.Numerics.LinearAlgebra;
//using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;
using System.Globalization;

namespace _3d
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            //Helpers.Add(new SphereDrawer() { Radius = 10, Center = new Vector3d() });
            UpdateHelpersList();
            /*MathNet.Numerics.Control.UseNativeMKL();

            Matrix<double> A = DenseMatrix.OfArray(new double[,] {
          {1,1,1,1},
          {1,2,3,4},
          {4,3,2,1}}); Matrix<double> AA = DenseMatrix.OfArray(new double[,] {
          {1,1,1,1},
          {1,2,3,4},
          {1,2,3,4},
          {4,3,2,1}
          });
            Vector<double>[] nullspace = A.Kernel();

            // verify: the following should be approximately (0,0,0)
            var a = (A * (2 * nullspace[0] - 3 * nullspace[1]));

            var b = AA * AA;*/

            glControl = new OpenTK.GLControl(new OpenTK.Graphics.GraphicsMode(32, 24, 0, 8));


            if (glControl.Context.GraphicsMode.Samples == 0)
            {
                glControl = new OpenTK.GLControl(new OpenTK.Graphics.GraphicsMode(32, 24, 0, 8));
            }
            evwrapper = new EventWrapperGlControl(glControl);

            glControl.Paint += Gl_Paint;
            ViewManager = new DefaultCameraViewManager();
            ViewManager.Attach(evwrapper, camera1);

            tableLayoutPanel1.Controls.Add(glControl, 0, 0);
            glControl.Dock = DockStyle.Fill;
        }
        private EventWrapperGlControl evwrapper;
        GLControl glControl;
        private void Gl_Paint(object sender, PaintEventArgs e)
        {
            //if (!loaded)
            //  return;
            if (!glControl.Context.IsCurrent)
            {
                glControl.MakeCurrent();
            }


            Redraw();

        }


        List<Vector3d> points = new List<Vector3d>();



        Random r = new Random();

        StringBuilder header = new StringBuilder();
        Tuple<double[], int[]> ransacPlane(PointIndexer indexer, double thresh = 0.05, int minPoints = 100, int maxIteration = 1000)
        {
            double[] best_eq = null;
            int[] inliers = null;
            int[] best_inliers = new int[0] { };

            for (int i = 0; i < maxIteration; i++)
            {
                var rnd = indexer.GetRandomIndecies(3, r);


                var pt_samples = new[] { indexer.Points[indexer.Indicies[rnd[0]]],
                indexer.Points[indexer.Indicies[rnd[1]]],
                indexer.Points[indexer.Indicies[rnd[2]]]};

                var vecA = pt_samples[1] - pt_samples[0];
                var vecB = pt_samples[2] - pt_samples[0];

                var vecC = Vector3d.Cross(vecA, vecB);

                vecC = vecC / vecC.Length;
                var mul = vecC * pt_samples[1];
                var k = -(mul.X + mul.Y + mul.Z);
                var plane_eq = new[] { vecC[0], vecC[1], vecC[2], k };



                List<int> pt_id_inliers = new List<int>();// # list of inliers ids
                double[] dist_pt = new double[indexer.Points.Length];
                var q = Math.Sqrt(Math.Pow(plane_eq[0], 2) + Math.Pow(plane_eq[1], 2) + Math.Pow(plane_eq[2], 2));


                for (int j = 0; j < indexer.Points.Length; j++)
                {
                    if (!indexer.Contains(j)) continue;
                    dist_pt[j] = (plane_eq[0] * indexer.Points[j].X + plane_eq[1] * indexer.Points[j].Y + plane_eq[2] * indexer.Points[j].Z + plane_eq[3]) / q;
                    if (Math.Abs(dist_pt[j]) <= thresh)
                    {
                        pt_id_inliers.Add(j);
                    }
                }



                if (pt_id_inliers.Count > best_inliers.Length)
                {
                    best_eq = plane_eq;
                    best_inliers = pt_id_inliers.ToArray();
                }
                inliers = best_inliers;

            }
            return new Tuple<double[], int[]>(best_eq, inliers);
        }

        private void button3_Click(object sender, EventArgs e)
        {

            var plane1 = ransacPlane(this.pin, 2, maxIteration: ransacMaxIterations);
            if (plane1 != null)
            {
                cands = pin.GetPoints(plane1.Item2);
                Helpers.Add(new PlaneDrawer() { RelatedIndexer = pin.GetSubIndexer(plane1.Item2) });
                UpdateHelpersList();
            }
            else
            {
                SetStatus("plane not found");
            }
        }



        List<Tuple<Vector3d, double, int[]>> allSpheres = new List<Tuple<Vector3d, double, int[]>>();

        public Tuple<Vector3d, double, int[]> ransacSphere(PointIndexer indexer, SpaceInfo space, double thresh = 0.2, int maxIterations = 1000
            , double maxRadius = double.MaxValue, double minRadius = 5, double minAreaKoef = 0.8, bool useClusters = true, bool checkAreaKoef = false)
        {
            int[] best_inliers = new int[] { };
            double best_rad = double.MinValue;
            Vector3d best_center = Vector3d.Zero;
            int best_keys_cnt = int.MaxValue;
            double best_sum = int.MinValue;
            double best_area = 1;
            allSpheres.Clear();
            bool was = false;
            var cls = (space as DumbClusterSpaceInfo).GetClusters();
            /*    double[,] dd = new double[indexer.Points.Length, 3];
                for (int i = 0; i < indexer.Points.Length; i++)
                {
                    Vector3d item = indexer.Points[i];
                    dd[i, 0] = item.X;
                    dd[i, 1] = item.Y;
                    dd[i, 2] = item.Z;
                }
                Matrix<double> A = DenseMatrix.OfArray(dd);*/

            double[] dist_pt = new double[indexer.Points.Length];
            for (int i = 0; i < maxIterations; i++)
            {
                Vector3d[] pt_samples = null;

                Stopwatch ss3 = Stopwatch.StartNew();

                if (useClusters)
                {
                    var index = indexer.Indicies[r.Next(indexer.Indicies.Length)];
                    var pnt1 = indexer.Points[index];

                    var cluster = cls[r.Next(cls.Length)];
                    while (cluster.Indicies.Count() < 100)
                    {
                        cluster = cls[r.Next(cls.Length)];
                    }
                    var pind = new PointIndexer() { Points = indexer.Points, Indicies = cluster.Indicies };
                    var rnd = pind.GetRandomIndecies(4, r);
                    var pnts2 = pind.GetPoints(rnd);
                    while (true)
                    {
                        double maxl = 0;
                        foreach (var item in pnts2)
                        {
                            foreach (var item2 in pnts2)
                            {
                                maxl = Math.Max((item - item2).Length, maxl);
                            }
                        }
                        if (maxl <= maxRadius * 2) break;
                        rnd = pind.GetRandomIndecies(4, r);
                        pnts2 = pind.GetPoints(rnd);
                    }
                    pt_samples = rnd.Union(new[] { index }).Select(z => indexer.Points[z]).ToArray();
                }
                else
                {
                    pt_samples = pin.GetPoints(pin.GetRandomIndecies(4, r));
                }



                // Multiplied by (x²+y²+z²)
                Matrix4d d_matrix = new Matrix4d(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

                for (int j = 0; j < 4; j++)
                {
                    d_matrix[j, 0] = pt_samples[j].X;
                    d_matrix[j, 1] = pt_samples[j].Y;
                    d_matrix[j, 2] = pt_samples[j].Z;
                }

                var M11 = d_matrix.Determinant;


                for (int j = 0; j < 4; j++)
                {
                    d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                    d_matrix[j, 1] = pt_samples[j].Y;
                    d_matrix[j, 2] = pt_samples[j].Z;
                }
                var M12 = d_matrix.Determinant;

                for (int j = 0; j < 4; j++)
                {
                    d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                    d_matrix[j, 1] = pt_samples[j].X;
                    d_matrix[j, 2] = pt_samples[j].Z;
                }
                var M13 = d_matrix.Determinant;

                for (int j = 0; j < 4; j++)
                {
                    d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                    d_matrix[j, 1] = pt_samples[j].X;
                    d_matrix[j, 2] = pt_samples[j].Y;
                }
                var M14 = d_matrix.Determinant;

                // Multiplied by 1
                for (int j = 0; j < 4; j++)
                {
                    d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                    d_matrix[j, 1] = pt_samples[j].X;
                    d_matrix[j, 2] = pt_samples[j].Y;
                    d_matrix[j, 3] = pt_samples[j].Z;
                };
                var M15 = d_matrix.Determinant;

                //# Now we calculate the center and radius
                var center = new Vector3d(0.5 * (M12 / M11), -0.5 * (M13 / M11), 0.5 * (M14 / M11));

                var radius = Math.Sqrt(Vector3d.Dot(center, center) - (M15 / M11));
                if (double.IsNaN(radius) || double.IsInfinity(radius) || radius > maxRadius || radius < minRadius) continue;



                List<int> pt_id_inliers = new List<int>();

                Stopwatch ss = Stopwatch.StartNew();
                for (int j = 0; j < indexer.Points.Length; j++)
                {
                    dist_pt[j] = (indexer.Points[j] - center).Length;
                }
                //indexer.Points.Select(z => (z - center).Length).ToArray();
                ss.Stop();
                var ms1 = ss.ElapsedMilliseconds;

                /*
                                Matrix<double> B = DenseMatrix.OfArray(new double[,] { { center.X, center.Y, center.Z } });
                                Matrix<double> B1 = DenseMatrix.OfArray(new double[,] { { center.X, center.Y, center.Z } });
                                Vector<double> v = Vector<double>.Build.Dense(new double[] { center.X, center.Y, center.Z });

                                for (int j = 1; j < indexer.Points.Length; j++)
                                {
                                    B = B.Stack(B1);

                                }

                                Stopwatch ss2 = Stopwatch.StartNew();



                                var dist = A-B;
                                /*for (int j = 0; j < dist.RowCount; j++)
                                {
                                    dist.SetRow(j, dist.Row(j).Subtract(B.Row(0)));
                                }*/
                /*var ddd = dist.RowNorms(2).Subtract(radius).PointwiseAbs();

ss2.Stop();
                var ms2 = ss2.ElapsedMilliseconds;*/

                HashSet<string> keys = new HashSet<string>();
                List<Vector3d> pp = new List<Vector3d>();
                //var radius2 = radius * radius;
                for (int j = 0; j < indexer.Points.Length; j++)
                {
                    if (!indexer.Contains(j)) continue;
                    if (Math.Abs(dist_pt[j] - radius) <= thresh)
                    //if (ddd[j] <= thresh)
                    {
                        pt_id_inliers.Add(j);
                        pp.Add(indexer.Points[j]);
                        keys.Add(space.GetKey(indexer.Points[j]));
                    }
                }
                if (pt_id_inliers.Count == 0) continue;

                /*var maxx = pp.Max(z => z.X);
                var maxy = pp.Max(z => z.Y);
                var maxz = pp.Max(z => z.Z);

                var minx = pp.Min(z => z.X);
                var miny = pp.Min(z => z.Y);
                var minz = pp.Min(z => z.Z);

                var dx = (maxx - minx) / radius;
                var dy = (maxy - miny) / radius;
                var dz = (maxz - minz) / radius;
                var sum = dx + dy + dz;*/
                double area = Math.PI * 4 * radius * radius;
                var koef = pt_id_inliers.Count / area;

                //if (pt_id_inliers.Count > best_inliers.Length && radius < maxRadius)
                bool good = true;

                if (checkAreaKoef) if (!(koef > minAreaKoef)) good = false;
                if (!(koef > (best_inliers.Length / best_area))) { good = false; }
                if (good && radius < maxRadius)
                //if (keys.Count < best_keys_cnt)
                //if (sum > best_sum && keys.Count <= 1)
                {
                    was = true;
                    best_area = area;
                    best_inliers = pt_id_inliers.ToArray();
                    best_rad = radius;
                    best_center = center;
                    best_keys_cnt = keys.Count;
                    //best_sum = sum;
                }
                if (pt_id_inliers.Count > 30 && radius < maxRadius && koef > minAreaKoef)
                {
                    allSpheres.Add(new Tuple<Vector3d, double, int[]>(center, radius, pt_id_inliers.ToArray()));
                }
                ss3.Stop();
                var ms3 = ss3.ElapsedMilliseconds;
            }
            if (!was) return null;
            return new Tuple<Vector3d, double, int[]>(best_center, best_rad, best_inliers);
        }




        private void button5_Click(object sender, EventArgs e)
        {
            int maxr = int.Parse(textBox3.Text);
            Stopwatch s = Stopwatch.StartNew();
            var sphere = ransacSphere(pin, pin.csi, 1, maxIterations: int.Parse(textBox1.Text), maxRadius: maxr, useClusters: checkBox2.Checked);
            s.Stop();
            if (sphere == null || sphere.Item3.Length == 0)
            {
                SetStatus("sphere not found");
            }
            else
            {
                var koef = sphere.Item3.Length / (Math.PI * 4 * sphere.Item2 * sphere.Item2);
                SetStatus("koef: " + koef + "; points: " + sphere.Item3.Length + "; rad: " + sphere.Item2 + "; ms: " + s.ElapsedMilliseconds);
                label1.Text = (sphere.Item3.Length) + "";



                var relp = sphere.Item3.Select(y => pin.Points[y]).ToArray();
                var sub = pin.GetSubIndexer(sphere.Item3);
                Helpers.Add(new SphereDrawer() { Center = sphere.Item1, Radius = sphere.Item2, RelatedIndexer = sub });
                cands = relp;
                UpdateHelpersList();

            }

        }

        private void SetStatus(string v)
        {
            toolStripStatusLabel1.Text = v;
        }

        private void button6_Click(object sender, EventArgs e)
        {

            var cl = pin.csi.GetClusters();
            List<Vector3> clrs = new List<Vector3>();
            List<Vector3> clusterClrs = new List<Vector3>();
            foreach (var item in cl)
            {
                clusterClrs.Add(new Vector3((float)r.NextDouble(), (float)r.NextDouble(), (float)r.NextDouble()));
            }
            for (int i = 0; i < pin.Points.Length; i++)
            {
                var k = pin.csi.GetKey(pin.Points[i]);
                var fr = cl.First((z) => z.Key == k);
                for (int j = 0; j < cl.Length; j++)
                {
                    if (cl[j].Key == fr.Key)
                    {
                        clrs.Add(clusterClrs[j]);
                        break;
                    }
                }

            }
            currentSourceInfo.colors = clrs.ToArray();

        }

        PointIndexer pin
        {
            get
            {
                return currentSourceInfo.Data;
            }
            set
            {
                currentSourceInfo.Data = value;
            }
        }
        private void button7_Click(object sender, EventArgs e)
        {
            camera1.CamTo = Vector3.Zero;
            camera1.CamFrom = new Vector3(-10, 0, 0);
            camera1.CamUp = Vector3.UnitZ;
        }

        private void button8_Click(object sender, EventArgs e)
        {
            camera1.CamTo = Vector3.Zero;
            camera1.CamFrom = new Vector3(0, -10, 0);
            camera1.CamUp = Vector3.UnitZ;
        }

        private void button9_Click(object sender, EventArgs e)
        {
            camera1.CamTo = Vector3.Zero;
            camera1.CamFrom = new Vector3(0, 0, -10);
            camera1.CamUp = Vector3.UnitY;
        }

        Camera camera1 = new Camera() { IsOrtho = true };
        public CameraViewManager ViewManager;

        SourceInfo currentSourceInfo = null;
        void Redraw()
        {
            ViewManager.Update();

            GL.ClearColor(Color.LightGray);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);



            GL.Viewport(0, 0, glControl.Width, glControl.Height);
            var o2 = Matrix4.CreateOrthographic(glControl.Width, glControl.Height, 1, 1000);

            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref o2);

            Matrix4 modelview2 = Matrix4.LookAt(0, 0, 70, 0, 0, 0, 0, 1, 0);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref modelview2);



            GL.Enable(EnableCap.DepthTest);

            float zz = -500;
            GL.Begin(PrimitiveType.Quads);
            GL.Color3(Color.LightBlue);
            GL.Vertex3(-glControl.Width / 2, -glControl.Height / 2, zz);
            GL.Vertex3(glControl.Width / 2, -glControl.Height / 2, zz);
            GL.Color3(Color.AliceBlue);
            GL.Vertex3(glControl.Width / 2, glControl.Height / 2, zz);
            GL.Vertex3(-glControl.Width / 2, glControl.Height, zz);
            GL.End();

            camera1.Setup(glControl);

            GL.LineWidth(2);
            GL.Color3(Color.Red);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(100, 0, 0);
            GL.End();

            GL.Color3(Color.Green);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 100, 0);
            GL.End();

            GL.Color3(Color.Blue);
            GL.Begin(PrimitiveType.Lines);
            GL.Vertex3(0, 0, 0);
            GL.Vertex3(0, 0, 100);
            GL.End();


            GL.PointSize(psize);
            GL.Color3(Color.Blue);
            GL.Begin(PrimitiveType.Points);

            if (currentSourceInfo != null)
                for (int i = 0; i < pin.Points.Length; i++)
                {
                    Vector3d item = (Vector3d)pin.Points[i];
                    if (currentSourceInfo != null && currentSourceInfo.colors != null)
                    {
                        GL.Color3(currentSourceInfo.colors[i]);
                    }
                    GL.Vertex3(item);
                }
            GL.End();
            GL.PointSize(psize2);

            GL.Color3(Color.Orange);
            GL.Begin(PrimitiveType.Points);

            if (cands != null)
                foreach (var item in cands)
                {
                    GL.Vertex3(item);
                }
            GL.End();

            foreach (var he in Helpers)
            {
                if (!he.Visible) continue;
                he.Draw();
            }
            glControl.SwapBuffers();
        }


        int psize = 3;
        int psize2 = 8;
        private void timer1_Tick(object sender, EventArgs e)
        {
            glControl.Invalidate();
        }

        private void button10_Click(object sender, EventArgs e)
        {
            pin.Centralized();
        }

        private void button11_Click(object sender, EventArgs e)
        {
            pin.ChangeAxis(1);
        }

        private void button12_Click(object sender, EventArgs e)
        {
            pin.ChangeAxis(2);
        }


        private void button13_Click(object sender, EventArgs e)
        {
            var cc = pin.GetRandomIndecies(4, r);

            var cl = pin.csi.GetClusters();
            var clus = cl[r.Next(cl.Length)];
            while (clus.Indicies.Length < 4)
            {
                clus = cl[r.Next(cl.Length)];
            }
            int[] rand = new int[4];
            while (rand.GroupBy(z => z).Count() != 4)
            {
                for (int i = 0; i < rand.Length; i++)
                {
                    rand[i] = r.Next(clus.Indicies.Length);
                }
            }
            var pp = rand.Select(z => pin.Points[z]).ToArray();
            cands = new[] { pin.Points[cc[0]] };
            var v1 = (cands[0]);
            label2.Text = Math.Round(v1.X, 3) + " " + Math.Round(v1.Y, 3) + " " + Math.Round(v1.Z, 3);


        }

        Vector3d[] cands;
        public List<AbstractHelper> Helpers = new List<AbstractHelper>();

        private void button14_Click(object sender, EventArgs e)
        {

        }


        List<SourceInfo> sources = new List<SourceInfo>();

        private void toolStripButton1_Click(object sender, EventArgs e)
        {

        }

        private void UpdateSourcesList()
        {
            listView2.Items.Clear();
            foreach (var item in sources)
            {
                listView2.Items.Add(new ListViewItem(new string[] { item.Name }) { Tag = item });
            }
        }

        private void button15_Click(object sender, EventArgs e)
        {
            // Multiplied by (x²+y²+z²)
            Matrix4d d_matrix = new Matrix4d(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
            var pt_samples = cands;
            for (int j = 0; j < 4; j++)
            {
                d_matrix[j, 0] = pt_samples[j].X;
                d_matrix[j, 1] = pt_samples[j].Y;
                d_matrix[j, 2] = pt_samples[j].Z;
            }

            var M11 = d_matrix.Determinant;


            for (int j = 0; j < 4; j++)
            {
                d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                d_matrix[j, 1] = pt_samples[j].Y;
                d_matrix[j, 2] = pt_samples[j].Z;
            }
            var M12 = d_matrix.Determinant;

            for (int j = 0; j < 4; j++)
            {
                d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                d_matrix[j, 1] = pt_samples[j].X;
                d_matrix[j, 2] = pt_samples[j].Z;
            }
            var M13 = d_matrix.Determinant;

            for (int j = 0; j < 4; j++)
            {
                d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                d_matrix[j, 1] = pt_samples[j].X;
                d_matrix[j, 2] = pt_samples[j].Y;
            }
            var M14 = d_matrix.Determinant;

            // Multiplied by 1
            for (int j = 0; j < 4; j++)
            {
                d_matrix[j, 0] = Vector3d.Dot(pt_samples[j], pt_samples[j]);
                d_matrix[j, 1] = pt_samples[j].X;
                d_matrix[j, 2] = pt_samples[j].Y;
                d_matrix[j, 3] = pt_samples[j].Z;
            };
            var M15 = d_matrix.Determinant;

            //# Now we calculate the center and radius
            var center = new Vector3d(0.5 * (M12 / M11), -0.5 * (M13 / M11), 0.5 * (M14 / M11));

            var radius = Math.Sqrt(Vector3d.Dot(center, center) - (M15 / M11));
            if (double.IsNaN(radius) || double.IsInfinity(radius)) return;


            List<int> pt_id_inliers = new List<int>();
            var dist_pt = pin.Points.Select(z => (z - center).Length).ToArray();

            double thresh = 2;
            HashSet<string> keys = new HashSet<string>();
            List<Vector3d> pp = new List<Vector3d>();
            for (int j = 0; j < pin.Points.Length; j++)
            {
                if (!pin.Contains(j)) continue;
                if (Math.Abs(dist_pt[j] - radius) <= thresh)
                {
                    pt_id_inliers.Add(j);
                    pp.Add(pin.Points[j]);
                    keys.Add(pin.csi.GetKey(pin.Points[j]));
                }
            }
            if (pt_id_inliers.Count == 0) return;

            var maxx = pp.Max(z => z.X);
            var maxy = pp.Max(z => z.Y);
            var maxz = pp.Max(z => z.Z);

            var minx = pp.Min(z => z.X);
            var miny = pp.Min(z => z.Y);
            var minz = pp.Min(z => z.Z);

            var dx = (maxx - minx) / radius;
            var dy = (maxy - miny) / radius;
            var dz = (maxz - minz) / radius;
            var sum = dx + dy + dz;
            double area = Math.PI * 4 * radius * radius;

            var koef = pt_id_inliers.Count / area;
            cands = pt_id_inliers.Select(z => pin.Points[z]).ToArray();
            var sub = pin.GetSubIndexer(pt_id_inliers.ToArray());
            Helpers.Add(new SphereDrawer() { Center = center, Radius = radius, RelatedIndexer = sub });
            UpdateHelpersList();
        }

        private void button16_Click(object sender, EventArgs e)
        {
            if (cands == null || cands.Count() < 4)
            {
                cands = new[] { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero };
            }
            cands[0] = pin.Points[r.Next(pin.Points.Length)];
        }

        private void button17_Click(object sender, EventArgs e)
        {
            if (cands == null || cands.Count() < 4)
            {
                cands = new[] { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero };
            }
            cands[1] = pin.Points[r.Next(pin.Points.Length)];
        }

        private void button19_Click(object sender, EventArgs e)
        {
            if (cands == null || cands.Count() < 4)
            {
                cands = new[] { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero };
            }
            cands[2] = pin.Points[r.Next(pin.Points.Length)];
        }

        private void button18_Click(object sender, EventArgs e)
        {
            if (cands == null || cands.Count() < 4)
            {
                cands = new[] { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero };
            }
            cands[3] = pin.Points[r.Next(pin.Points.Length)];
        }

        public void UpdateHelpersList()
        {
            listView1.Items.Clear();
            foreach (var item in Helpers)
            {
                listView1.Items.Add(new ListViewItem(new string[] { item.GetType().Name, item.Visible + "" }) { Tag = item });
            }
        }

        private void listView1_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (listView1.SelectedItems.Count == 0)
            {
                cands = null;
                return;
            }
            var hlp = listView1.SelectedItems[0].Tag as AbstractHelper;
            propertyGrid2.SelectedObject = hlp;
            cands = new Vector3d[] { };

            cands = hlp.RelatedPoints;


        }

        private void button20_Click(object sender, EventArgs e)
        {
            foreach (var item in allSpheres)
            {
                Helpers.Add(new SphereDrawer() { Center = item.Item1, Radius = item.Item2 });
            }
            UpdateHelpersList();
        }

        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Helpers.Clear();
            UpdateHelpersList();
        }

        private void button21_Click(object sender, EventArgs e)
        {
            var indexer = pin;
            var index = indexer.Indicies[r.Next(indexer.Indicies.Length)];
            var pnt1 = indexer.Points[index];
            var cls = (pin.csi).GetClusters();
            var cluster = cls[r.Next(cls.Length)];
            var avg = cls.Average(z => z.Indicies.Length);
            while (cluster.Indicies.Count() < avg)
            {
                cluster = cls[r.Next(cls.Length)];
            }
            //cluster = cls.OrderByDescending(z => z.Item2.Length).First();
            var pind = new PointIndexer() { Points = indexer.Points, Indicies = cluster.Indicies };
            var rnd = pind.GetRandomIndecies(4, r);
            var pnts2 = pind.GetPoints(rnd);
            while (true)
            {
                double maxl = 0;
                foreach (var item in pnts2)
                {
                    foreach (var item2 in pnts2)
                    {
                        maxl = Math.Max((item - item2).Length, maxl);
                    }
                }
                if (maxl <= 15 * 2) break;
                rnd = pind.GetRandomIndecies(4, r);
                pnts2 = pind.GetPoints(rnd);
            }
            cands = pnts2;


        }

        private void button22_Click(object sender, EventArgs e)
        {
            List<Vector3d> pp = new List<Vector3d>();
            for (int i = 0; i < pin.Points.Length; i++)
            {
                if (i % 2 == 0)
                {
                    pp.Add(pin.Points[i]);
                }
            }

            pin = new PointIndexer() { Points = pp.ToArray(), Indicies = pp.Select((z, i) => i).ToArray() };
            pin.Init();
            updateStatus2();

        }

        void updateStatus2()
        {
            toolStripStatusLabel2.Text = "Points: " + currentSourceInfo.Data.Indicies.Length;
        }

        private void listView2_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        public void LoadSource(SourceInfo si)
        {
            //this.pin = si.Data;            
            currentSourceInfo = si;
            pin.Init();

        }
        private void listView2_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            if (listView2.SelectedItems.Count == 0) return;
            var si = listView2.SelectedItems[0].Tag as SourceInfo;
            LoadSource(si);
            updateStatus2();
        }

        private void toolStripButton2_Click(object sender, EventArgs e)
        {
            timer2.Enabled = !timer2.Enabled;
            toolStripButton2.Text = timer2.Enabled ? "stop animation" : "run animation";
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            if (sources.Count == 0 || currentSourceInfo == null) return;
            var index = sources.IndexOf(currentSourceInfo) + 1;

            if (index == sources.Count - 1)
            {
                if (checkBox1.Checked)
                {
                    index = 0;
                }
                else { timer2.Enabled = false; return; }
            }
            LoadSource(sources[index]);
        }

        private void textBox2_TextChanged(object sender, EventArgs e)
        {
            timer2.Interval = int.Parse(textBox2.Text);
        }

        private void button23_Click(object sender, EventArgs e)
        {
            camera1.CamTo = Vector3.Zero;
            camera1.CamFrom = new Vector3(10, 0, 0);
            camera1.CamUp = Vector3.UnitZ;
        }

        private void button24_Click(object sender, EventArgs e)
        {
            var cls = pin.csi.GetClusters();
            foreach (var cluster in cls)
            {
                if (cluster.Indicies.Length < 100) continue;
                var pin2 = new PointIndexer() { Points = pin.Points, Indicies = cluster.Indicies };
                pin2.Init();
                var sphere = ransacSphere(pin2, pin2.csi, 1, maxIterations: 10, maxRadius: 30);
                if (sphere != null)
                {
                    var kk = sphere.Item3.Length / (double)cluster.Indicies.Length;
                    //if (kk > 0.9)
                    {
                        Helpers.Add(new SphereDrawer() { Radius = sphere.Item2, Center = sphere.Item1 });
                        UpdateHelpersList();
                        break;
                    }
                }
            }
        }

        private void button25_Click(object sender, EventArgs e)
        {
            pin.ClusterStep = double.Parse(textBox6.Text, CultureInfo.InvariantCulture);
            pin.Init();
            listView3.Items.Clear();
            var array = pin.csi.GetClusters();
            for (int i = 0; i < array.Length; i++)
            {
                var item = array[i];
                listView3.Items.Add(new ListViewItem(new string[] { i + "", item.Name, item.Indicies.Length + "" }) { Tag = item });
            }
        }

        private void listView3_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (listView3.SelectedItems.Count == 0) return;
            var ci = listView3.SelectedItems[0].Tag as ClusterInfo;
            propertyGrid2.SelectedObject = ci;
            cands = pin.GetPoints(ci.Indicies);
        }

        private void button26_Click(object sender, EventArgs e)
        {

        }

        private void random3PointsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (listView3.SelectedItems.Count == 0) return;
            var ci = listView3.SelectedItems[0].Tag as ClusterInfo;
            var pp = new PointIndexer() { Points = pin.Points, Indicies = ci.Indicies };
            var pnts2 = pp.GetPoints(pp.GetRandomIndecies(4, r));
            while (true)
            {
                double maxl = 0;
                foreach (var item in pnts2)
                {
                    foreach (var item2 in pnts2)
                    {
                        maxl = Math.Max((item - item2).Length, maxl);
                    }
                }
                if (maxl <= 15 * 2) break;
                pnts2 = pp.GetPoints(pp.GetRandomIndecies(4, r));
            }
            cands = pnts2;
        }

        private void button27_Click(object sender, EventArgs e)
        {
            double surfNoise = double.Parse(textBox5.Text.Replace(",", "."), CultureInfo.InvariantCulture);
            List<Vector3d> vv = new List<Vector3d>();
            var rad = 100;
            for (int i = 0; i < 10000; i++)
            {
                var ang = r.Next(-180, 180) * Math.PI / 180f;
                var noise = (r.NextDouble() - 0.5) * 5;
                var xx = (noise + rad) * Math.Cos(ang);
                var yy = (noise + rad) * Math.Sin(ang);
                Vector3d v = new Vector3d(xx, yy, 0);
                var ang2 = r.Next(-180, 180) * Math.PI / 180f;
                var mtr = Matrix4d.CreateFromAxisAngle(Vector3d.UnitX, ang2);
                var res = Vector3d.TransformVector(v, mtr);

                vv.Add(res);
            }
            if (checkBox6.Checked)
            {
                for (int i = 0; i < 10000; i++)
                {
                    var ang = r.Next(-180, 180) * Math.PI / 180f;
                    var noise = (r.NextDouble() - 0.5) * surfNoise;
                    var xx = (noise + rad) * Math.Cos(ang);
                    var yy = (noise + rad) * Math.Sin(ang);
                    Vector3d v = new Vector3d(xx, yy, 0);
                    var ang2 = r.Next(-180, 180) * Math.PI / 180f;
                    var mtr = Matrix4d.CreateFromAxisAngle(Vector3d.UnitX, ang2);
                    var res = Vector3d.TransformVector(v, mtr);

                    vv.Add(res);
                }
            }
            if (checkBox4.Checked)
            {
                for (int i = 0; i < 10000; i++)
                {
                    vv.Add(new Vector3d((r.NextDouble() - 0.5) * 200, (r.NextDouble() - 0.5) * 200, (r.NextDouble() - 0.5) * 200));
                }
            }
            var pin = PointIndexer.FromPoints(vv.ToArray());
            pin.Init();
            if (checkBox8.Checked)
            {
                var shiftx = r.Next(-200, 200);
                var shifty = r.Next(-200, 200);
                var shiftz = r.Next(-200, 200);
                pin.TranslatePoints(new Vector3d(shiftx, shifty, shiftz));
            }
            if (checkBox7.Checked && currentSourceInfo != null)
            {
                currentSourceInfo.Data.Merge(pin);
            }
            else
            {
                if (SingleSourceMode)
                {
                    sources.Clear();
                }
                sources.Add(new SourceInfo() { Data = pin, Name = "sphere" + sources.Count });
            }

            LoadSource(sources.Last());
            UpdateSourcesList();

        }

        private void button14_Click_1(object sender, EventArgs e)
        {
            List<Vector3d> vv = new List<Vector3d>();
            var rad = 100;
            var ang2 = r.Next(-180, 180) * Math.PI / 180f;
            var ang = r.Next(-180, 180) * Math.PI / 180f;

            for (int i = 0; i < 10000; i++)
            {
                var noise = (r.NextDouble() - 0.5) * 5;
                var xx = (noise + (r.NextDouble() - 0.5) * 200);
                var yy = (noise + (r.NextDouble() - 0.5) * 200);
                Vector3d v = new Vector3d(xx, yy, 0);

                var mtr = Matrix4d.CreateFromAxisAngle(Vector3d.UnitX, ang2);
                var mtr2 = Matrix4d.CreateFromAxisAngle(Vector3d.UnitY, ang);
                var res = Vector3d.TransformVector(v, mtr);
                res = Vector3d.TransformVector(res, mtr2);

                vv.Add(res);
            }
            if (checkBox4.Checked)
            {
                for (int i = 0; i < 3000; i++)
                {
                    vv.Add(new Vector3d((r.NextDouble() - 0.5) * 400, (r.NextDouble() - 0.5) * 400, (r.NextDouble() - 0.5) * 400));
                }
            }
            var pin = PointIndexer.FromPoints(vv.ToArray());
            pin.Init();
            if (SingleSourceMode)
            {
                sources.Clear();
            }
            sources.Add(new SourceInfo() { Data = pin, Name = "sphere" + sources.Count });
            LoadSource(sources.Last());
            UpdateSourcesList();
        }

        private void textBox4_TextChanged(object sender, EventArgs e)
        {
            try
            {
                psize = int.Parse(textBox4.Text);
            }
            catch (Exception ex)
            {

            }
        }

        int ransacMaxIterations = 1000;
        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            try
            {
                ransacMaxIterations = int.Parse(textBox1.Text);
            }
            catch (Exception ex)
            {

            }
        }

        private void button28_Click(object sender, EventArgs e)
        {
            cands = null;
        }

        private void loadToolStripMenuItem_Click(object sender, EventArgs e)
        {
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Multiselect = true;

            if (ofd.ShowDialog() == DialogResult.OK)
            {
                LoadParams lp = new LoadParams();

                lp.ShowDialog();
                sources.Clear();

                Waiter w = new Waiter();
                Action act = () =>
                {
                    for (int i = 0; i < ofd.FileNames.Length; i++)
                    {
                        double prog = (double)i / ofd.FileNames.Length;
                        w.Progress((int)(prog * 100));

                        var t = PlyLoader.LoadPly(ofd.FileNames[i]);
                        if (lp.ScaleMode == LoadParams.ScaleModeEnum.Auto)
                        {
                            var bbox = t.BoundingBox();
                            var koef = 1000 / bbox.Sizes.Max();
                            t.ScalePoints(koef);
                        }
                        if (lp.ScaleMode == LoadParams.ScaleModeEnum.Manual)
                        {
                            t.ScalePoints(lp.ScaleKoef);
                        }

                        sources.Add(new SourceInfo() { Name = new FileInfo(ofd.FileNames[i]).Name, Data = t });

                        if (lp.Centralized)
                        {
                            t.ChangeAxis(1);
                            t.ChangeAxis(2);
                            t.Centralized();
                        }

                    }
                };
                w.Init(act);
                w.ShowDialog();

                LoadSource(sources.First());

                UpdateSourcesList();
                updateStatus2();
            }
        }

        private void saveToolStripMenuItem_Click(object sender, EventArgs e)
        {
            SaveFileDialog sfd = new SaveFileDialog();
            sfd.Filter = "ply files (*.ply)|*.ply";
            if (sfd.ShowDialog() == DialogResult.OK)
            {
                PlyLoader.SavePly(sfd.FileName, currentSourceInfo.Data.Points.ToArray(), currentSourceInfo.colors);
            }
        }

        public bool SingleSourceMode = true;
        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            SingleSourceMode = radioButton1.Checked;
        }

        private void deleteRelatedPointsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (listView1.SelectedItems.Count == 0) return;
            var hlp = listView1.SelectedItems[0].Tag as AbstractHelper;

            //hlp.RelatedIndexer.
        }

        private void deleteToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (listView1.SelectedItems.Count == 0) return;
            var hlp = listView1.SelectedItems[0].Tag as AbstractHelper;
            Helpers.Remove(hlp);
            UpdateHelpersList();
            cands = null;
        }

        private void textBox7_TextChanged(object sender, EventArgs e)
        {
            try
            {
                psize2 = int.Parse(textBox7.Text);
            }
            catch (Exception ex)
            {

            }
        }
    }
}

