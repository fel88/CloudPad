using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;

namespace _3d
{

    public class SphereDrawer : AbstractHelper
    {
        public Vector3d Center { get { return _center; } set { _center = value; } }
        Vector3d _center;

        public double Radius { get; set; }
        public double X { get => Center.X; set => _center.X = value; }
        public double Y { get => Center.Y; set => _center.Y = value; }
        public double Z { get => Center.Z; set => _center.Z = value; }
        void drawSphere(double r, int lats, int longs)
        {
            int i, j;
            for (i = 0; i <= lats; i++)
            {
                double lat0 = Math.PI * (-0.5 + (double)(i - 1) / lats);
                double z0 = Math.Sin(lat0);
                double zr0 = Math.Cos(lat0);

                double lat1 = Math.PI * (-0.5 + (double)i / lats);
                double z1 = Math.Sin(lat1);
                double zr1 = Math.Cos(lat1);

                GL.Begin(PrimitiveType.QuadStrip);
                for (j = 0; j <= longs; j++)
                {
                    double lng = 2 * Math.PI * (double)(j - 1) / longs;
                    double x = Math.Cos(lng);
                    double y = Math.Sin(lng);

                    GL.Normal3(x * zr0, y * zr0, z0);
                    GL.Vertex3(r * x * zr0, r * y * zr0, r * z0);
                    GL.Normal3(x * zr1, y * zr1, z1);
                    GL.Vertex3(r * x * zr1, r * y * zr1, r * z1);
                }
                GL.End();
            }
        }
        public override void Draw()
        {
            GL.PushMatrix();
            GL.Translate(Center);
            drawSphere(Radius, 20, 20);
            GL.PopMatrix();
        }
    }
}
