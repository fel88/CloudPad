﻿using OpenTK;
using System;
using System.Collections.Generic;
using System.Data;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace _3d
{
    public class PlyLoader
    {
        public class PropInfo
        {
            public string Name;
            public string Type;
            public int TypeLen
            {
                get
                {
                    if (Type == "double") return 8;
                    if (Type == "float") return 4;
                    if (Type == "int") return 4;
                    if (Type == "uchar") return 1;
                    throw new ArgumentException();
                }
            }
        }

        public static PointIndexer LoadPly(string path)
        {
            var points = new List<Vector3d>();

            var rf = File.ReadAllLines(path);
            bool binary = false;
            if (rf.Take(5).Any(z => z.Contains("binary")))
            {
                binary = true;

                var bts = File.ReadAllBytes(path);
                List<byte> accum1 = new List<byte>();
                int pos = 0;
                List<PropInfo> props = new List<PropInfo>();
                for (int i = 0; i < bts.Length; i++)
                {
                    accum1.Add(bts[i]);
                    if (bts[i] == 0x0a)
                    {
                        var str = Encoding.UTF8.GetString(accum1.ToArray());
                        if (str.Contains("property"))
                        {
                            var aa1 = str.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries).ToArray();
                            props.Add(new PropInfo() { Name = aa1[2], Type = aa1[1] });


                        }
                        if (str.Contains("end_header"))
                        {
                            pos = i;
                            break;
                        }
                        accum1.Clear();
                    }
                }
                var data = bts.Skip(pos + 1).ToArray();
                List<double> dd = new List<double>();
                var word = props.Sum(z => z.TypeLen);
                for (int i = 0; i < data.Length; i += word)
                {
                    int shift = 0;
                    foreach (var prop in props)
                    {
                        if (prop.Type == "double")
                        {
                            dd.Add(BitConverter.ToDouble(data, i + shift));
                            shift += 8;
                        }
                        if (prop.Type == "float")
                        {
                            dd.Add(BitConverter.ToSingle(data, i + shift));
                            shift += 4;
                        }
                    }

                }

                for (int i = 0; i < dd.Count; i += 3)
                {
                    Vector3d v = new Vector3d(dd[i], dd[i + 1], dd[i + 2]);
                    points.Add(v);
                }
                return PointIndexer.FromPoints(points.ToArray());
            }
            bool end = false;

            foreach (var item in rf)
            {
                var ar = item.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries).ToArray();
                if (item.ToLower().Contains("end") && item.Contains("header"))
                {
                    end = true;


                    continue;
                }
                if (end)
                {
                    var d1 = ar.Select(z => float.Parse(z.Replace(",", "."), CultureInfo.InvariantCulture)).ToArray();
                    var v1 = new Vector3d(d1[0], d1[1], d1[2]);
                    points.Add(v1);
                }
            }


            return PointIndexer.FromPoints(points.ToArray());
        }


        public static void SavePly(string path, Vector3d[] pnts, Vector3[] colors = null, int[] indcs = null)
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("ply");
            sb.AppendLine("format ascii 1.0");
            sb.AppendLine("comment VCGLIB generated");
            if (indcs != null)
            {
                sb.AppendLine("element vertex " + indcs.Length);
            }
            else
                sb.AppendLine("element vertex " + pnts.Length);
            sb.AppendLine("property float x");
            sb.AppendLine("property float y");
            sb.AppendLine("property float z");
            if (colors != null)
            {
                sb.AppendLine("property uchar red");
                sb.AppendLine("property uchar green");
                sb.AppendLine("property uchar blue");
            }
            sb.AppendLine("element face 0");
            sb.AppendLine("property list uchar int vertex_indices");
            sb.AppendLine("end_header");
            HashSet<int> hash = new HashSet<int>();
            if (indcs != null)
                foreach (var item in indcs)
                {
                    hash.Add(item);
                }

            for (int i = 0; i < pnts.Length; i++)
            {
                if (indcs != null)
                {
                    if (!hash.Contains(i)) continue;
                }
                Vector3d p = pnts[i];
                if (colors != null)
                {
                    Vector3 clr = colors[i];
                    sb.AppendLine((p.X + " " + p.Y + " " + p.Z + " " + (int)(clr.X * 255) + " " + (int)(clr.Y * 255) + " " + (int)(clr.Z * 255)).Replace(",", "."));
                }
                else
                {
                    sb.AppendLine((p.X + " " + p.Y + " " + p.Z).Replace(",", "."));
                }
            }
            File.WriteAllText(path, sb.ToString());
        }


    }

}

