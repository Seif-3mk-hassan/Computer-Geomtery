using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{

    public class ExtremeSegments : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            outPoints = new List<Point>();

            if (points == null || points.Count == 0)
            {
                return;
            }


            List<Point> uniquePoints = HullSupport.Distinct(points);


            if (uniquePoints.Count <= 2)
            {
                Publish(uniquePoints, ref outPoints, ref outLines);
                return;
            }


            for (int i = 0; i < uniquePoints.Count - 1; i++)
            {
                for (int j = i + 1; j < uniquePoints.Count; j++)
                {
                    Point a = uniquePoints[i];
                    Point b = uniquePoints[j];

                    if (!IsExtreme(a, b, uniquePoints) || !IsMaximal(a, b, uniquePoints))
                    {
                        continue;
                    }

                    if (!outPoints.Any(p => p.Equals(a)))
                    {
                        outPoints.Add(a);
                    }

                    if (!outPoints.Any(p => p.Equals(b)))
                    {
                        outPoints.Add(b);
                    }
                }
            }


            if (outPoints.Count == 0)
            {
                return;
            }

            Point pivot = HullSupport.FindLowestThenLeftmost(outPoints);
            outPoints.Sort((a, b) => HullSupport.CompareByAngle(pivot, a, b));

            outLines = HullSupport.BuildEdges(outPoints);
        }


        private static bool IsExtreme(Point a, Point b, List<Point> points)
        {
            bool hasLeft = false;
            bool hasRight = false;

            foreach (Point p in points)
            {
                double turn = HullSupport.Cross(a, b, p);
                if (turn > 0)
                {
                    hasLeft = true;
                }
                else if (turn < 0)
                {
                    hasRight = true;
                }

                if (hasLeft && hasRight)
                {
                    return false;
                }
            }

            return true;
        }

 
        private static bool IsMaximal(Point a, Point b, List<Point> points)
        {
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            double lengthSquared = HullSupport.SquaredDistance(a, b);

            foreach (Point p in points)
            {
                if (p.Equals(a) || p.Equals(b) || HullSupport.Cross(a, b, p) != 0)
                {
                    continue;
                }


                double projection = (p.X - a.X) * dx + (p.Y - a.Y) * dy;
                if (projection < 0 || projection > lengthSquared)
                {
                    return false;
                }
            }

            return true;
        }

     
        private static void Publish(List<Point> vertices, ref List<Point> outPoints, ref List<Line> outLines)
        {
            outPoints = vertices;
            outLines = HullSupport.BuildEdges(vertices);
        }

        public override string ToString()
        {
            return "Convex Hull - Extreme Segments";
        }
    }
}
