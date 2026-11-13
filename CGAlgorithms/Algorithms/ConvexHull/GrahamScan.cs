using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Convex hull using the Graham scan.
    /// Sort the points by polar angle around the lowest point, then sweep them once
    /// while keeping a stack of the hull vertices. Runs in O(n log n).
    /// </summary>
    public class GrahamScan : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            List<Point> hull = BuildHull(HullSupport.Distinct(points));

            outPoints = hull;
            outLines = HullSupport.BuildEdges(hull);
        }

        /// <summary>
        /// Computes the convex hull of the given points.
        /// </summary>
        /// <param name="points">The input points, without duplicates.</param>
        /// <returns>The hull vertices in counter-clockwise order.</returns>
        private static List<Point> BuildHull(List<Point> points)
        {
            // Fewer than three distinct points: the hull is the points themselves.
            if (points.Count < 3)
            {
                return points;
            }

            Point pivot = HullSupport.FindLowestThenLeftmost(points);

            List<Point> ordered = new List<Point>();
            foreach (Point point in points)
            {
                if (!point.Equals(pivot))
                {
                    ordered.Add(point);
                }
            }

            ordered.Sort((a, b) => HullSupport.CompareByAngle(pivot, a, b));

            // A list used as a stack: the last element is the top of the stack.
            List<Point> hull = new List<Point>();
            hull.Add(pivot);
            hull.Add(ordered[0]);

            for (int i = 1; i < ordered.Count; i++)
            {
                Point next = ordered[i];

                // A right turn (or a collinear point) means the top of the stack is
                // not a hull vertex, so drop it and look at the previous one.
                while (hull.Count > 1 && HullSupport.Cross(hull[hull.Count - 2], hull[hull.Count - 1], next) <= 0)
                {
                    hull.RemoveAt(hull.Count - 1);
                }

                hull.Add(next);
            }

            return hull;
        }

        public override string ToString()
        {
            return "Convex Hull - Graham Scan";
        }
    }
}
