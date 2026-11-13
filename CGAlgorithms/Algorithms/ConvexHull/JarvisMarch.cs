using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Convex hull using the Jarvis march (gift wrapping).
    /// Start at the lowest point and repeatedly walk to the most clockwise point
    /// around the current vertex. Runs in O(n * h), where h is the hull size.
    /// </summary>
    public class JarvisMarch : Algorithm
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

            Point start = HullSupport.FindLowestThenLeftmost(points);

            List<Point> hull = new List<Point>();
            Point current = start;

            do
            {
                hull.Add(current);
                current = FindNextVertex(points, current);

                // The count guard only protects against malformed input;
                // for valid input the walk returns to the start point.
            }
            while (!current.Equals(start) && hull.Count <= points.Count);

            return hull;
        }

        /// <summary>
        /// Finds the next hull vertex when wrapping around the current one.
        /// </summary>
        /// <param name="points">All input points.</param>
        /// <param name="current">The vertex the wrapping is currently at.</param>
        /// <returns>
        /// The most clockwise point around <paramref name="current"/>,
        /// or the farthest one when several points share the same direction.
        /// </returns>
        private static Point FindNextVertex(List<Point> points, Point current)
        {
            Point next = null;

            foreach (Point candidate in points)
            {
                if (candidate.Equals(current))
                {
                    continue;
                }

                if (next == null)
                {
                    next = candidate;
                    continue;
                }

                double turn = HullSupport.Cross(current, next, candidate);

                // A right turn means the candidate is more clockwise than the current pick.
                // A collinear candidate is only interesting when it is farther away,
                // so the points in between are skipped instead of becoming hull vertices.
                bool isMoreClockwise = turn < 0;
                bool isFartherOnSameRay = turn == 0
                    && HullSupport.SquaredDistance(current, candidate) > HullSupport.SquaredDistance(current, next);

                if (isMoreClockwise || isFartherOnSameRay)
                {
                    next = candidate;
                }
            }

            return next;
        }

        public override string ToString()
        {
            return "Convex Hull - Jarvis March";
        }
    }
}
