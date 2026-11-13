using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Convex hull using the QuickHull algorithm.
    /// Split the points with the left most and right most points, then keep the farthest
    /// point on each side and recurse on the two halves. Runs in O(n log n) on average
    /// and O(n^2) in the worst case.
    /// </summary>
    public class QuickHull : Algorithm
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

            Point left = points[0];
            Point right = points[0];

            foreach (Point point in points)
            {
                if (point.X < left.X || (point.X == left.X && point.Y < left.Y))
                {
                    left = point;
                }

                if (point.X > right.X || (point.X == right.X && point.Y > right.Y))
                {
                    right = point;
                }
            }

            // The upper chain goes from the left most point to the right most one,
            // the lower chain closes the hull on the way back.
            List<Point> hull = new List<Point>();
            hull.Add(left);
            hull.AddRange(FindChain(left, right, points));
            hull.Add(right);
            hull.AddRange(FindChain(right, left, points));

            return hull;
        }

        /// <summary>
        /// Computes the part of the hull that joins two points while staying on the left side
        /// of the directed line from <paramref name="from"/> to <paramref name="to"/>.
        /// The two end points themselves are not included, they belong to the caller.
        /// </summary>
        /// <param name="from">The first end point of the chain.</param>
        /// <param name="to">The second end point of the chain.</param>
        /// <param name="candidates">The points available for the chain.</param>
        /// <returns>The chain vertices, ordered from <paramref name="from"/> to <paramref name="to"/>.</returns>
        private static List<Point> FindChain(Point from, Point to, List<Point> candidates)
        {
            List<Point> leftSide = new List<Point>();
            foreach (Point candidate in candidates)
            {
                if (HullSupport.Cross(from, to, candidate) > 0)
                {
                    leftSide.Add(candidate);
                }
            }

            if (leftSide.Count == 0)
            {
                return leftSide;
            }

            Point farthest = FindFarthestFromLine(from, to, leftSide);

            // Points between "from" and "farthest", then the farthest point itself,
            // then the points between "farthest" and "to".
            List<Point> chain = FindChain(from, farthest, leftSide);
            chain.Add(farthest);
            chain.AddRange(FindChain(farthest, to, leftSide));

            return chain;
        }

        /// <summary>
        /// Finds the point of the given set that is the farthest from the line through from and to.
        /// Since every point of the set is on the same side of the line, the largest
        /// cross product also has the largest distance.
        /// </summary>
        private static Point FindFarthestFromLine(Point from, Point to, List<Point> points)
        {
            Point farthest = points[0];
            double largestArea = HullSupport.Cross(from, to, farthest);

            for (int i = 1; i < points.Count; i++)
            {
                double area = HullSupport.Cross(from, to, points[i]);
                if (area > largestArea)
                {
                    largestArea = area;
                    farthest = points[i];
                }
            }

            return farthest;
        }

        public override string ToString()
        {
            return "Convex Hull - Quick Hull";
        }
    }
}
