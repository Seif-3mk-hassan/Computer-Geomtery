using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Convex hull using the extreme segments definition: the directed segment from a point a
    /// to a point b is extreme when every other point lies to its left or exactly on its line.
    /// Segments contained in a longer collinear segment are not hull edges but only parts of
    /// one, so only the maximal extreme segments are kept.
    /// Every hull vertex has exactly one outgoing extreme segment, which makes the hull a chain
    /// of extreme segments. Direct implementation of the definition, O(n^3).
    /// </summary>
    public class ExtremeSegments : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            outPoints = new List<Point>();

            if (points == null || points.Count == 0)
            {
                return;
            }

            // Point equality is tolerance based but Point has no matching GetHashCode,
            // so LINQ Distinct() keeps duplicates. HullSupport.Distinct really removes them.
            List<Point> uniquePoints = HullSupport.Distinct(points);

            if (uniquePoints.Count <= 2)
            {
                Publish(uniquePoints, ref outPoints, ref outLines);
                return;
            }

            // nextVertex[i] is the hull vertex that follows point i on the counter-clockwise
            // hull, or -1 when point i is not a hull vertex.
            int[] nextVertex = new int[uniquePoints.Count];
            for (int i = 0; i < nextVertex.Length; i++)
            {
                nextVertex[i] = -1;
            }

            for (int i = 0; i < uniquePoints.Count; i++)
            {
                for (int j = 0; j < uniquePoints.Count; j++)
                {
                    if (i != j && IsExtremeSegment(uniquePoints, i, j))
                    {
                        nextVertex[i] = j;
                        break;
                    }
                }
            }

            // The lowest then left most point is always a hull vertex, so the chain can be
            // started there and followed until it comes back to it. The distinct step left no
            // two points within the tolerance, so IndexOf cannot pick the wrong one.
            Point start = HullSupport.FindLowestThenLeftmost(uniquePoints);
            int startIndex = uniquePoints.IndexOf(start);
            int current = startIndex;

            List<Point> hull = new List<Point>(uniquePoints.Count);

            while (current != -1)
            {
                hull.Add(uniquePoints[current]);
                current = nextVertex[current];

                if (current == startIndex)
                {
                    break;
                }
            }

            Publish(hull, ref outPoints, ref outLines);
        }

        /// <summary>
        /// Checks whether the directed segment from one point to another is a maximal extreme
        /// segment, that is whether every other point lies to its left or on its line and no
        /// point lies on that line outside the segment.
        /// </summary>
        /// <param name="points">All the points.</param>
        /// <param name="first">The index of the start of the segment.</param>
        /// <param name="second">The index of the end of the segment.</param>
        /// <returns>True when the segment is an edge of the hull.</returns>
        private static bool IsExtremeSegment(List<Point> points, int first, int second)
        {
            for (int i = 0; i < points.Count; i++)
            {
                if (i == first || i == second)
                {
                    continue;
                }

                double side = HullSupport.Cross(points[first], points[second], points[i]);

                // A point on the right means the segment cuts through the point set.
                if (side < 0)
                {
                    return false;
                }

                // A point on the line but beyond one of the ends means this segment is only
                // a part of the real hull edge, so it is not an edge by itself.
                if (side == 0 && IsOutsideSegment(points[first], points[second], points[i]))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Checks whether a point that is already known to be collinear with the segment lies
        /// outside it, that is before its start or after its end.
        /// </summary>
        /// <param name="start">The start of the segment.</param>
        /// <param name="end">The end of the segment.</param>
        /// <param name="point">The point on the line of the segment.</param>
        /// <returns>True when the point is not between the two ends.</returns>
        private static bool IsOutsideSegment(Point start, Point end, Point point)
        {
            double dx = end.X - start.X;
            double dy = end.Y - start.Y;
            double projection = (point.X - start.X) * dx + (point.Y - start.Y) * dy;

            return projection < 0 || projection > dx * dx + dy * dy;
        }

        /// <summary>
        /// Copies the vertices to the output and builds the matching hull edges.
        /// </summary>
        /// <param name="vertices">The hull vertices, in counter-clockwise order.</param>
        /// <param name="outPoints">The hull vertices, in counter-clockwise order.</param>
        /// <param name="outLines">The edges of the hull.</param>
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
