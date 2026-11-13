using System;
using System.Collections.Generic;
using CGUtilities;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Geometry helpers shared by the convex hull algorithms.
    /// Kept in one place so every hull algorithm uses the same orientation,
    /// duplicate removal and result-building rules.
    /// </summary>
    internal static class HullSupport
    {
        /// <summary>
        /// Signed area of the triangle (origin, a, b), doubled.
        /// The value is positive when origin -> a -> b is a counter-clockwise (left) turn,
        /// negative for a clockwise (right) turn and zero when the three points are collinear.
        /// </summary>
        /// <param name="origin">The point the two vectors are taken from.</param>
        /// <param name="a">The first vector end point.</param>
        /// <param name="b">The second vector end point.</param>
        /// <returns>The signed doubled area of the triangle.</returns>
        public static double Cross(Point origin, Point a, Point b)
        {
            double ax = a.X - origin.X;
            double ay = a.Y - origin.Y;
            double bx = b.X - origin.X;
            double by = b.Y - origin.Y;

            return ax * by - ay * bx;
        }

        /// <summary>
        /// Squared distance between two points.
        /// Used instead of the distance to avoid an unnecessary square root (and the rounding it adds).
        /// </summary>
        /// <param name="a">The first point.</param>
        /// <param name="b">The second point.</param>
        /// <returns>The squared distance between a and b.</returns>
        public static double SquaredDistance(Point a, Point b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;

            return dx * dx + dy * dy;
        }

        /// <summary>
        /// Removes duplicated points (points equal within <see cref="Constants.Epsilon"/>).
        /// A duplicated input point would otherwise produce zero-length hull edges
        /// and could make gift wrapping loop forever.
        /// </summary>
        /// <param name="points">The points to filter.</param>
        /// <returns>A new list containing every distinct point, sorted by X then Y.</returns>
        public static List<Point> Distinct(List<Point> points)
        {
            List<Point> sorted = new List<Point>(points);
            sorted.Sort(CompareByXThenY);

            List<Point> unique = new List<Point>(sorted.Count);
            for (int i = 0; i < sorted.Count; i++)
            {
                Point candidate = sorted[i];

                // Two points are equal only if their X values are within Epsilon,
                // so only the already accepted points inside that X window can match.
                bool alreadySeen = false;
                for (int j = unique.Count - 1; j >= 0 && candidate.X - unique[j].X <= Constants.Epsilon; j--)
                {
                    if (unique[j].Equals(candidate))
                    {
                        alreadySeen = true;
                        break;
                    }
                }

                if (!alreadySeen)
                {
                    unique.Add(candidate);
                }
            }

            return unique;
        }

        /// <summary>
        /// Finds the lowest point of the set (the one with the smallest Y coordinate).
        /// Ties are broken by taking the left most point, which makes the result independent of input order.
        /// This point is always a vertex of the convex hull.
        /// </summary>
        /// <param name="points">A non-empty list of points.</param>
        /// <returns>The lowest, then left most point.</returns>
        public static Point FindLowestThenLeftmost(List<Point> points)
        {
            Point lowest = points[0];
            for (int i = 1; i < points.Count; i++)
            {
                Point candidate = points[i];
                if (candidate.Y < lowest.Y || (candidate.Y == lowest.Y && candidate.X < lowest.X))
                {
                    lowest = candidate;
                }
            }

            return lowest;
        }

        /// <summary>
        /// Orders two points by their polar angle around a pivot.
        /// The pivot must be the lowest point of the set, so every angle is in [0, PI)
        /// and the two points can be compared with a single cross product.
        /// Points on the same ray are ordered by increasing distance from the pivot.
        /// </summary>
        /// <param name="pivot">The point to measure the angles from.</param>
        /// <param name="a">The first point to compare.</param>
        /// <param name="b">The second point to compare.</param>
        /// <returns>A negative value when a comes first, a positive value when b comes first.</returns>
        public static int CompareByAngle(Point pivot, Point a, Point b)
        {
            double turn = Cross(pivot, a, b);
            if (turn > 0)
            {
                // b is counter-clockwise from a, so a has the smaller angle.
                return -1;
            }

            if (turn < 0)
            {
                return 1;
            }

            return SquaredDistance(pivot, a).CompareTo(SquaredDistance(pivot, b));
        }

        /// <summary>
        /// Builds the closed chain of lines that outlines the given hull vertices.
        /// </summary>
        /// <param name="vertices">The hull vertices, in order around the hull.</param>
        /// <returns>The edges of the hull. Empty for a single point, one segment for two points.</returns>
        public static List<Line> BuildEdges(List<Point> vertices)
        {
            List<Line> edges = new List<Line>();

            if (vertices.Count < 2)
            {
                return edges;
            }

            if (vertices.Count == 2)
            {
                edges.Add(new Line(vertices[0], vertices[1]));
                return edges;
            }

            for (int i = 0; i < vertices.Count; i++)
            {
                edges.Add(new Line(vertices[i], vertices[(i + 1) % vertices.Count]));
            }

            return edges;
        }

        /// <summary>
        /// Compares two points by X coordinate, then by Y coordinate.
        /// </summary>
        private static int CompareByXThenY(Point a, Point b)
        {
            int result = a.X.CompareTo(b.X);

            return result != 0 ? result : a.Y.CompareTo(b.Y);
        }
    }
}
