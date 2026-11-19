using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Convex hull using divide and conquer.
    /// The points are split by X coordinate, the hull of each half is computed recursively and
    /// the two hulls are glued together with their common upper and lower tangents.
    /// Runs in O(n log n).
    /// </summary>
    public class DivideAndConquer : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            outPoints = new List<Point>();

            if (points == null || points.Count == 0)
            {
                return;
            }

            // Distinct() also sorts the points by X then Y, which is the order the
            // recursion relies on to split the points into a left and a right half.
            List<Point> sorted = HullSupport.Distinct(points);
            List<Point> hull = BuildHull(sorted, 0, sorted.Count - 1);

            outPoints = hull;
            outLines = HullSupport.BuildEdges(hull);
        }

        /// <summary>
        /// Computes the hull of a range of the sorted array.
        /// </summary>
        /// <param name="points">The points, sorted by X then Y.</param>
        /// <param name="start">The first index of the range.</param>
        /// <param name="end">The last index of the range.</param>
        /// <returns>The hull vertices in counter-clockwise order.</returns>
        private static List<Point> BuildHull(List<Point> points, int start, int end)
        {
            int count = end - start + 1;

            // The base case is small enough to be solved directly.
            if (count <= 3)
            {
                return SmallHull(points, start, count);
            }

            int middle = start + count / 2;

            List<Point> leftHull = BuildHull(points, start, middle - 1);
            List<Point> rightHull = BuildHull(points, middle, end);

            return Merge(leftHull, rightHull);
        }

        /// <summary>
        /// Computes the hull of at most three points directly.
        /// </summary>
        /// <param name="points">The points, sorted by X then Y.</param>
        /// <param name="start">The first index of the range.</param>
        /// <param name="count">The number of points in the range.</param>
        /// <returns>The hull vertices in counter-clockwise order, without collinear points.</returns>
        private static List<Point> SmallHull(List<Point> points, int start, int count)
        {
            List<Point> hull = new List<Point>(3);

            if (count <= 0)
            {
                return hull;
            }

            if (count == 1)
            {
                hull.Add(points[start]);
                return hull;
            }

            if (count == 2)
            {
                hull.Add(points[start]);
                hull.Add(points[start + 1]);
                return hull;
            }

            Point a = points[start];
            Point b = points[start + 1];
            Point c = points[start + 2];
            double turn = HullSupport.Cross(a, b, c);

            if (turn == 0)
            {
                // Collinear points: only the two end points of the segment are hull vertices,
                // and the sorted order puts them first and last.
                hull.Add(a);
                hull.Add(c);
            }
            else if (turn > 0)
            {
                hull.Add(a);
                hull.Add(b);
                hull.Add(c);
            }
            else
            {
                hull.Add(a);
                hull.Add(c);
                hull.Add(b);
            }

            return hull;
        }

        /// <summary>
        /// Merges the hulls of the left and the right half of the points.
        /// Both hulls are counter-clockwise and every point of the left hull is at or left of
        /// every point of the right hull, so the merged hull is the left hull from the upper
        /// tangent point to the lower one, followed by the right hull from the lower tangent
        /// point to the upper one.
        /// </summary>
        /// <param name="leftHull">The hull of the left half, counter-clockwise.</param>
        /// <param name="rightHull">The hull of the right half, counter-clockwise.</param>
        /// <returns>The merged hull vertices in counter-clockwise order.</returns>
        private static List<Point> Merge(List<Point> leftHull, List<Point> rightHull)
        {
            // A hull with fewer than three vertices is a point or a segment and has no tangents.
            if (leftHull.Count < 3 || rightHull.Count < 3)
            {
                return MergeWithMonotoneChain(leftHull, rightHull);
            }

            int upperLeft, upperRight, lowerLeft, lowerRight;
            FindTangents(leftHull, rightHull, out upperLeft, out upperRight, out lowerLeft, out lowerRight);

            List<Point> merged = new List<Point>(leftHull.Count + rightHull.Count);

            for (int i = upperLeft; ; i = (i + 1) % leftHull.Count)
            {
                merged.Add(leftHull[i]);
                if (i == lowerLeft)
                {
                    break;
                }
            }

            for (int i = lowerRight; ; i = (i + 1) % rightHull.Count)
            {
                merged.Add(rightHull[i]);
                if (i == upperRight)
                {
                    break;
                }
            }

            return merged;
        }

        /// <summary>
        /// Finds the four points where the common upper and lower tangents touch the two hulls.
        /// </summary>
        private static void FindTangents(List<Point> leftHull, List<Point> rightHull,
            out int upperLeft, out int upperRight, out int lowerLeft, out int lowerRight)
        {
            upperLeft = FindExtremeIndex(leftHull, true, true);
            upperRight = FindExtremeIndex(rightHull, false, true);
            WalkToTangent(leftHull, rightHull, true, ref upperLeft, ref upperRight);

            lowerLeft = FindExtremeIndex(leftHull, true, false);
            lowerRight = FindExtremeIndex(rightHull, false, false);
            WalkToTangent(leftHull, rightHull, false, ref lowerLeft, ref lowerRight);
        }

        /// <summary>
        /// Rotates the two candidate points of a tangent until the line through them touches
        /// both hulls, that is until neither hull has a vertex above (or below) the line.
        /// </summary>
        /// <param name="leftHull">The left hull, counter-clockwise.</param>
        /// <param name="rightHull">The right hull, counter-clockwise.</param>
        /// <param name="upper">True for the upper tangent, false for the lower one.</param>
        /// <param name="leftIndex">The tangent point on the left hull.</param>
        /// <param name="rightIndex">The tangent point on the right hull.</param>
        private static void WalkToTangent(List<Point> leftHull, List<Point> rightHull, bool upper,
            ref int leftIndex, ref int rightIndex)
        {
            int guard = 2 * (leftHull.Count + rightHull.Count) + 4;
            bool moved = true;

            while (moved && guard-- > 0)
            {
                moved = false;

                if (upper)
                {
                    // The upper tangent keeps both hulls below the line: rotate the left
                    // hull counter-clockwise and the right hull clockwise while the
                    // candidate vertex is above the current line. Collinear vertices are
                    // rotated through as well, otherwise the walk stops at the first of
                    // several vertices that all touch the tangent line.
                    int nextLeft = (leftIndex + 1) % leftHull.Count;
                    if (HullSupport.Cross(leftHull[leftIndex], rightHull[rightIndex], leftHull[nextLeft]) >= 0)
                    {
                        leftIndex = nextLeft;
                        moved = true;
                    }

                    int previousRight = (rightIndex - 1 + rightHull.Count) % rightHull.Count;
                    if (HullSupport.Cross(leftHull[leftIndex], rightHull[rightIndex], rightHull[previousRight]) >= 0)
                    {
                        rightIndex = previousRight;
                        moved = true;
                    }
                }
                else
                {
                    // The lower tangent is the mirrored case: rotate the left hull
                    // clockwise and the right hull counter-clockwise while the candidate
                    // vertex is below the current line. Collinear vertices are rotated
                    // through as well.
                    int previousLeft = (leftIndex - 1 + leftHull.Count) % leftHull.Count;
                    if (HullSupport.Cross(leftHull[leftIndex], rightHull[rightIndex], leftHull[previousLeft]) <= 0)
                    {
                        leftIndex = previousLeft;
                        moved = true;
                    }

                    int nextRight = (rightIndex + 1) % rightHull.Count;
                    if (HullSupport.Cross(leftHull[leftIndex], rightHull[rightIndex], rightHull[nextRight]) <= 0)
                    {
                        rightIndex = nextRight;
                        moved = true;
                    }
                }
            }
        }

        /// <summary>
        /// Finds the index of the right most or left most vertex of a hull.
        /// Ties are broken by taking the highest (or lowest) vertex, which is what the
        /// tangent walk needs as a starting point.
        /// </summary>
        /// <param name="hull">The hull to search.</param>
        /// <param name="rightmost">True to look for the right most vertex, false for the left most.</param>
        /// <param name="upper">True to prefer the highest vertex on a tie, false for the lowest.</param>
        private static int FindExtremeIndex(List<Point> hull, bool rightmost, bool upper)
        {
            int best = 0;

            for (int i = 1; i < hull.Count; i++)
            {
                bool betterX = rightmost ? hull[i].X > hull[best].X : hull[i].X < hull[best].X;
                bool sameX = hull[i].X == hull[best].X;
                bool betterY = upper ? hull[i].Y > hull[best].Y : hull[i].Y < hull[best].Y;

                if (betterX || (sameX && betterY))
                {
                    best = i;
                }
            }

            return best;
        }

        /// <summary>
        /// Merges two hulls by running a monotone chain over their vertices.
        /// Used for the degenerate halves whose hull is only a point or a segment,
        /// where tangent walking does not apply.
        /// </summary>
        private static List<Point> MergeWithMonotoneChain(List<Point> leftHull, List<Point> rightHull)
        {
            List<Point> vertices = new List<Point>(leftHull.Count + rightHull.Count);
            vertices.AddRange(leftHull);
            vertices.AddRange(rightHull);
            vertices.Sort(CompareByXThenY);

            return MonotoneChain(vertices);
        }

        /// <summary>
        /// Andrew's monotone chain: builds the hull of points that are already sorted by X then Y.
        /// </summary>
        private static List<Point> MonotoneChain(List<Point> sorted)
        {
            if (sorted.Count <= 2)
            {
                return sorted;
            }

            List<Point> lower = new List<Point>();
            foreach (Point point in sorted)
            {
                while (lower.Count >= 2 && HullSupport.Cross(lower[lower.Count - 2], lower[lower.Count - 1], point) <= 0)
                {
                    lower.RemoveAt(lower.Count - 1);
                }

                lower.Add(point);
            }

            List<Point> upper = new List<Point>();
            for (int i = sorted.Count - 1; i >= 0; i--)
            {
                Point point = sorted[i];
                while (upper.Count >= 2 && HullSupport.Cross(upper[upper.Count - 2], upper[upper.Count - 1], point) <= 0)
                {
                    upper.RemoveAt(upper.Count - 1);
                }

                upper.Add(point);
            }

            // The last point of each chain is the first point of the other one.
            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);
            lower.AddRange(upper);

            return lower;
        }

        /// <summary>
        /// Compares two points by X coordinate, then by Y coordinate.
        /// </summary>
        private static int CompareByXThenY(Point a, Point b)
        {
            int result = a.X.CompareTo(b.X);

            return result != 0 ? result : a.Y.CompareTo(b.Y);
        }

        public override string ToString()
        {
            return "Convex Hull - Divide & Conquer";
        }

    }
}
