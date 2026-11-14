using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    /// <summary>
    /// Convex hull using the extreme points definition: a point belongs to the hull when it
    /// cannot be found inside (or on the edge of) any triangle built from the other points.
    /// Direct implementation of the definition, O(n^4).
    /// </summary>
    public class ExtremePoints : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            outPoints = new List<Point>();

            if (points == null || points.Count == 0)
            {
                return;
            }

            // Point equality is tolerance based but Point has no matching GetHashCode,
            // so LINQ Distinct() keeps duplicates. A duplicated vertex would make the point
            // look like it lies on the edge of a triangle and it would be dropped below.
            List<Point> uniquePoints = HullSupport.Distinct(points);

            // Fewer than three distinct points: the hull is the points themselves.
            if (uniquePoints.Count <= 2)
            {
                Publish(uniquePoints, ref outPoints, ref outLines);
                return;
            }

            // No triangle exists when every point is on one line, so the checks below
            // would reject every point. The hull of a segment is its two end points,
            // which are the first and the last point of the sorted set.
            if (IsCollinear(uniquePoints))
            {
                List<Point> ends = new List<Point> { uniquePoints[0], uniquePoints[uniquePoints.Count - 1] };
                Publish(ends, ref outPoints, ref outLines);
                return;
            }

            // Iterate through every point to check if it is extreme
            for (int i = 0; i < uniquePoints.Count; i++)
            {
                bool isExtreme = true;
                Point p = uniquePoints[i];

                // Check against every possible triplet (triangle) in the set
                for (int j = 0; j < uniquePoints.Count; j++)
                {
                    if (i == j) continue;

                    for (int k = j + 1; k < uniquePoints.Count; k++)
                    {
                        if (i == k) continue;

                        for (int m = k + 1; m < uniquePoints.Count; m++)
                        {
                            if (i == m) continue;

                            Point a = uniquePoints[j];
                            Point b = uniquePoints[k];
                            Point c = uniquePoints[m];

                            // Check if point p lies inside the triangle formed by a, b, and c
                            Enums.PointInPolygon location = HelperMethods.PointInTriangle(p, a, b, c);

                            // If p is inside or on the edge of the triangle, it is not an extreme point
                            if (location == Enums.PointInPolygon.Inside || location == Enums.PointInPolygon.OnEdge)
                            {
                                isExtreme = false;
                                break;
                            }
                        }
                        if (!isExtreme) break;
                    }
                    if (!isExtreme) break;
                }

                // If the point survived the checks, it is part of the convex hull
                if (isExtreme)
                {
                    outPoints.Add(p);
                }
            }

            // The points were collected in input order; order them around the hull so the
            // result can be drawn as a closed polygon.
            Point pivot = HullSupport.FindLowestThenLeftmost(outPoints);
            outPoints.Sort((a, b) => HullSupport.CompareByAngle(pivot, a, b));

            outLines = HullSupport.BuildEdges(outPoints);
        }

        /// <summary>
        /// Checks whether all the points lie on a single line.
        /// </summary>
        /// <param name="sortedPoints">The points, sorted by X then Y, without duplicates.</param>
        /// <returns>True when no triangle can be formed from the points.</returns>
        private static bool IsCollinear(List<Point> sortedPoints)
        {
            for (int i = 2; i < sortedPoints.Count; i++)
            {
                if (HullSupport.Cross(sortedPoints[0], sortedPoints[1], sortedPoints[i]) != 0)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Copies the vertices to the output and builds the matching hull edges.
        /// </summary>
        private static void Publish(List<Point> vertices, ref List<Point> outPoints, ref List<Line> outLines)
        {
            outPoints = vertices;
            outLines = HullSupport.BuildEdges(vertices);
        }

        public override string ToString()
        {
            return "Convex Hull - Extreme Points";
        }
    }
}
