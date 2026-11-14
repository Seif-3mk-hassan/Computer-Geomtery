using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class ExtremePoints : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            outPoints = new List<Point>();

            // A convex hull cannot be formed with fewer than 3 points
            if (points == null || points.Count <= 3)
            {
                if (points != null) outPoints.AddRange(points);
                return;
            }

            // Remove duplicates to prevent overlapping checks
            List<Point> uniquePoints = points.Distinct().ToList();

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
        }

        public override string ToString()
        {
            return "Convex Hull - Extreme Points";
        }
    }
}
