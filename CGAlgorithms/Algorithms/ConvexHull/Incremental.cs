using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class Incremental : Algorithm
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
                outPoints = uniquePoints;
                outLines = HullSupport.BuildEdges(outPoints);
                return;
            }

            List<Point> hull = new List<Point>();

            // Initialization: Find the first 3 non-collinear points to form the initial triangle
            int p1Idx = 0, p2Idx = 1, p3Idx = 2;
            while (p3Idx < uniquePoints.Count &&
                   HelperMethods.CheckTurn(new Line(uniquePoints[p1Idx], uniquePoints[p2Idx]), uniquePoints[p3Idx]) == Enums.TurnType.Colinear)
            {
                p3Idx++;
            }

            // Fallback if all points in the set are collinear. Distinct() already sorted
            // them, so the two ends of the segment are the first and the last point.
            if (p3Idx == uniquePoints.Count)
            {
                outPoints.Add(uniquePoints[0]);
                outPoints.Add(uniquePoints[uniquePoints.Count - 1]);
                outLines = HullSupport.BuildEdges(outPoints);
                return;
            }

            // Ensure the initial triangle is oriented Counter-Clockwise (CCW)
            hull.Add(uniquePoints[p1Idx]);
            if (HelperMethods.CheckTurn(new Line(uniquePoints[p1Idx], uniquePoints[p2Idx]), uniquePoints[p3Idx]) == Enums.TurnType.Left)
            {
                hull.Add(uniquePoints[p2Idx]);
                hull.Add(uniquePoints[p3Idx]);
            }
            else
            {
                hull.Add(uniquePoints[p3Idx]);
                hull.Add(uniquePoints[p2Idx]);
            }

            bool[] used = new bool[uniquePoints.Count];
            used[p1Idx] = true;
            used[p2Idx] = true;
            used[p3Idx] = true;

            // Advance step: Process the remaining points one by one
            for (int i = 0; i < uniquePoints.Count; i++)
            {
                if (used[i]) continue;
                Point currentPoint = uniquePoints[i];

                List<int> visibleEdges = new List<int>();

                // A point is in the exterior if it is to the right of any directed edge in the CCW hull
                for (int j = 0; j < hull.Count; j++)
                {
                    int next = (j + 1) % hull.Count;
                    if (HelperMethods.CheckTurn(new Line(hull[j], hull[next]), currentPoint) == Enums.TurnType.Right)
                    {
                        visibleEdges.Add(j);
                    }
                }

                // If no edges are visible, the point lies in the interior of the polygon and is ignored
                if (visibleEdges.Count == 0) continue;

                // Find a non-visible edge to serve as a reliable starting anchor for finding tangents
                int nonVisibleIdx = -1;
                for (int j = 0; j < hull.Count; j++)
                {
                    if (!visibleEdges.Contains(j))
                    {
                        nonVisibleIdx = j;
                        break;
                    }
                }

                // Identify the boundary of the visible chain (the two tangent points)
                int firstVisible = -1;
                int lastVisible = -1;

                for (int step = 0; step < hull.Count; step++)
                {
                    int currEdge = (nonVisibleIdx + step) % hull.Count;
                    int nextEdge = (currEdge + 1) % hull.Count;

                    bool currentIsVisible = visibleEdges.Contains(currEdge);
                    bool nextIsVisible = visibleEdges.Contains(nextEdge);

                    if (!currentIsVisible && nextIsVisible) firstVisible = nextEdge;
                    if (currentIsVisible && !nextIsVisible) lastVisible = currEdge;
                }

                // Replace the visible chain in the hull with the new point
                List<Point> updatedHull = new List<Point>();

                // The right tangent is the end of the last visible edge
                int rightTangentIdx = (lastVisible + 1) % hull.Count;

                int index = rightTangentIdx;
                while (index != firstVisible)
                {
                    updatedHull.Add(hull[index]);
                    index = (index + 1) % hull.Count;
                }

                // Add the left tangent and the new point
                updatedHull.Add(hull[firstVisible]);
                updatedHull.Add(currentPoint);

                hull = updatedHull;

                // The insertion can leave a vertex exactly on one of the new edges
                // (a point that is collinear with its two neighbours). Such a point is
                // not a hull vertex, so drop it before the next point is handled.
                HullSupport.RemoveCollinearVertices(hull);
            }

            outPoints = hull;
            outLines = HullSupport.BuildEdges(hull);
        }

        public override string ToString()
        {
            return "Convex Hull - Incremental";
        }
    }
}
