using CGUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    
    public class DivideAndConquer : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            List<Point> hull = BuildHull(HullSupport.Distinct(points));

            outPoints = hull;
            outLines = HullSupport.BuildEdges(hull);
        }

       
        private static List<Point> BuildHull(List<Point> points)
        {
        
            if (points.Count <= 3)
            {
                return SmallHull(points);
            }

            if (IsCollinear(points))
            {
                return new List<Point> { points[0], points[points.Count - 1] };
            }

            int middle = points.Count / 2;
            List<Point> leftPoints = points.GetRange(0, middle);
            List<Point> rightPoints = points.GetRange(middle, points.Count - middle);

            List<Point> leftHull = BuildHull(leftPoints);
            List<Point> rightHull = BuildHull(rightPoints);

            return Merge(leftHull, rightHull);
        }

      
        private static bool IsCollinear(List<Point> points)
        {
            for (int i = 2; i < points.Count; i++)
            {
                if (HullSupport.Cross(points[0], points[1], points[i]) != 0)
                {
                    return false;
                }
            }

            return true;
        }

    
        private static List<Point> SmallHull(List<Point> points)
        {
  
            if (points.Count < 3)
            {
                return new List<Point>(points);
            }

            Point first = points[0];
            Point middle = points[1];
            Point last = points[2];

            double turn = HullSupport.Cross(first, middle, last);
            if (turn > 0)
            {
                return new List<Point> { first, middle, last };
            }

            if (turn < 0)
            {
                return new List<Point> { first, last, middle };
            }


            return new List<Point> { first, last };
        }

 
        private static List<Point> Merge(List<Point> left, List<Point> right)
        {
            int upperLeft, upperRight;
            FindTangent(left, right, true, out upperLeft, out upperRight);

            int lowerLeft, lowerRight;
            FindTangent(left, right, false, out lowerLeft, out lowerRight);

            List<Point> merged = new List<Point>();


            int index = upperLeft;
            while (true)
            {
                merged.Add(left[index]);
                if (index == lowerLeft)
                {
                    break;
                }

                index = (index + 1) % left.Count;
            }

            index = lowerRight;
            while (true)
            {
                merged.Add(right[index]);
                if (index == upperRight)
                {
                    break;
                }

                index = (index + 1) % right.Count;
            }

            return RemoveCollinear(merged);
        }

 
        private static List<Point> RemoveCollinear(List<Point> hull)
        {
            if (hull.Count <= 2)
            {
                return hull;
            }

            List<Point> result = new List<Point>(hull.Count);
            foreach (Point point in hull)
            {
                while (result.Count >= 2 && HullSupport.Cross(
                    result[result.Count - 2], result[result.Count - 1], point) == 0)
                {
                    result.RemoveAt(result.Count - 1);
                }

                result.Add(point);
            }

            while (result.Count > 2)
            {
                if (HullSupport.Cross(result[result.Count - 2], result[result.Count - 1], result[0]) == 0)
                {
                    result.RemoveAt(result.Count - 1);
                    continue;
                }

                if (HullSupport.Cross(result[result.Count - 1], result[0], result[1]) == 0)
                {
                    result.RemoveAt(0);
                    continue;
                }

                break;
            }

            return result;
        }

 
        private static void FindTangent(List<Point> left, List<Point> right, bool upper,
            out int leftIndex, out int rightIndex)
        {
            leftIndex = upper ? FindRightmost(left, true) : FindRightmost(left, false);
            rightIndex = upper ? FindLeftmost(right, true) : FindLeftmost(right, false);


            int leftStep = upper ? 1 : -1;
            int rightStep = upper ? -1 : 1;


            int limit = left.Count + right.Count;
            for (int step = 0; step < limit; step++)
            {
                bool moved = false;

                while (ShouldMove(left[leftIndex], right[rightIndex],
                    left[(leftIndex + leftStep + left.Count) % left.Count], upper))
                {
                    leftIndex = (leftIndex + leftStep + left.Count) % left.Count;
                    moved = true;
                }

                while (ShouldMove(left[leftIndex], right[rightIndex],
                    right[(rightIndex + rightStep + right.Count) % right.Count], upper))
                {
                    rightIndex = (rightIndex + rightStep + right.Count) % right.Count;
                    moved = true;
                }

                if (!moved)
                {
                    return;
                }
            }
        }


        private static bool ShouldMove(Point from, Point to, Point candidate, bool upper)
        {
            double turn = HullSupport.Cross(from, to, candidate);

            return upper ? turn > 0 : turn < 0;
        }

        private static int FindRightmost(List<Point> hull, bool highest)
        {
            int best = 0;
            for (int i = 1; i < hull.Count; i++)
            {
                Point candidate = hull[i];
                Point current = hull[best];

                if (candidate.X > current.X ||
                    (candidate.X == current.X &&
                     (highest ? candidate.Y > current.Y : candidate.Y < current.Y)))
                {
                    best = i;
                }
            }

            return best;
        }


        private static int FindLeftmost(List<Point> hull, bool highest)
        {
            int best = 0;
            for (int i = 1; i < hull.Count; i++)
            {
                Point candidate = hull[i];
                Point current = hull[best];

                if (candidate.X < current.X ||
                    (candidate.X == current.X &&
                     (highest ? candidate.Y > current.Y : candidate.Y < current.Y)))
                {
                    best = i;
                }
            }

            return best;
        }

        public override string ToString()
        {
            return "Convex Hull - Divide & Conquer";
        }
    }
}
