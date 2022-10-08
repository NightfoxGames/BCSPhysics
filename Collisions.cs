using System;

namespace BCSP
{
    public static class Collisions
    {
        public static void PointSegmentDistance(BasicVector p, BasicVector a, BasicVector b, out float distanceSquared, out BasicVector cp)
        {
            BasicVector ab = b - a;
            BasicVector ap = p - a;

            float proj = BasicMath.Dot(ap, ab);
            float abLenSq = BasicMath.LengthSquared(ab);
            float d = proj / abLenSq;

            if(d <= 0f)
            {
                cp = a;
            }
            else if(d >= 1f)
            {
                cp = b;
            }
            else
            {
                cp = a + ab * d;
            }

            distanceSquared = BasicMath.DistanceSquared(p, cp);
        }

        public static bool IntersectAABBs(AABB a, AABB b)
        {
            if(a.Max.X <= b.Min.X || b.Max.X <= a.Min.X ||
                a.Max.Y <= b.Min.Y || b.Max.Y <= a.Min.Y)
            {
                return false;
            }

            return true;
        }

        public static void FindContactPoints(
            RigidBody bodyA, RigidBody bodyB, 
            out BasicVector contact1, out BasicVector contact2, 
            out int contactCount)
        {
            contact1 = BasicVector.Zero;
            contact2 = BasicVector.Zero;
            contactCount = 0;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapeTypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    Collisions.FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(),
                        out contact1, out contact2, out contactCount);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collisions.FindCirclePolygonContactPoint(bodyB.Position, bodyB.Radius, bodyA.Position, bodyA.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
            }
            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    Collisions.FindCirclePolygonContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, bodyB.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collisions.FindCirclesContactPoint(bodyA.Position, bodyA.Radius, bodyB.Position, out contact1);
                    contactCount = 1;
                }
            }
        }

        private static void FindPolygonsContactPoints(
            BasicVector[] verticesA, BasicVector[] verticesB, 
            out BasicVector contact1, out BasicVector contact2, out int contactCount)
        {
            contact1 = BasicVector.Zero;
            contact2 = BasicVector.Zero;
            contactCount = 0;

            float minDistSq = float.MaxValue;

            for(int i = 0; i < verticesA.Length; i++)
            {
                BasicVector p = verticesA[i];

                for(int j = 0; j < verticesB.Length; j++)
                {
                    BasicVector va = verticesB[j];
                    BasicVector vb = verticesB[(j + 1) % verticesB.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out BasicVector cp);

                    if(BasicMath.NearlyEqual(distSq, minDistSq))
                    {
                        if (!BasicMath.NearlyEqual(cp, contact1))
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if(distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }
                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                BasicVector p = verticesB[i];

                for (int j = 0; j < verticesA.Length; j++)
                {
                    BasicVector va = verticesA[j];
                    BasicVector vb = verticesA[(j + 1) % verticesA.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out BasicVector cp);

                    if (BasicMath.NearlyEqual(distSq, minDistSq))
                    {
                        if (!BasicMath.NearlyEqual(cp, contact1))
                        {
                            contact2 = cp;
                            contactCount = 2;
                        }
                    }
                    else if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }
                }
            }
        }

        private static void FindCirclePolygonContactPoint(
            BasicVector circleCenter, float circleRadius, 
            BasicVector polygonCenter, BasicVector[] polygonVertices, 
            out BasicVector cp)
        {
            cp = BasicVector.Zero;

            float minDistSq = float.MaxValue;

            for(int i = 0; i < polygonVertices.Length; i++)
            {
                BasicVector va = polygonVertices[i];
                BasicVector vb = polygonVertices[(i + 1) % polygonVertices.Length];

                Collisions.PointSegmentDistance(circleCenter, va, vb, out float distSq, out BasicVector contact);

                if(distSq < minDistSq)
                {
                    minDistSq = distSq;
                    cp = contact;
                }
            }
        }

        private static void FindCirclesContactPoint(BasicVector centerA, float radiusA, BasicVector centerB, out BasicVector cp)
        {
            BasicVector ab = centerB - centerA;
            BasicVector dir = BasicMath.Normalize(ab);
            cp = centerA + dir * radiusA;
        }

        public static bool Collide(RigidBody bodyA, RigidBody bodyB, out BasicVector normal, out float depth)
        {
            normal = BasicVector.Zero;
            depth = 0f;

            ShapeType shapeTypeA = bodyA.ShapeType;
            ShapeType shapeTypeB = bodyB.ShapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    return Collisions.IntersectPolygons(
                        bodyA.Position, bodyA.GetTransformedVertices(),
                        bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    bool result = Collisions.IntersectCirclePolygon(
                        bodyB.Position, bodyB.Radius,
                        bodyA.Position, bodyA.GetTransformedVertices(),
                        out normal, out depth);

                    normal = -normal;
                    return result;
                }
            }
            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    return Collisions.IntersectCirclePolygon(
                        bodyA.Position, bodyA.Radius,
                        bodyB.Position, bodyB.GetTransformedVertices(),
                        out normal, out depth);
                }
                else if (shapeTypeB is ShapeType.Circle)
                {
                    return Collisions.IntersectCircles(
                        bodyA.Position, bodyA.Radius,
                        bodyB.Position, bodyB.Radius,
                        out normal, out depth);
                }
            }

            return false;
        }

        public static bool IntersectCirclePolygon(BasicVector circleCenter, float circleRadius,
                                                    BasicVector polygonCenter, BasicVector[] vertices,
                                                    out BasicVector normal, out float depth)
        {
            normal = BasicVector.Zero;
            depth = float.MaxValue;

            BasicVector axis = BasicVector.Zero;
            float axisDepth = 0f;
            float minA, maxA, minB, maxB;

            for (int i = 0; i < vertices.Length; i++)
            {
                BasicVector va = vertices[i];
                BasicVector vb = vertices[(i + 1) % vertices.Length];

                BasicVector edge = vb - va;
                axis = new BasicVector(-edge.Y, edge.X);
                axis = BasicMath.Normalize(axis);

                Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
                Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            int cpIndex = Collisions.FindClosestPointOnPolygon(circleCenter, vertices);
            BasicVector cp = vertices[cpIndex];

            axis = cp - circleCenter;
            axis = BasicMath.Normalize(axis);

            Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
            Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

            if (minA >= maxB || minB >= maxA)
            {
                return false;
            }

            axisDepth = MathF.Min(maxB - minA, maxA - minB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }

            BasicVector direction = polygonCenter - circleCenter;

            if (BasicMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }

        private static int FindClosestPointOnPolygon(BasicVector circleCenter, BasicVector[] vertices)
        {
            int result = -1;
            float minDistance = float.MaxValue;

            for(int i = 0; i < vertices.Length; i++)
            {
                BasicVector v = vertices[i];
                float distance = BasicMath.Distance(v, circleCenter);

                if(distance < minDistance)
                {
                    minDistance = distance;
                    result = i;
                }
            }

            return result;
        }

        private static void ProjectCircle(BasicVector center, float radius, BasicVector axis, out float min, out float max)
        {
            BasicVector direction = BasicMath.Normalize(axis);
            BasicVector directionAndRadius = direction * radius;

            BasicVector p1 = center + directionAndRadius;
            BasicVector p2 = center - directionAndRadius;

            min = BasicMath.Dot(p1, axis);
            max = BasicMath.Dot(p2, axis);

            if(min > max)
            {
                // swap the min and max values.
                float t = min;
                min = max;
                max = t;
            }
        }

        public static bool IntersectPolygons(BasicVector centerA, BasicVector[] verticesA, BasicVector centerB, BasicVector[] verticesB, out BasicVector normal, out float depth)
        {
            normal = BasicVector.Zero;
            depth = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++)
            {
                BasicVector va = verticesA[i];
                BasicVector vb = verticesA[(i + 1) % verticesA.Length];

                BasicVector edge = vb - va;
                BasicVector axis = new BasicVector(-edge.Y, edge.X);
                axis = BasicMath.Normalize(axis);

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                BasicVector va = verticesB[i];
                BasicVector vb = verticesB[(i + 1) % verticesB.Length];

                BasicVector edge = vb - va;
                BasicVector axis = new BasicVector(-edge.Y, edge.X);
                axis = BasicMath.Normalize(axis);

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || minB >= maxA)
                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);

                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }
            }

            BasicVector direction = centerB - centerA;

            if (BasicMath.Dot(direction, normal) < 0f)
            {
                normal = -normal;
            }

            return true;
        }

        private static void ProjectVertices(BasicVector[] vertices, BasicVector axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            for(int i = 0; i < vertices.Length; i++)
            {
                BasicVector v = vertices[i];
                float proj = BasicMath.Dot(v, axis);

                if(proj < min) { min = proj; }
                if(proj > max) { max = proj; }
            }
        }

        public static bool IntersectCircles(
            BasicVector centerA, float radiusA, 
            BasicVector centerB, float radiusB, 
            out BasicVector normal, out float depth)
        {
            normal = BasicVector.Zero;
            depth = 0f;

            float distance = BasicMath.Distance(centerA, centerB);
            float radii = radiusA + radiusB;

            if(distance >= radii)
            {
                return false;
            }

            normal = BasicMath.Normalize(centerB - centerA);
            depth = radii - distance;

            return true;
        }

    }
}
