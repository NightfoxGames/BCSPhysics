using System;

namespace BCSP
{
    public enum ShapeType
    {
        Circle = 0,
        Box = 1
    }

    public sealed class RigidBody
    {
        private BasicVector position;
        private BasicVector linearVelocity;
        private float angle;
        private float angularVelocity;
        private BasicVector force;

        public readonly ShapeType ShapeType;
        public readonly float Density;
        public readonly float Mass;
        public readonly float InvMass;
        public readonly float Restitution;
        public readonly float Area;
        public readonly float Inertia;
        public readonly float InvInertia;
        public readonly bool IsStatic;
        public readonly float Radius;
        public readonly float Width;
        public readonly float Height;

        private readonly BasicVector[] vertices;
        private BasicVector[] transformedVertices;
        private AABB aabb;

        private bool transformUpdateRequired;
        private bool aabbUpdateRequired;


        public BasicVector Position
        {
            get { return this.position; }
        }

        public BasicVector LinearVelocity
        {
            get { return this.linearVelocity; }
            internal set { this.linearVelocity = value; }
        }

        public float Angle
        {
            get { return this.angle; }
        }

        public float AngularVelocity
        {
            get { return this.angularVelocity; }
            internal set { this.angularVelocity = value; }
        }
        

        private RigidBody(float density, float mass, float inertia, float restitution, float area,
            bool isStatic, float radius, float width, float height, BasicVector[] vertices, ShapeType shapeType)
        {
            this.position = BasicVector.Zero;
            this.linearVelocity = BasicVector.Zero;
            this.angle = 0f;
            this.angularVelocity = 0f;
            this.force = BasicVector.Zero;

            this.ShapeType = shapeType;
            this.Density = density;
            this.Mass = mass;
            this.InvMass = mass > 0f ? 1f / mass : 0f;
            this.Inertia = inertia;
            this.InvInertia = inertia > 0f ? 1f / inertia : 0f;
            this.Restitution = restitution;
            this.Area = area;
            this.IsStatic = isStatic;
            this.Radius = radius;
            this.Width = width;
            this.Height = height;

            if (this.ShapeType is ShapeType.Box)
            {
                this.vertices = vertices;
                this.transformedVertices = new BasicVector[this.vertices.Length];
            }
            else
            {
                this.vertices = null;
                this.transformedVertices = null;
            }

            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        private static BasicVector[] CreateBoxVertices(float width, float height)
        {
            float left = -width / 2f;
            float right = left + width;
            float bottom = -height / 2f;
            float top = bottom + height;

            BasicVector[] vertices = new BasicVector[4];
            vertices[0] = new BasicVector(left, top);
            vertices[1] = new BasicVector(right, top);
            vertices[2] = new BasicVector(right, bottom);
            vertices[3] = new BasicVector(left, bottom);

            return vertices;
        }

        private static int[] CreateBoxTriangles()
        {
            int[] triangles = new int[6];
            triangles[0] = 0;
            triangles[1] = 1;
            triangles[2] = 2;
            triangles[3] = 0;
            triangles[4] = 2;
            triangles[5] = 3;
            return triangles;
        }

        public BasicVector[] GetTransformedVertices()
        {
            if (this.transformUpdateRequired)
            {
                BasicTransform transform = new BasicTransform(this.position, this.angle);

                for (int i = 0; i < this.vertices.Length; i++)
                {
                    BasicVector v = this.vertices[i];
                    this.transformedVertices[i] = BasicVector.Transform(v, transform);
                }

                PhysicsWorld.TransformCount++;
            }
            else
            {
                PhysicsWorld.NoTransformCount++;
            }

            this.transformUpdateRequired = false;
            return this.transformedVertices;
        }

        public AABB GetAABB()
        {
            if (this.aabbUpdateRequired)
            {
                float minX = float.MaxValue;
                float minY = float.MaxValue;
                float maxX = float.MinValue;
                float maxY = float.MinValue;

                if (this.ShapeType is ShapeType.Box)
                {
                    BasicVector[] vertices = this.GetTransformedVertices();

                    for (int i = 0; i < vertices.Length; i++)
                    {
                        BasicVector v = vertices[i];

                        if (v.X < minX) { minX = v.X; }
                        if (v.X > maxX) { maxX = v.X; }
                        if (v.Y < minY) { minY = v.Y; }
                        if (v.Y > maxY) { maxY = v.Y; }
                    }
                }
                else if (this.ShapeType is ShapeType.Circle)
                {
                    minX = this.position.X - this.Radius;
                    minY = this.position.Y - this.Radius;
                    maxX = this.position.X + this.Radius;
                    maxY = this.position.Y + this.Radius;
                }
                else
                {
                    throw new Exception("Unknown ShapeType.");
                }

                this.aabb = new AABB(minX, minY, maxX, maxY);
            }

            this.aabbUpdateRequired = false;
            return this.aabb;
        }

        internal void Step(float time, BasicVector gravity, int iterations)
        {
            if (this.IsStatic)
            {
                return;
            }

            time /= (float)iterations;

            // force = mass * acc
            // acc = force / mass;

            BasicVector acceleration = this.force / this.Mass;
            this.linearVelocity += acceleration * time;


            this.linearVelocity += gravity * time;
            this.position += this.linearVelocity * time;

            this.angle += this.angularVelocity * time;

            this.force = BasicVector.Zero;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void Move(BasicVector amount)
        {
            this.position += amount;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void MoveTo(BasicVector position)
        {
            this.position = position;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void Rotate(float amount)
        {
            this.angle += amount;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void RotateTo(float angle)
        {
            this.angle = angle;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void AddForce(BasicVector amount)
        {
            this.force = amount;
        }
        public void SetVelociy(BasicVector amount)
        {
            this.linearVelocity = amount;
        }

        public static bool CreateCircleBody(float radius, float density, bool isStatic, float restitution, out RigidBody body, out string errorMessage)
        {
            body = null;
            errorMessage = string.Empty;

            float area = radius * radius * MathF.PI;

            if (area < PhysicsWorld.MinBodySize)
            {
                errorMessage = $"Circle radius is too small. Min circle area is {PhysicsWorld.MinBodySize}.";
                return false;
            }

            if (area > PhysicsWorld.MaxBodySize)
            {
                errorMessage = $"Circle radius is too large. Max circle area is {PhysicsWorld.MaxBodySize}.";
                return false;
            }

            if (density < PhysicsWorld.MinDensity)
            {
                errorMessage = $"Density is too small. Min density is {PhysicsWorld.MinDensity}";
                return false;
            }

            if (density > PhysicsWorld.MaxDensity)
            {
                errorMessage = $"Density is too large. Max density is {PhysicsWorld.MaxDensity}";
                return false;
            }

            restitution = BasicMath.Clamp(restitution, 0f, 1f);

            float mass = 0f;
            float inertia = 0f;

            if (!isStatic)
            {
                // mass = area * depth * density
                mass = area * density;
                inertia = (1f / 2f) * mass * radius * radius;
            }

            body = new RigidBody(density, mass, inertia, restitution, area, isStatic, radius, 0f, 0f, null, ShapeType.Circle);
            return true;
        }

        public static bool CreateBoxBody(float width, float height, float density, bool isStatic, float restitution, out RigidBody body, out string errorMessage)
        {
            body = null;
            errorMessage = string.Empty;

            float area = width * height;

            if (area < PhysicsWorld.MinBodySize)
            {
                errorMessage = $"Area is too small. Min area is {PhysicsWorld.MinBodySize}.";
                return false;
            }

            if (area > PhysicsWorld.MaxBodySize)
            {
                errorMessage = $"Area is too large. Max area is {PhysicsWorld.MaxBodySize}.";
                return false;
            }

            if (density < PhysicsWorld.MinDensity)
            {
                errorMessage = $"Density is too small. Min density is {PhysicsWorld.MinDensity}";
                return false;
            }

            if (density > PhysicsWorld.MaxDensity)
            {
                errorMessage = $"Density is too large. Max density is {PhysicsWorld.MaxDensity}";
                return false;
            }

            restitution = BasicMath.Clamp(restitution, 0f, 1f);

            float mass = 0f;
            float inertia = 0f;

            if (!isStatic)
            {
                // mass = area * depth * density
                mass = area * density;
                inertia = (1f / 12) * mass * (width * width + height * height);
            }

            BasicVector[] vertices = RigidBody.CreateBoxVertices(width, height);

            body = new RigidBody(density, mass, inertia, restitution, area, isStatic, 0f, width, height, vertices, ShapeType.Box);
            return true;
        }
    }
}
