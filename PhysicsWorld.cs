using BCSL.Graphics;
using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;

namespace BCSP
{
    public sealed class PhysicsWorld
    {
        public static int TransformCount = 0;
        public static int NoTransformCount = 0;

        public static readonly float MinBodySize = 0.01f * 0.01f;
        public static readonly float MaxBodySize = 64f * 64f;

        public static readonly float MinDensity = 0.5f;     // g/cm^3
        public static readonly float MaxDensity = 21.4f;

        public static readonly int MinIterations = 1;
        public static readonly int MaxIterations = 128;

        private BasicVector gravity;
        private List<RigidBody> bodyList;
        private List<(int, int)> contactPairs;

        private BasicVector[] contactList;
        private BasicVector[] impulseList;
        private BasicVector[] raList;
        private BasicVector[] rbList;

        public int BodyCount
        {
            get { return this.bodyList.Count; }
        }

        public PhysicsWorld()
        {
            this.gravity = new BasicVector(0f, -9.81f);
            this.bodyList = new List<RigidBody>();
            this.contactPairs = new List<(int, int)>();

            this.contactList = new BasicVector[2];
            this.impulseList = new BasicVector[2];
            this.raList = new BasicVector[2];
            this.rbList = new BasicVector[2];
        }

        public void AddBody(RigidBody body)
        {
            this.bodyList.Add(body);
        }

        public bool RemoveBody(RigidBody body)
        {
            return this.bodyList.Remove(body);
        }

        public bool GetBody(int index, out RigidBody body)
        {
            body = null;

            if (index < 0 || index >= this.bodyList.Count)
            {
                return false;
            }

            body = this.bodyList[index];
            return true;
        }

        public void Step(float time, int totalIterations)
        {
            totalIterations = BasicMath.Clamp(totalIterations, PhysicsWorld.MinIterations, PhysicsWorld.MaxIterations);

            for (int currentIteration = 0; currentIteration < totalIterations; currentIteration++)
            {
                this.contactPairs.Clear();
                this.StepBodies(time, totalIterations);
                this.BroadPhase();
                this.NarrowPhase();
            }
        }

        private void BroadPhase()
        {
            for (int i = 0; i < this.bodyList.Count - 1; i++)
            {
                RigidBody bodyA = this.bodyList[i];
                AABB bodyA_aabb = bodyA.GetAABB();

                for (int j = i + 1; j < this.bodyList.Count; j++)
                {
                    RigidBody bodyB = this.bodyList[j];
                    AABB bodyB_aabb = bodyB.GetAABB();

                    if (bodyA.IsStatic && bodyB.IsStatic)
                    {
                        continue;
                    }

                    if (!Collisions.IntersectAABBs(bodyA_aabb, bodyB_aabb))
                    {
                        continue;
                    }

                    this.contactPairs.Add((i, j));
                }
            }
        }

        private void NarrowPhase()
        {
            for (int i = 0; i < this.contactPairs.Count; i++)
            {
                (int, int) pair = this.contactPairs[i];
                RigidBody bodyA = this.bodyList[pair.Item1];
                RigidBody bodyB = this.bodyList[pair.Item2];

                if (Collisions.Collide(bodyA, bodyB, out BasicVector normal, out float depth))
                {
                    this.SeparateBodies(bodyA, bodyB, normal * depth);
                    Collisions.FindContactPoints(bodyA, bodyB, out BasicVector contact1, out BasicVector contact2, out int contactCount);
                    BasicManifold contact = new BasicManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
                    this.ResolveCollisionWithRotation(in contact);
                }

            }
        }

        /// <summary>
        /// Changes The Gravity, Default Gravity = 0f, -9.81f
        /// </summary>
        /// <param name="newGravity"></param>
        /// <param name="errorMessage"></param>
        /// <returns></returns>
        public bool ChangeGravity(BasicVector newGravity, out string errorMessage)
        {
            errorMessage = "";
            if (gravity.X > 20 || gravity.X < -20 || gravity.Y > 20 || gravity.Y < -20)
            {
                errorMessage = "Not Able To Go That High";
                return false;
            }
            gravity = newGravity;
            return true;
        }


        public void StepBodies(float time, int totalIterations)
        {
            for (int i = 0; i < this.bodyList.Count; i++)
            {
                this.bodyList[i].Step(time, this.gravity, totalIterations);
            }
        }

        private void SeparateBodies(RigidBody bodyA, RigidBody bodyB, BasicVector mtv)
        {
            if (bodyA.IsStatic)
            {
                bodyB.Move(mtv);
            }
            else if (bodyB.IsStatic)
            {
                bodyA.Move(-mtv);
            }
            else
            {
                bodyA.Move(-mtv / 2f);
                bodyB.Move(mtv / 2f);
            }
        }

        public void ResolveCollisionBasic(in BasicManifold contact)
        {
            RigidBody bodyA = contact.BodyA;
            RigidBody bodyB = contact.BodyB;
            BasicVector normal = contact.Normal;
            float depth = contact.Depth;

            BasicVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

            if (BasicMath.Dot(relativeVelocity, normal) > 0f)
            {
                return;
            }

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            float j = -(1f + e) * BasicMath.Dot(relativeVelocity, normal);
            j /= bodyA.InvMass + bodyB.InvMass;

            BasicVector impulse = j * normal;

            bodyA.LinearVelocity -= impulse * bodyA.InvMass;
            bodyB.LinearVelocity += impulse * bodyB.InvMass;
        }

        public void ResolveCollisionWithRotation(in BasicManifold contact)
        {
            RigidBody bodyA = contact.BodyA;
            RigidBody bodyB = contact.BodyB;
            BasicVector normal = contact.Normal;
            BasicVector contact1 = contact.Contact1;
            BasicVector contact2 = contact.Contact2;
            int contactCount = contact.ContactCount;

            float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

            this.contactList[0] = contact1;
            this.contactList[1] = contact2;

            for (int i = 0; i < contactCount; i++)
            {
                this.impulseList[i] = BasicVector.Zero;
                this.raList[i] = BasicVector.Zero;
                this.rbList[i] = BasicVector.Zero;
            }

            for (int i = 0; i < contactCount; i++)
            {
                BasicVector ra = contactList[i] - bodyA.Position;
                BasicVector rb = contactList[i] - bodyB.Position;

                raList[i] = ra;
                rbList[i] = rb;

                BasicVector raPerp = new BasicVector(-ra.Y, ra.X);
                BasicVector rbPerp = new BasicVector(-rb.Y, rb.X);

                BasicVector angularLinearVelocityA = raPerp * bodyA.AngularVelocity;
                BasicVector angularLinearVelocityB = rbPerp * bodyB.AngularVelocity;

                BasicVector relativeVelocity =
                    (bodyB.LinearVelocity + angularLinearVelocityB) -
                    (bodyA.LinearVelocity + angularLinearVelocityA);

                float contactVelocityMag = BasicMath.Dot(relativeVelocity, normal);

                if (contactVelocityMag > 0f)
                {
                    continue;
                }

                float raPerpDotN = BasicMath.Dot(raPerp, normal);
                float rbPerpDotN = BasicMath.Dot(rbPerp, normal);

                float denom = bodyA.InvMass + bodyB.InvMass +
                    (raPerpDotN * raPerpDotN) * bodyA.InvInertia +
                    (rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

                float j = -(1f + e) * contactVelocityMag;
                j /= denom;
                j /= (float)contactCount;

                BasicVector impulse = j * normal;
                impulseList[i] = impulse;
            }

            for (int i = 0; i < contactCount; i++)
            {
                BasicVector impulse = impulseList[i];
                BasicVector ra = raList[i];
                BasicVector rb = rbList[i];

                bodyA.LinearVelocity += -impulse * bodyA.InvMass;
                bodyA.AngularVelocity += -BasicMath.Cross(ra, impulse) * bodyA.InvInertia;
                bodyB.LinearVelocity += impulse * bodyB.InvMass;
                bodyB.AngularVelocity += BasicMath.Cross(rb, impulse) * bodyB.InvInertia;
            }
        }
        /// <summary>
        /// Just simple screenwrap, do i need to explain. aahh, ok. When a body goes offscreen. it places it on the other side.
        /// </summary>
        /// <param name="camera"></param>
        /// <exception cref="Exception"></exception>
        public void ScreenWrap(Camera camera)
        {

            camera.GetExtents(out Vector2 camMin, out Vector2 camMax);

            float viewW = camMax.X - camMin.X;
            float viewH = camMax.Y - camMin.Y;
            for (int i = 0; i < BodyCount; i++)
            {
                if (!GetBody(i, out RigidBody body))
                {
                    throw new Exception("Not found");
                }

                if (body.Position.X < camMin.X) { body.MoveTo(body.Position + new BasicVector(viewW, 0)); }
                if (body.Position.X > camMax.X) { body.MoveTo(body.Position - new BasicVector(viewW, 0)); }
                if (body.Position.Y < camMin.Y) { body.MoveTo(body.Position + new BasicVector(0, viewH)); }
                if (body.Position.Y > camMax.Y) { body.MoveTo(body.Position - new BasicVector(0, viewH)); }
            }
        }


        /// <summary>
        /// Removes any body when it is off the bottom of the screen. else returns false. goes out with an index it returned at.
        /// </summary>
        /// <param name="camera"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public bool RemoveWhenOffBottom(Camera camera, out int index)
        {
            camera.GetExtents(out float left, out float right, out float bottom, out float top);
            index = -1;
            for (int i = 0; i < BodyCount; i++)
            {
                if (!GetBody(i, out RigidBody body))
                {
                    throw new ArgumentOutOfRangeException();
                }

                AABB box = body.GetAABB();

                if (box.Max.Y < bottom)
                {
                    RemoveBody(body);
                    index = i;
                    return true;
                }


            }
            index = -1;
            return false;
        }
        /// <summary>
        /// Removes any body when it is off the bottom of the screen. else returns false. 
        /// </summary>
        /// <param name="camera"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public bool RemoveWhenOffBottom(Camera camera)
        {
            camera.GetExtents(out float left, out float right, out float bottom, out float top);

            for (int i = 0; i < BodyCount; i++)
            {
                if (!GetBody(i, out RigidBody body))
                {
                    throw new ArgumentOutOfRangeException();
                }

                AABB box = body.GetAABB();

                if (box.Max.Y < bottom)
                {
                    RemoveBody(body);

                    return true;
                }

            }

            return false;
        }

        /// <summary>
        /// Removes any body when it is offscreen. else returns false. goes out with an index it returned at.
        /// </summary>
        /// <param name="camera"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public bool RemoveWhenOffScreen(Camera camera, out int index)
        {
            camera.GetExtents(out float left, out float right, out float bottom, out float top);
            index = -1;
            for (int i = 0; i < BodyCount; i++)
            {
                if (!GetBody(i, out RigidBody body))
                {
                    throw new ArgumentOutOfRangeException();
                }

                AABB box = body.GetAABB();

                if (box.Max.Y < bottom)
                {
                    RemoveBody(body);
                    index = i;
                    return true;
                }
                if (box.Max.Y > top)
                {
                    RemoveBody(body);
                    index = i;
                    return true;
                }
                if (box.Max.X < left)
                {
                    RemoveBody(body);
                    index = i;
                    return true;
                }
                if (box.Max.X > right)
                {
                    RemoveBody(body);
                    index = i;
                    return true;
                }


            }
            index = -1;
            return false;
        }
        /// <summary>
        /// Removes any body when it is offscreen. else returns false. 
        /// </summary>
        /// <param name="camera"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public bool RemoveWhenOffScreen(Camera camera)
        {
            camera.GetExtents(out float left, out float right, out float bottom, out float top);

            for (int i = 0; i < BodyCount; i++)
            {
                if (!GetBody(i, out RigidBody body))
                {
                    throw new ArgumentOutOfRangeException();
                }

                AABB box = body.GetAABB();

                if (box.Max.Y < bottom)
                {
                    RemoveBody(body);

                    return true;
                }
                if (box.Max.Y > top)
                {
                    RemoveBody(body);

                    return true;
                }
                if (box.Max.X < left)
                {
                    RemoveBody(body);

                    return true;
                }
                if (box.Max.X > right)
                {
                    RemoveBody(body);

                    return true;
                }
            }

            return false;
        }
    }
}
