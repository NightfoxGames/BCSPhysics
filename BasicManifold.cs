using System;

namespace BCSP
{
    public readonly struct BasicManifold
    {
        public readonly RigidBody BodyA;
        public readonly RigidBody BodyB;
        public readonly BasicVector Normal;
        public readonly float Depth;
        public readonly BasicVector Contact1;
        public readonly BasicVector Contact2;
        public readonly int ContactCount;

        public BasicManifold(
            RigidBody bodyA, RigidBody bodyB, 
            BasicVector normal, float depth, 
            BasicVector contact1, BasicVector contact2, int contactCount)
        {
            this.BodyA = bodyA;
            this.BodyB = bodyB;
            this.Normal = normal;
            this.Depth = depth;
            this.Contact1 = contact1;
            this.Contact2 = contact2;
            this.ContactCount = contactCount;
        }
    }
}
