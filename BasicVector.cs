using System;

namespace BCSP
{
    public readonly struct BasicVector
    {
        public readonly float X;
        public readonly float Y;

        public static readonly BasicVector Zero = new BasicVector(0f, 0f);

        public BasicVector(float x, float y)
        {
            this.X = x;
            this.Y = y;
        }

        

        public static BasicVector operator +(BasicVector a, BasicVector b)
        {
            return new BasicVector(a.X + b.X, a.Y + b.Y);
        }

        public static BasicVector operator -(BasicVector a, BasicVector b)
        {
            return new BasicVector(a.X - b.X, a.Y - b.Y);
        }

        public static BasicVector operator -(BasicVector v)
        {
            return new BasicVector(-v.X, -v.Y);
        }

        public static BasicVector operator *(BasicVector v, float s)
        {
            return new BasicVector(v.X * s, v.Y * s);
        }

        public static BasicVector operator *(float s, BasicVector v)
        {
            return new BasicVector(v.X * s, v.Y * s);
        }

        public static BasicVector operator /(BasicVector v, float s)
        {
            return new BasicVector(v.X / s, v.Y / s);
        }

        internal static BasicVector Transform(BasicVector v, BasicTransform transform)
        {
            return new BasicVector(
                transform.Cos * v.X - transform.Sin * v.Y + transform.PositionX, 
                transform.Sin * v.X + transform.Cos * v.Y + transform.PositionY);
        }

        public bool Equals(BasicVector other)
        {
            return this.X == other.X && this.Y == other.Y;
        }

        public bool MoreThan(BasicVector other)
        {
            return this.X > other.X && this.Y > other.Y;
        }
        public bool LessThan(BasicVector other)
        {
            return this.X < other.X && this.Y < other.Y;
        }
        

        public override bool Equals(object obj)
        {
            if (obj is BasicVector other)
            {
                return this.Equals(other);
            }

            return false;
        }

        public override int GetHashCode()
        {
            return new { this.X, this.Y }.GetHashCode();
        }

        public override string ToString()
        {
            return $"X: {this.X}, Y: {this.Y}";
        }
    }
}
