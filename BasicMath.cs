using System;

namespace BCSP
{
    public static class BasicMath
    {
        /// <summary>
        /// Equal to 1/2 of a millimeter.
        /// </summary>
        public static readonly float VerySmallAmount = 0.0005f;

        public static float Clamp(float value, float min, float max)
        {
            if(min == max)
            {
                return min;
            }

            if(min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than the max.");
            }

            if(value < min)
            {
                return min;
            }

            if(value > max)
            {
                return max;
            }

            return value;
        }

        public static int Clamp(int value, int min, int max)
        {
            if (min == max)
            {
                return min;
            }

            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than the max.");
            }

            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;
        }

        public static float LengthSquared(BasicVector v)
        {
            return v.X * v.X + v.Y * v.Y;
        }

        public static float Length(BasicVector v)
        {
            return MathF.Sqrt(v.X * v.X + v.Y * v.Y);
        }

        public static float DistanceSquared(BasicVector a, BasicVector b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return dx * dx + dy * dy;
        }

        public static float Distance(BasicVector a, BasicVector b)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            return MathF.Sqrt(dx * dx + dy * dy);
        }

        public static BasicVector Normalize(BasicVector v)
        {
            float len = BasicMath.Length(v);
            return new BasicVector(v.X / len, v.Y / len);
        }

        public static float Dot(BasicVector a, BasicVector b)
        {
            // a · b = ax * bx + ay * by
            return a.X * b.X + a.Y * b.Y;
        }

        public static float Cross(BasicVector a, BasicVector b)
        {
            // cz = ax * by − ay * bx
            return a.X * b.Y - a.Y * b.X;
        }

        public static bool NearlyEqual(float a, float b)
        {
            return MathF.Abs(a - b) < BasicMath.VerySmallAmount;
        }

        public static bool NearlyEqual(BasicVector a, BasicVector b)
        {
            return BasicMath.DistanceSquared(a, b) < BasicMath.VerySmallAmount * BasicMath.VerySmallAmount;
        }
    }
}
