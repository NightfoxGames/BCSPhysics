using System;

namespace BCSP
{
    internal readonly struct BasicTransform
    {
        public readonly static BasicTransform Zero = new BasicTransform(0f, 0f, 0f);

        public readonly float PositionX;
        public readonly float PositionY;
        public readonly float Sin;
        public readonly float Cos;

        public BasicTransform(BasicVector position, float angle)
        {
            this.PositionX = position.X;
            this.PositionY = position.Y;
            this.Sin = MathF.Sin(angle);
            this.Cos = MathF.Cos(angle);
        }

        public BasicTransform(float x, float y, float angle)
        {
            this.PositionX = x;
            this.PositionY = y;
            this.Sin = MathF.Sin(angle);
            this.Cos = MathF.Cos(angle);
        }


    }
}
