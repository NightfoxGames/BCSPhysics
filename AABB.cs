using System;

namespace BCSP
{
    public readonly struct AABB
    {
        public readonly BasicVector Min;
        public readonly BasicVector Max;

        public AABB(BasicVector min, BasicVector max)
        {
            this.Min = min;
            this.Max = max;
        }

        public AABB(float minX, float minY, float maxX, float maxY)
        {
            this.Min = new BasicVector(minX, minY);
            this.Max = new BasicVector(maxX, maxY);
        }
    }
}
