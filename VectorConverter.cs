using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BCSP
{
    public class VectorConverter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToXnaVector2(BasicVector v)
        {
            return new Vector2(v.X, v.Y);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BasicVector ToBasicVector(Vector2 v)
        {
            return new BasicVector(v.X, v.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ToXnaVector2Array(BasicVector[] src, ref Vector2[] dst)
        {
            if (dst == null || src.Length != dst.Length)
            {
                dst = new Vector2[src.Length];
            }

            for (int i = 0; i < src.Length; i++)
            {
                BasicVector v = src[i];
                dst[i] = new Vector2(v.X, v.Y);
            }
        }
    }
}
