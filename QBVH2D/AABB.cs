using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace QBVH2D;

/// <summary>
/// Axis-Aligned Bounding Box (AABB) in 2D space
/// </summary>
public struct AABB
{
    /// <summary>
    /// The minimum corner point of the AABB (bottom-left in 2D space)
    /// </summary>
    public Vector2 Min { get; set; }

    /// <summary>
    /// The maximum corner point of the AABB (top-right in 2D space)
    /// </summary>
    public Vector2 Max { get; set; }

    /// <summary>
    /// Represents an empty/invalid AABB that contains no area. Used as a sentinel value for initialization.
    /// </summary>
    public static readonly AABB Empty = new(
        new Vector2(float.PositiveInfinity, float.PositiveInfinity),
        new Vector2(float.NegativeInfinity, float.NegativeInfinity)
    );

    /// <summary>
    /// Initializes a new instance of the AABB structure with the specified minimum and maximum points
    /// </summary>
    /// <param name="min">The minimum corner point (bottom-left)</param>
    /// <param name="max">The maximum corner point (top-right)</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public AABB(Vector2 min, Vector2 max)
    {
        Min = min;
        Max = max;
    }

    /// <summary>
    /// Joins this AABB with another in-place
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void JoinMut(in AABB other)
    {
        Min = Vector2.Min(Min, other.Min);
        Max = Vector2.Max(Max, other.Max);
    }

    /// <summary>
    /// Returns the smallest AABB that encloses both <paramref name="a"/> and <paramref name="b"/>.
    /// Safe to call with <see cref="Empty"/> as either operand: because <see cref="Empty"/> is
    /// (+Infinity, +Infinity) - (-Infinity, -Infinity), unioning it with any real AABB returns
    /// that AABB unchanged.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static AABB Union(in AABB a, in AABB b) => new(Vector2.Min(a.Min, b.Min), Vector2.Max(a.Max, b.Max));

    /// <summary>
    /// Grows the AABB to include a point in-place
    /// </summary>
    /// <param name="point"></param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void GrowMut(in Vector2 point)
    {
        Min = Vector2.Min(Min, point);
        Max = Vector2.Max(Max, point);
    }

    /// <summary>
    /// Gets the size of the AABB
    /// </summary>
    public readonly Vector2 Size
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Max - Min;
    }

    /// <summary>
    /// Gets the center point of the AABB
    /// </summary>
    public readonly Vector2 Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Min + (Size / 2.0f);
    }

    /// <summary>
    /// Checks if the AABB is empty (invalid)
    /// </summary>
    internal readonly bool IsEmpty
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Min.X > Max.X || Min.Y > Max.Y;
    }

    /// <summary>
    /// Checks if a point is contained within this AABB
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Contains(in Vector2 point) => point.X >= Min.X && point.X <= Max.X &&
               point.Y >= Min.Y && point.Y <= Max.Y;

    /// <summary>
    /// Checks if a point is contained in any of 4 AABBs using SIMD operations.
    /// Takes the 4 children's bounds already transposed into SoA form (one lane per child),
    /// so no gather/transpose is needed here - the caller (QBVH2dNode) stores bounds this way.
    /// Returns a bitmask where bit i is set if the point is in AABB i.
    /// </summary>
    /// <param name="point">The point to test</param>
    /// <param name="minX">Min.X of children 0-3, one per lane</param>
    /// <param name="minY">Min.Y of children 0-3, one per lane</param>
    /// <param name="maxX">Max.X of children 0-3, one per lane</param>
    /// <param name="maxY">Max.Y of children 0-3, one per lane</param>
    /// <returns>Bitmask indicating which AABBs contain the point (bit 0-3)</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int Contains4(in Vector2 point, Vector128<float> minX, Vector128<float> minY, Vector128<float> maxX, Vector128<float> maxY)
    {
        // Use SIMD to test 4 AABBs simultaneously
        if (Sse.IsSupported)
        {
            // Load point X and Y into all lanes
            var px = Vector128.Create(point.X);
            var py = Vector128.Create(point.Y);

            // Check: point.X >= Min.X && point.X <= Max.X
            var geMinX = Sse.CompareGreaterThanOrEqual(px, minX);
            var leMaxX = Sse.CompareLessThanOrEqual(px, maxX);

            // Check: point.Y >= Min.Y && point.Y <= Max.Y
            var geMinY = Sse.CompareGreaterThanOrEqual(py, minY);
            var leMaxY = Sse.CompareLessThanOrEqual(py, maxY);

            // Combine all conditions with AND
            var resultX = Sse.And(geMinX, leMaxX);
            var resultY = Sse.And(geMinY, leMaxY);
            var result = Sse.And(resultX, resultY);

            // Convert to bitmask
            return Sse.MoveMask(result);
        }
        else
        {
            // Fallback: scalar version, reading each lane out of the SoA vectors
            int mask = 0;
            for (int i = 0; i < 4; i++)
            {
                bool inside = point.X >= minX.GetElement(i) && point.X <= maxX.GetElement(i) &&
                              point.Y >= minY.GetElement(i) && point.Y <= maxY.GetElement(i);
                if (inside) mask |= 1 << i;
            }
            return mask;
        }
    }

    /// <summary>
    /// Checks if a query AABB intersects with any of 4 AABBs using SIMD operations.
    /// Takes the 4 children's bounds already transposed into SoA form (one lane per child).
    /// Returns a bitmask where bit i is set if the query intersects AABB i.
    /// </summary>
    /// <param name="query">The query AABB</param>
    /// <param name="minX">Min.X of children 0-3, one per lane</param>
    /// <param name="minY">Min.Y of children 0-3, one per lane</param>
    /// <param name="maxX">Max.X of children 0-3, one per lane</param>
    /// <param name="maxY">Max.Y of children 0-3, one per lane</param>
    /// <returns>Bitmask indicating which AABBs intersect (bit 0-3)</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int Intersects4(in AABB query, Vector128<float> minX, Vector128<float> minY, Vector128<float> maxX, Vector128<float> maxY)
    {
        // Use SIMD to test 4 AABB intersections simultaneously
        if (Sse.IsSupported)
        {
            // Load query bounds
            var qMinX = Vector128.Create(query.Min.X);
            var qMinY = Vector128.Create(query.Min.Y);
            var qMaxX = Vector128.Create(query.Max.X);
            var qMaxY = Vector128.Create(query.Max.Y);

            // Check intersection: query.Min.X <= aabb.Max.X && query.Max.X >= aabb.Min.X
            var cond1 = Sse.CompareLessThanOrEqual(qMinX, maxX);
            var cond2 = Sse.CompareGreaterThanOrEqual(qMaxX, minX);
            var cond3 = Sse.CompareLessThanOrEqual(qMinY, maxY);
            var cond4 = Sse.CompareGreaterThanOrEqual(qMaxY, minY);

            // Combine all conditions
            var result = Sse.And(Sse.And(cond1, cond2), Sse.And(cond3, cond4));

            return Sse.MoveMask(result);
        }
        else
        {
            // Fallback: scalar version, reading each lane out of the SoA vectors
            int mask = 0;
            for (int i = 0; i < 4; i++)
            {
                bool hit = query.Min.X <= maxX.GetElement(i) && query.Max.X >= minX.GetElement(i) &&
                           query.Min.Y <= maxY.GetElement(i) && query.Max.Y >= minY.GetElement(i);
                if (hit) mask |= 1 << i;
            }
            return mask;
        }
    }

    /// <summary>
    /// Checks if this AABB intersects with another AABB
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool Intersects(in AABB other) =>
        Min.X <= other.Max.X && Max.X >= other.Min.X &&
        Min.Y <= other.Max.Y && Max.Y >= other.Min.Y;

    /// <summary>
    /// Computes the component-wise inverse of a ray direction (1/dir.X, 1/dir.Y).
    /// Precompute this once per ray/segment and reuse it across many <see cref="IntersectsRay(in Vector2, in Vector2, float)"/>
    /// or <see cref="IntersectsRay4"/> calls to avoid repeated division.
    /// A zero component correctly produces +-Infinity, which the slab method handles safely.
    /// </summary>
    /// <param name="direction">The ray direction (does not need to be normalized)</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 InvDir(in Vector2 direction) => new(1f / direction.X, 1f / direction.Y);

    /// <summary>
    /// Checks if a ray/segment intersects this AABB using the slab method.
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="invDir">
    /// Component-wise inverse of the ray direction. Use <see cref="InvDir"/> to compute it once
    /// per ray rather than dividing on every call.
    /// </param>
    /// <param name="maxT">
    /// Maximum ray parameter to accept as a hit, e.g. pass the segment length when direction is
    /// not normalized and you only care about intersections between origin and origin + direction.
    /// Defaults to positive infinity for an unbounded ray.
    /// </param>
    /// <returns><see langword="true"/> if the ray intersects the AABB within [0, maxT]</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IntersectsRay(in Vector2 origin, in Vector2 invDir, float maxT = float.PositiveInfinity)
    {
        // Intersect the ray with the X slab (the region between the left and right planes)
        float tx1 = (Min.X - origin.X) * invDir.X;
        float tx2 = (Max.X - origin.X) * invDir.X;

        float tMin = MathF.Min(tx1, tx2);
        float tMax = MathF.Max(tx1, tx2);

        // Intersect with the Y slab and narrow the running [tMin, tMax] interval
        float ty1 = (Min.Y - origin.Y) * invDir.Y;
        float ty2 = (Max.Y - origin.Y) * invDir.Y;

        tMin = MathF.Max(tMin, MathF.Min(ty1, ty2));
        tMax = MathF.Min(tMax, MathF.Max(ty1, ty2));

        // Hit if the slabs overlap (tMax >= tMin), the overlap isn't entirely behind the
        // origin (tMax >= 0), and it starts within the allowed range (tMin <= maxT)
        return tMax >= tMin && tMax >= 0f && tMin <= maxT;
    }

    /// <summary>
    /// Same as <see cref="IntersectsRay(in Vector2, in Vector2, float)"/> but also returns the
    /// entry distance along the ray, clamped to 0 (useful when the origin starts inside the box).
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="invDir">
    /// Component-wise inverse of the ray direction. Use <see cref="InvDir"/> to compute it once
    /// per ray rather than dividing on every call.
    /// </param>
    /// <param name="maxT">
    /// Maximum ray parameter to accept as a hit, e.g. pass the segment length when direction is
    /// not normalized and you only care about intersections between origin and origin + direction.
    /// </param>
    /// <param name="tHit">The ray parameter at the entry point when this returns true; otherwise undefined</param>
    /// <returns><see langword="true"/> if the ray intersects the AABB within [0, maxT]</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IntersectsRay(in Vector2 origin, in Vector2 invDir, float maxT, out float tHit)
    {
        float tx1 = (Min.X - origin.X) * invDir.X;
        float tx2 = (Max.X - origin.X) * invDir.X;

        float tMin = MathF.Min(tx1, tx2);
        float tMax = MathF.Max(tx1, tx2);

        float ty1 = (Min.Y - origin.Y) * invDir.Y;
        float ty2 = (Max.Y - origin.Y) * invDir.Y;

        tMin = MathF.Max(tMin, MathF.Min(ty1, ty2));
        tMax = MathF.Min(tMax, MathF.Max(ty1, ty2));

        bool hit = tMax >= tMin && tMax >= 0f && tMin <= maxT;
        tHit = hit ? MathF.Max(tMin, 0f) : 0f;
        return hit;
    }

    /// <summary>
    /// Checks if a ray/segment intersects any of 4 AABBs using SIMD operations (slab method).
    /// Takes the 4 children's bounds already transposed into SoA form (one lane per child), so
    /// the hot traversal path only needs a straight load of these vectors from QBVH2dNode - no
    /// per-query gather/transpose.
    /// Returns a bitmask where bit i is set if the ray intersects AABB i within [0, maxT].
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="invDir">Component-wise inverse of the ray direction, see <see cref="InvDir"/></param>
    /// <param name="maxT">Maximum ray parameter to accept as a hit (e.g. segment length)</param>
    /// <param name="minX">Min.X of children 0-3, one per lane</param>
    /// <param name="minY">Min.Y of children 0-3, one per lane</param>
    /// <param name="maxX">Max.X of children 0-3, one per lane</param>
    /// <param name="maxY">Max.Y of children 0-3, one per lane</param>
    /// <returns>Bitmask indicating which AABBs the ray intersects (bit 0-3)</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int IntersectsRay4(in Vector2 origin, in Vector2 invDir, float maxT,
        Vector128<float> minX, Vector128<float> minY, Vector128<float> maxX, Vector128<float> maxY)
    {
        if (Sse.IsSupported)
        {
            // Broadcast the ray (same origin/direction tested against all 4 boxes)
            var ox = Vector128.Create(origin.X);
            var oy = Vector128.Create(origin.Y);
            var invDx = Vector128.Create(invDir.X);
            var invDy = Vector128.Create(invDir.Y);

            // X slab
            var tx1 = Sse.Multiply(Sse.Subtract(minX, ox), invDx);
            var tx2 = Sse.Multiply(Sse.Subtract(maxX, ox), invDx);
            var tMinX = Sse.Min(tx1, tx2);
            var tMaxX = Sse.Max(tx1, tx2);

            // Y slab
            var ty1 = Sse.Multiply(Sse.Subtract(minY, oy), invDy);
            var ty2 = Sse.Multiply(Sse.Subtract(maxY, oy), invDy);
            var tMinY = Sse.Min(ty1, ty2);
            var tMaxY = Sse.Max(ty1, ty2);

            // Narrow the running interval across both slabs
            var tMin = Sse.Max(tMinX, tMinY);
            var tMax = Sse.Min(tMaxX, tMaxY);

            var zero = Vector128<float>.Zero;
            var maxTVec = Vector128.Create(maxT);

            var overlap = Sse.CompareGreaterThanOrEqual(tMax, tMin);
            var inFront = Sse.CompareGreaterThanOrEqual(tMax, zero);
            var withinRange = Sse.CompareLessThanOrEqual(tMin, maxTVec);

            var result = Sse.And(Sse.And(overlap, inFront), withinRange);

            return Sse.MoveMask(result);
        }
        else
        {
            // Fallback: scalar version, reading each lane out of the SoA vectors
            int mask = 0;
            for (int i = 0; i < 4; i++)
            {
                float tx1 = (minX.GetElement(i) - origin.X) * invDir.X;
                float tx2 = (maxX.GetElement(i) - origin.X) * invDir.X;
                float tMinF = MathF.Min(tx1, tx2);
                float tMaxF = MathF.Max(tx1, tx2);

                float ty1 = (minY.GetElement(i) - origin.Y) * invDir.Y;
                float ty2 = (maxY.GetElement(i) - origin.Y) * invDir.Y;
                tMinF = MathF.Max(tMinF, MathF.Min(ty1, ty2));
                tMaxF = MathF.Min(tMaxF, MathF.Max(ty1, ty2));

                if (tMaxF >= tMinF && tMaxF >= 0f && tMinF <= maxT) mask |= 1 << i;
            }
            return mask;
        }
    }
}