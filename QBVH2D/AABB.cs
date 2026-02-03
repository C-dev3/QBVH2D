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
    /// Returns a bitmask where bit i is set if the point is in AABB i.
    /// </summary>
    /// <param name="point">The point to test</param>
    /// <param name="aabb0">First AABB</param>
    /// <param name="aabb1">Second AABB</param>
    /// <param name="aabb2">Third AABB</param>
    /// <param name="aabb3">Fourth AABB</param>
    /// <returns>Bitmask indicating which AABBs contain the point (bit 0-3)</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static unsafe int Contains4(in Vector2 point, in AABB aabb0, in AABB aabb1, in AABB aabb2, in AABB aabb3)
    {
        // Use SIMD to test 4 AABBs simultaneously
        if (Sse.IsSupported)
        {
            // Load point X and Y into all lanes
            var px = Vector128.Create(point.X);
            var py = Vector128.Create(point.Y);

            // Load Min.X from all 4 AABBs
            var minX = Vector128.Create(aabb0.Min.X, aabb1.Min.X, aabb2.Min.X, aabb3.Min.X);
            var minY = Vector128.Create(aabb0.Min.Y, aabb1.Min.Y, aabb2.Min.Y, aabb3.Min.Y);

            // Load Max.X from all 4 AABBs
            var maxX = Vector128.Create(aabb0.Max.X, aabb1.Max.X, aabb2.Max.X, aabb3.Max.X);
            var maxY = Vector128.Create(aabb0.Max.Y, aabb1.Max.Y, aabb2.Max.Y, aabb3.Max.Y);

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
            // Fallback: scalar version
            int mask = 0;
            if (aabb0.Contains(in point)) mask |= 1;
            if (aabb1.Contains(in point)) mask |= 2;
            if (aabb2.Contains(in point)) mask |= 4;
            if (aabb3.Contains(in point)) mask |= 8;
            return mask;
        }
    }

    /// <summary>
    /// Checks if a query AABB intersects with any of 4 AABBs using SIMD operations.
    /// Returns a bitmask where bit i is set if the query intersects AABB i.
    /// </summary>
    /// <param name="query">The query AABB</param>
    /// <param name="aabb0">First AABB</param>
    /// <param name="aabb1">Second AABB</param>
    /// <param name="aabb2">Third AABB</param>
    /// <param name="aabb3">Fourth AABB</param>
    /// <returns>Bitmask indicating which AABBs intersect (bit 0-3)</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static unsafe int Intersects4(in AABB query, in AABB aabb0, in AABB aabb1, in AABB aabb2, in AABB aabb3)
    {
        // Use SIMD to test 4 AABB intersections simultaneously
        if (Sse.IsSupported)
        {
            // Load query bounds
            var qMinX = Vector128.Create(query.Min.X);
            var qMinY = Vector128.Create(query.Min.Y);
            var qMaxX = Vector128.Create(query.Max.X);
            var qMaxY = Vector128.Create(query.Max.Y);

            // Load AABB bounds
            var minX = Vector128.Create(aabb0.Min.X, aabb1.Min.X, aabb2.Min.X, aabb3.Min.X);
            var minY = Vector128.Create(aabb0.Min.Y, aabb1.Min.Y, aabb2.Min.Y, aabb3.Min.Y);
            var maxX = Vector128.Create(aabb0.Max.X, aabb1.Max.X, aabb2.Max.X, aabb3.Max.X);
            var maxY = Vector128.Create(aabb0.Max.Y, aabb1.Max.Y, aabb2.Max.Y, aabb3.Max.Y);

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
            // Fallback: scalar version
            int mask = 0;
            if (query.Intersects(in aabb0)) mask |= 1;
            if (query.Intersects(in aabb1)) mask |= 2;
            if (query.Intersects(in aabb2)) mask |= 4;
            if (query.Intersects(in aabb3)) mask |= 8;
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
}