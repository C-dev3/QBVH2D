using System.Numerics;

namespace QBVH2D.Bench;

/// <summary>
/// Minimal <see cref="IBounded"/> shape used only to populate trees for benchmarking.
/// A fixed-size axis-aligned box centered on <see cref="Center"/>.
/// </summary>
internal readonly struct BenchShape(Vector2 center, float halfSize) : IBounded
{
    private readonly AABB _aabb = new(center - new Vector2(halfSize), center + new Vector2(halfSize));

    public AABB GetAABB() => _aabb;
}

/// <summary>
/// Deterministic generation of shapes, query points, rays and query AABBs so that every
/// benchmark run (and every method within a run) operates on identical input data.
/// </summary>
internal static class BenchData
{
    // Keep the world bounds fixed regardless of shape count so density changes with N,
    // which is representative of "more obstacles in the same map" rather than "same
    // density, bigger map".
    public const float WorldSize = 10_000f;
    private const float ShapeHalfSize = 2.5f;

    public static BenchShape[] CreateShapes(int count, int seed = 12345)
    {
        var rng = new Random(seed);
        var shapes = new BenchShape[count];
        for (int i = 0; i < count; i++)
        {
            var center = new Vector2(
                (float)(rng.NextDouble() * WorldSize),
                (float)(rng.NextDouble() * WorldSize));
            shapes[i] = new BenchShape(center, ShapeHalfSize);
        }
        return shapes;
    }

    public static Vector2[] CreatePoints(int count, int seed = 999)
    {
        var rng = new Random(seed);
        var points = new Vector2[count];
        for (int i = 0; i < count; i++)
        {
            points[i] = new Vector2(
                (float)(rng.NextDouble() * WorldSize),
                (float)(rng.NextDouble() * WorldSize));
        }
        return points;
    }

    public static (Vector2 Origin, Vector2 Direction)[] CreateRays(int count, int seed = 4242)
    {
        var rng = new Random(seed);
        var rays = new (Vector2, Vector2)[count];
        for (int i = 0; i < count; i++)
        {
            var origin = new Vector2(
                (float)(rng.NextDouble() * WorldSize),
                (float)(rng.NextDouble() * WorldSize));
            var target = new Vector2(
                (float)(rng.NextDouble() * WorldSize),
                (float)(rng.NextDouble() * WorldSize));
            rays[i] = (origin, target - origin);
        }
        return rays;
    }

    public static AABB[] CreateQueryAABBs(int count, float querySize, int seed = 777)
    {
        var rng = new Random(seed);
        var boxes = new AABB[count];
        for (int i = 0; i < count; i++)
        {
            var center = new Vector2(
                (float)(rng.NextDouble() * WorldSize),
                (float)(rng.NextDouble() * WorldSize));
            var half = new Vector2(querySize * 0.5f);
            boxes[i] = new AABB(center - half, center + half);
        }
        return boxes;
    }
}