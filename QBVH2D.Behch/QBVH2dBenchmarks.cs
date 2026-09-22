using System.Numerics;
using BenchmarkDotNet.Attributes;
using QBVH2D;

namespace QBVH2D.Bench;

/// <summary>
/// Benchmarks for QBVH2D build and query performance.
/// Run with: dotnet run -c Release -- --filter *QBVH2dBenchmarks*
/// </summary>
[MemoryDiagnoser]
[RankColumn]
public class QBVH2dBenchmarks
{
    // Shape counts to sweep. Adjust as needed; 1,000,000 can take a while to build
    // repeatedly under BenchmarkDotNet's default iteration count.
    [Params(1_000, 100_000, 1_000_000)]
    public int ShapeCount;

    // Number of queries issued per benchmark invocation. Doing several per invocation
    // amortizes per-call overhead and keeps individual invocations above the timer
    // resolution, which matters most for the point/ray queries at low ShapeCount.
    private const int QueriesPerInvocation = 2_000;

    private BenchShape[] _shapes = [];
    private QBVH2d _tree = null!;

    private Vector2[] _queryPoints = [];
    private (Vector2 Origin, Vector2 Direction)[] _rays = [];
    private AABB[] _smallQueryAABBs = []; // sparse hits, like a grid cell check
    private AABB[] _largeQueryAABBs = []; // dense hits, many results per query

    // Reused across QueryPoint/QueryRay(ref List<int>) benchmarks to isolate traversal
    // cost from list (re)allocation cost.
    private List<int> _reusableResults = new(64);

    [GlobalSetup]
    public void GlobalSetup()
    {
        _shapes = BenchData.CreateShapes(ShapeCount);
        _tree = QBVH2d.Build(_shapes);

        _queryPoints = BenchData.CreatePoints(QueriesPerInvocation);
        _rays = BenchData.CreateRays(QueriesPerInvocation);

        // Small query box: roughly matches shape size, so most queries hit 0-1 shapes.
        // Representative of a grid-cell obstacle check (e.g. the D* Lite use case).
        _smallQueryAABBs = BenchData.CreateQueryAABBs(QueriesPerInvocation, querySize: 10f);

        // Large query box: covers a meaningful fraction of the world, so most queries
        // hit many shapes. Representative of a broad-phase range query.
        _largeQueryAABBs = BenchData.CreateQueryAABBs(QueriesPerInvocation, querySize: 500f);
    }

    // ---- Build ----

    [Benchmark(Description = "Build")]
    public QBVH2d Build() => QBVH2d.Build(_shapes);

    // ---- Point queries ----

    [Benchmark(Description = "QueryPoint (allocates List<int>)")]
    public int QueryPoint_Allocating()
    {
        int total = 0;
        for (int i = 0; i < _queryPoints.Length; i++)
        {
            total += _tree.QueryPoint(_queryPoints[i]).Count;
        }
        return total;
    }

    [Benchmark(Description = "QueryPoint (reused List<int>)")]
    public int QueryPoint_Reused()
    {
        int total = 0;
        for (int i = 0; i < _queryPoints.Length; i++)
        {
            _reusableResults.Clear();
            _tree.QueryPoint(_queryPoints[i], ref _reusableResults);
            total += _reusableResults.Count;
        }
        return total;
    }

    // ---- AABB range queries ----

    [Benchmark(Description = "QueryAABB small (sparse hits, allocates)")]
    public int QueryAABB_Small()
    {
        int total = 0;
        for (int i = 0; i < _smallQueryAABBs.Length; i++)
        {
            total += _tree.QueryAABB(_smallQueryAABBs[i]).Count;
        }
        return total;
    }

    [Benchmark(Description = "QueryAABB large (dense hits, allocates)")]
    public int QueryAABB_Large()
    {
        int total = 0;
        for (int i = 0; i < _largeQueryAABBs.Length; i++)
        {
            total += _tree.QueryAABB(_largeQueryAABBs[i]).Count;
        }
        return total;
    }

    [Benchmark(Description = "QueryAABBAny small (existence check, no alloc)")]
    public int QueryAABBAny_Small()
    {
        int hits = 0;
        for (int i = 0; i < _smallQueryAABBs.Length; i++)
        {
            if (_tree.QueryAABBAny(_smallQueryAABBs[i])) hits++;
        }
        return hits;
    }

    // ---- Ray queries ----

    [Benchmark(Description = "QueryRay (allocates List<int>)")]
    public int QueryRay_Allocating()
    {
        int total = 0;
        for (int i = 0; i < _rays.Length; i++)
        {
            var (origin, direction) = _rays[i];
            total += _tree.QueryRay(origin, direction).Count;
        }
        return total;
    }

    [Benchmark(Description = "RaycastAny (existence check, no alloc)")]
    public int RaycastAny()
    {
        int hits = 0;
        for (int i = 0; i < _rays.Length; i++)
        {
            var (origin, direction) = _rays[i];
            // direction is target - origin here, so maxT = 1.0 restricts to the segment,
            // matching RaycastAny's documented contract.
            if (_tree.RaycastAny(origin, origin + direction)) hits++;
        }
        return hits;
    }
}