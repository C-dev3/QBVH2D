using System.Numerics;
using System.Runtime.CompilerServices;

namespace QBVH2D;

/// <summary>
/// 2D Quad Bounding Volume Hierarchy (QBVH) for efficient spatial queries
/// Uses 4-way branching instead of binary for better cache utilization and reduced tree depth
/// </summary>
public class QBVH2d
{
    internal QBVH2dNode[] Nodes { get; set; }
    internal int NodeCount { get; set; }

    /// <summary>
    /// Creates an empty QBVH2D
    /// </summary>
    public QBVH2d()
    {
        Nodes = Array.Empty<QBVH2dNode>();
        NodeCount = 0;
    }

    /// <summary>
    /// Builds a QBVH from a collection of shapes
    /// </summary>
    /// <typeparam name="T">Type of shape that implements IBounded</typeparam>
    /// <param name="shapes">Array of shapes to build the QBVH from</param>
    /// <returns>A new QBVH2D containing the shapes</returns>
    public static QBVH2d Build<T>(T[] shapes) where T : IBounded
    {
        ArgumentNullException.ThrowIfNull(shapes);

        if (shapes.Length == 0)
            return new QBVH2d();

        // QBVH uses 4-way branching, so calculate max nodes needed
        // Upper bound: each level reduces by factor of 4
        int maxDepth = (int)Math.Ceiling(Math.Log(shapes.Length, 4)) + 1;
        int expectedNodeCount = (int)((Math.Pow(4, maxDepth) - 1) / 3) + shapes.Length;
        var nodes = new QBVH2dNode[expectedNodeCount];
        int nodeCount = 0;

        if (shapes.Length <= 1024)
        {
            // Use stackalloc for indices buffer
            Span<int> indices = stackalloc int[shapes.Length];
            for (int i = 0; i < shapes.Length; i++)
                indices[i] = i;

            QBVH2dNode.Build(shapes, indices, nodes, ref nodeCount);
        }
        else
        {
            // For very large datasets, fall back to array
            var indices = new int[shapes.Length];
            for (int i = 0; i < shapes.Length; i++)
                indices[i] = i;

            QBVH2dNode.Build(shapes, indices.AsSpan(), nodes, ref nodeCount);
        }


        return new QBVH2d
        {
            Nodes = nodes,
            NodeCount = nodeCount,
        };
    }

    /// <summary>
    /// Creates an iterator that traverses shapes containing the given point
    /// </summary>
    /// <param name="point">The point to query</param>
    /// <returns>An iterator over shape indices that may contain the point</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public QBVH2DTraverseIterator ContainsIterator(Vector2 point) => new(this, point);

    /// <summary>
    /// Gets all shape indices that may contain the given point
    /// </summary>
    /// <param name="point">The point to query</param>
    /// <returns>List of shape indices</returns>
    public List<int> QueryPoint(Vector2 point)
    {
        List<int> results = new(16);

        using var iterator = ContainsIterator(point);
        foreach (var index in iterator)
        {
            results.Add(index);
        }
        return results;
    }

    /// <summary>
    /// Gets all shape indices that may contain the given point
    /// </summary>
    /// <param name="point">The point to query</param>
    /// <param name="results">Span to write results to</param>
    /// <returns>Number of results written</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int QueryPoint(Vector2 point, Span<int> results)
    {
        int count = 0;

        using var iterator = ContainsIterator(point);
        foreach (var index in iterator)
        {
            if (count < results.Length)
            {
                results[count++] = index;
            }
            else
            {
                break;
            }
        }
        return count;
    }

    /// <summary>
    /// Gets all shape indices that may contain the given point
    /// </summary>
    /// <param name="point">The point to query</param>
    /// <param name="results">List to add results to (not cleared)</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void QueryPoint(Vector2 point, ref List<int> results)
    {
        using var iterator = ContainsIterator(point);
        foreach (var index in iterator)
        {
            results.Add(index);
        }
    }

    /// <summary>
    /// Queries all shapes that intersect with the given AABB
    /// </summary>
    /// <param name="aabb">The AABB to query</param>
    /// <returns>List of shape indices</returns>
    public List<int> QueryAABB(AABB aabb)
    {
        List<int> results = new(32);
        QueryAABBRecursive(0, aabb, results);
        return results;
    }

    private void QueryAABBRecursive(int nodeIndex, AABB queryAABB, List<int> results)
    {
        if (nodeIndex >= NodeCount) return;

        ref var node = ref Nodes[nodeIndex];

        if (node.IsLeaf)
        {
            results.Add(node.ShapeIndex);
            return;
        }

        node.GetChildAABBRefs(out var aabb0, out var aabb1, out var aabb2, out var aabb3);
        int intersectsMask = AABB.Intersects4(in queryAABB, in aabb0, in aabb1, in aabb2, in aabb3);

        if ((intersectsMask & 1) != 0 && node.HasChild(0))
        {
            QueryAABBRecursive(node.GetChildIndex(0), queryAABB, results);
        }
        if ((intersectsMask & 2) != 0 && node.HasChild(1))
        {
            QueryAABBRecursive(node.GetChildIndex(1), queryAABB, results);
        }
        if ((intersectsMask & 4) != 0 && node.HasChild(2))
        {
            QueryAABBRecursive(node.GetChildIndex(2), queryAABB, results);
        }
        if ((intersectsMask & 8) != 0 && node.HasChild(3))
        {
            QueryAABBRecursive(node.GetChildIndex(3), queryAABB, results);
        }
    }
}
