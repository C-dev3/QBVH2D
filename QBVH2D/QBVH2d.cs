using System.Buffers;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace QBVH2D;

/// <summary>
/// 2D Quad Bounding Volume Hierarchy (QBVH) for efficient spatial queries
/// Uses 4-way branching instead of binary for better cache utilization and reduced tree depth
/// </summary>
public class QBVH2d
{
    internal QBVH2dNode[] Nodes { get; set; } = Array.Empty<QBVH2dNode>();

    /// <summary>
    /// Number of nodes currently in the tree. Valid node indices are in range [0, NodeCount).
    /// Always 0 when <see cref="RootLeafShapeIndex"/> is set, since a single-shape tree doesn't
    /// need a node at all.
    /// </summary>
    public int NodeCount { get; internal set; }

    /// <summary>
    /// When the whole tree is a single shape, its index - direct-encoded here instead of in a
    /// node, since a lone shape has no siblings to branch against. -1 for every other tree
    /// (including an empty one), in which case <see cref="RootIndex"/> is the real root.
    /// </summary>
    internal int RootLeafShapeIndex { get; set; } = -1;

    /// <summary>
    /// 
    /// </summary>
    public const int RootIndex = 0;

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

        // Start small and let QBVH2dNode.Build grow the array on demand instead of
        // pre-allocating the theoretical worst case (a perfectly balanced quad-tree).
        // MaxLeafSize == 4 means internal-node count is typically well under the shape
        // count, so this is already a generous starting point for the common case.
        int initialCapacity = Math.Max(4, shapes.Length + shapes.Length / 2);
        var nodes = new QBVH2dNode[initialCapacity];
        int nodeCount = 0;

        int rootEncoded;
        if (shapes.Length <= 1024)
        {
            Span<int> indices = stackalloc int[shapes.Length];
            for (int i = 0; i < shapes.Length; i++)
                indices[i] = i;

            rootEncoded = QBVH2dNode.Build(shapes, indices, ref nodes, ref nodeCount);
        }
        else
        {
            var indices = new int[shapes.Length];
            for (int i = 0; i < shapes.Length; i++)
                indices[i] = i;

            rootEncoded = QBVH2dNode.Build(shapes, indices.AsSpan(), ref nodes, ref nodeCount);
        }

        if (rootEncoded < 0)
        {
            // The whole tree is a single shape: QBVH2dNode.Build direct-encoded it without
            // creating any node at all.
            return new QBVH2d
            {
                Nodes = Array.Empty<QBVH2dNode>(),
                NodeCount = 0,
                RootLeafShapeIndex = ~rootEncoded,
            };
        }

        if (nodeCount < nodes.Length)
        {
            Array.Resize(ref nodes, nodeCount);
        }

        return new QBVH2d
        {
            Nodes = nodes,
            NodeCount = nodeCount,
        };
    }

    /// <summary>
    /// Gets a read-only view of the node at the given index, for custom traversals that the
    /// built-in Query* methods don't cover (e.g. k-nearest-neighbor / best-first search, queries
    /// against custom shapes, debug visualization, or tree statistics). Start from
    /// <see cref="RootIndex"/> and use <see cref="QBVHNodeView.GetChildIndex"/> to descend.
    /// </summary>
    /// <remarks>
    /// Not valid when <see cref="RootLeafShapeIndex"/> is set (a single-shape tree has no
    /// nodes) - check that first for custom traversals that need to handle every tree.
    /// </remarks>
    /// <param name="index">Node index, in range [0, NodeCount)</param>
    /// <returns>A read-only view of the node</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public QBVHNodeView GetNode(int index) => new(Nodes[index]);

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

        foreach (var index in ContainsIterator(point))
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

        foreach (var index in ContainsIterator(point))
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
        foreach (var index in ContainsIterator(point))
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

        if (RootLeafShapeIndex >= 0)
        {
            results.Add(RootLeafShapeIndex);
            return results;
        }

        if (NodeCount == 0) return results;

        const int InitialCapacity = 64;
        int[] stack = ArrayPool<int>.Shared.Rent(InitialCapacity);
        try
        {
            int sp = 0;
            stack[sp++] = RootIndex;

            // Every entry on this stack is an internal node index: direct-encoded leaves are
            // resolved straight from the parent's mask below and never pushed.
            while (sp > 0)
            {
                int nodeIndex = stack[--sp];
                ref var node = ref Nodes[nodeIndex];

                node.GetChildBoundsSoA(out var minX, out var minY, out var maxX, out var maxY);
                int intersectsMask = AABB.Intersects4(in aabb, minX, minY, maxX, maxY);
                int mask = intersectsMask & node.Flags & 0xF;

                while (mask != 0)
                {
                    int bit = BitOperations.TrailingZeroCount(mask);
                    mask &= mask - 1;

                    if (node.IsChildLeaf(bit))
                    {
                        results.Add(node.GetChildIndex(bit));
                        continue;
                    }

                    if (sp == stack.Length)
                    {
                        var bigger = ArrayPool<int>.Shared.Rent(stack.Length * 2);
                        stack.AsSpan(0, sp).CopyTo(bigger);
                        ArrayPool<int>.Shared.Return(stack);
                        stack = bigger;
                    }

                    stack[sp++] = node.GetChildIndex(bit);
                }
            }
        }
        finally
        {
            ArrayPool<int>.Shared.Return(stack);
        }

        return results;
    }

    /// <summary>
    /// Checks whether any shape's AABB intersects the given query AABB, stopping at the first hit
    /// without allocating a result list. Use this instead of <see cref="QueryAABB"/> when only the
    /// presence of an intersection matters (e.g. obstacle/occupancy checks), since it avoids the
    /// list allocation and returns as soon as one match is found.
    /// </summary>
    /// <param name="aabb">The AABB to query</param>
    /// <returns><see langword="true"/> if at least one shape's AABB intersects <paramref name="aabb"/></returns>
    public bool QueryAABBAny(AABB aabb)
    {
        if (RootLeafShapeIndex >= 0) return true;
        if (NodeCount == 0) return false;

        int[] stack = ArrayPool<int>.Shared.Rent(64);
        try
        {
            int sp = 0;
            stack[sp++] = RootIndex;

            while (sp > 0)
            {
                int nodeIndex = stack[--sp];
                ref var node = ref Nodes[nodeIndex];

                node.GetChildBoundsSoA(out var minX, out var minY, out var maxX, out var maxY);
                int mask = AABB.Intersects4(in aabb, minX, minY, maxX, maxY) & node.Flags & 0xF;

                while (mask != 0)
                {
                    int bit = BitOperations.TrailingZeroCount(mask);
                    mask &= mask - 1;

                    if (node.IsChildLeaf(bit)) return true;

                    if (sp == stack.Length)
                    {
                        var bigger = ArrayPool<int>.Shared.Rent(stack.Length * 2);
                        stack.AsSpan(0, sp).CopyTo(bigger);
                        ArrayPool<int>.Shared.Return(stack);
                        stack = bigger;
                    }
                    stack[sp++] = node.GetChildIndex(bit);
                }
            }
            return false;
        }
        finally
        {
            ArrayPool<int>.Shared.Return(stack);
        }
    }

    /// <summary>
    /// Creates an iterator that traverses shapes whose AABB intersects the given ray or segment
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="direction">The ray's direction (does not need to be normalized)</param>
    /// <param name="maxT">
    /// Maximum ray parameter to accept as a hit. When <paramref name="direction"/> is the raw
    /// displacement from <paramref name="origin"/> to a target point, pass 1.0 to restrict the
    /// query to that exact segment. Defaults to positive infinity for an unbounded ray.
    /// </param>
    /// <returns>An iterator over shape indices whose AABB the ray/segment intersects</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public QBVH2DRayIterator RayIterator(Vector2 origin, Vector2 direction, float maxT = float.PositiveInfinity) => new(this, origin, direction, maxT);

    /// <summary>
    /// Gets all shape indices whose AABB intersects the given ray or segment
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="direction">The ray's direction (does not need to be normalized)</param>
    /// <param name="maxT">Maximum ray parameter to accept as a hit (see <see cref="RayIterator"/>)</param>
    /// <returns>List of shape indices</returns>
    public List<int> QueryRay(Vector2 origin, Vector2 direction, float maxT = float.PositiveInfinity)
    {
        List<int> results = new(16);

        foreach (var index in RayIterator(origin, direction, maxT))
        {
            results.Add(index);
        }
        return results;
    }

    /// <summary>
    /// Gets all shape indices whose AABB intersects the given ray or segment
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="direction">The ray's direction (does not need to be normalized)</param>
    /// <param name="maxT">Maximum ray parameter to accept as a hit (see <see cref="RayIterator"/>)</param>
    /// <param name="results">Span to write results to</param>
    /// <returns>Number of results written</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int QueryRay(Vector2 origin, Vector2 direction, float maxT, Span<int> results)
    {
        int count = 0;

        foreach (var index in RayIterator(origin, direction, maxT))
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
    /// Gets all shape indices whose AABB intersects the given ray or segment
    /// </summary>
    /// <param name="origin">The ray's origin point</param>
    /// <param name="direction">The ray's direction (does not need to be normalized)</param>
    /// <param name="maxT">Maximum ray parameter to accept as a hit (see <see cref="RayIterator"/>)</param>
    /// <param name="results">List to add results to (not cleared)</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void QueryRay(Vector2 origin, Vector2 direction, float maxT, ref List<int> results)
    {
        foreach (var index in RayIterator(origin, direction, maxT))
        {
            results.Add(index);
        }
    }

    /// <summary>
    /// Checks whether any shape's AABB intersects the segment from <paramref name="from"/> to
    /// <paramref name="to"/>, stopping at the first hit without allocating. Intended for
    /// line-of-sight checks (e.g. Theta* pathfinding) against a QBVH built from obstacle shapes:
    /// a <see langword="false"/> result means the straight line between the two points is clear
    /// of every registered obstacle's AABB.
    /// </summary>
    /// <param name="from">The segment's start point</param>
    /// <param name="to">The segment's end point</param>
    /// <returns><see langword="true"/> if at least one shape's AABB blocks the segment</returns>
    public bool RaycastAny(Vector2 from, Vector2 to)
    {
        if (RootLeafShapeIndex >= 0) return true;
        if (NodeCount == 0) return false;

        var direction = to - from;
        var invDir = AABB.InvDir(direction);
        const float maxT = 1.0f;

        const int InitialCapacity = 64;
        int[] stack = ArrayPool<int>.Shared.Rent(InitialCapacity);
        try
        {
            int sp = 0;
            stack[sp++] = RootIndex;

            while (sp > 0)
            {
                int nodeIndex = stack[--sp];
                ref var node = ref Nodes[nodeIndex];

                node.GetChildBoundsSoA(out var minX, out var minY, out var maxX, out var maxY);
                int hitMask = AABB.IntersectsRay4(in from, in invDir, maxT, minX, minY, maxX, maxY);
                int mask = hitMask & node.Flags & 0xF;

                while (mask != 0)
                {
                    int bit = BitOperations.TrailingZeroCount(mask);
                    mask &= mask - 1;

                    if (node.IsChildLeaf(bit)) return true;

                    if (sp == stack.Length)
                    {
                        var bigger = ArrayPool<int>.Shared.Rent(stack.Length * 2);
                        stack.AsSpan(0, sp).CopyTo(bigger);
                        ArrayPool<int>.Shared.Return(stack);
                        stack = bigger;
                    }

                    stack[sp++] = node.GetChildIndex(bit);
                }
            }

            return false;
        }
        finally
        {
            ArrayPool<int>.Shared.Return(stack);
        }
    }
}