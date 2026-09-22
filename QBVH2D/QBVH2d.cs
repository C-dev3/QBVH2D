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
    /// Number of nodes in <see cref="Nodes"/> that are orphaned - no longer reachable from the
    /// root - because <see cref="Remove"/> collapsed them into their parent's slot without
    /// reclaiming their array slot. Reset to 0 by <see cref="Build{T}"/>/<see cref="Rebuild{T}"/>,
    /// since a freshly built tree has no garbage. See <see cref="GarbageRatio"/>.
    /// </summary>
    internal int GarbageNodeCount { get; set; }

    /// <summary>
    /// The fraction of <see cref="NodeCount"/> that is currently orphaned garbage (see
    /// <see cref="GarbageNodeCount"/>). Callers that Remove a lot can poll this to decide when a
    /// <see cref="Rebuild{T}"/> is worth its O(n log n) cost to reclaim the wasted space.
    /// </summary>
    internal float GarbageRatio => NodeCount == 0 ? 0f : (float)GarbageNodeCount / NodeCount;

    /// <summary>
    /// When the whole tree is a single shape, its index - direct-encoded here instead of in a
    /// node, since a lone shape has no siblings to branch against. -1 for every other tree
    /// (including an empty one), in which case <see cref="RootIndex"/> is the real root.
    /// </summary>
    internal int RootLeafShapeIndex { get; set; } = -1;

    /// <summary>
    /// Reverse lookup: shapeIndex -> encoded leaf location (nodeIndex * 4 + slot). -1 when the
    /// shape isn't currently tracked as a direct-encoded leaf inside a node (absent from the tree,
    /// or it's the sole shape held directly via <see cref="RootLeafShapeIndex"/>). Maintained by
    /// <see cref="Build{T}"/>, <see cref="Rebuild{T}"/>, <see cref="Insert{T}"/> and
    /// <see cref="Remove"/> so that <see cref="Remove"/>/<see cref="Update"/> can find a shape's
    /// slot in O(1) instead of searching the tree.
    /// </summary>
    internal int[] ShapeLeafLocation { get; set; } = Array.Empty<int>();

    /// <summary>
    /// Per-node reverse lookup: for node <c>i</c> (other than <see cref="RootIndex"/>),
    /// <c>ParentSlot[i]</c> is the encoded (parentNodeIndex * 4 + slotInParent) of the slot in its
    /// parent that points to it. -1 for <see cref="RootIndex"/> and for any unused/orphaned node
    /// slot left behind by a collapse in <see cref="Remove"/>.
    /// </summary>
    internal int[] ParentSlot { get; set; } = Array.Empty<int>();

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
        GarbageNodeCount = 0;
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
            var shapeLeafLocationEncoded = new int[shapes.Length];
            Array.Fill(shapeLeafLocationEncoded, -1);
            var parentSlotEncoded = new int[nodes.Length];
            Array.Fill(parentSlotEncoded, -1);
            IndexTree(nodes, nodeCount, shapeLeafLocationEncoded, parentSlotEncoded);

            // The whole tree is a single shape: QBVH2dNode.Build direct-encoded it without
            // creating any node at all.
            return new QBVH2d
            {
                Nodes = Array.Empty<QBVH2dNode>(),
                NodeCount = 0,
                RootLeafShapeIndex = ~rootEncoded,
                ShapeLeafLocation = shapeLeafLocationEncoded,
                ParentSlot = parentSlotEncoded,
                GarbageNodeCount = 0
            };
        }

        if (nodeCount < nodes.Length)
        {
            Array.Resize(ref nodes, nodeCount);
        }

        var shapeLeafLocation = new int[shapes.Length];
        Array.Fill(shapeLeafLocation, -1);
        var parentSlot = new int[nodes.Length];
        Array.Fill(parentSlot, -1);
        IndexTree(nodes, nodeCount, shapeLeafLocation, parentSlot);

        return new QBVH2d
        {
            Nodes = nodes,
            NodeCount = nodeCount,
            ShapeLeafLocation = shapeLeafLocation,
            ParentSlot = parentSlot,
            GarbageNodeCount = 0
        };
    }

    /// <summary>
    /// Rebuilds this QBVH2d in place from an explicit subset of <paramref name="shapes"/>,
    /// selected by <paramref name="shapeIndices"/>, reusing the existing <see cref="Nodes"/>
    /// buffer when it is already large enough.
    /// </summary>
    /// <remarks>
    /// Lets a caller that tombstones removed shapes as gaps in its own backing array (rather
    /// than shifting every later index on removal) pass that raw, possibly-sparse array directly
    /// together with just the indices of its currently-active shapes - batching any number of
    /// adds/removes/moves accumulated since the last rebuild into one call, without first
    /// compacting its own collection into a fresh, gap-free array.
    /// <paramref name="shapeIndices"/> is a mutable <see cref="Span{T}"/>, not
    /// <see cref="ReadOnlySpan{T}"/>, because <c>QBVH2dNode.Build</c> partitions the index buffer
    /// in place while splitting; pass a scratch buffer you're fine seeing reordered.
    /// </remarks>
    /// <typeparam name="T">Type of shape that implements IBounded</typeparam>
    /// <param name="shapes">The backing shape array (may contain entries not referenced by <paramref name="shapeIndices"/>)</param>
    /// <param name="shapeIndices">The indices into <paramref name="shapes"/> to include in the rebuilt tree; reordered in place during the build</param>
    public void Rebuild<T>(T[] shapes, Span<int> shapeIndices) where T : IBounded
    {
        ArgumentNullException.ThrowIfNull(shapes);

        if (shapeIndices.Length == 0)
        {
            Nodes = Array.Empty<QBVH2dNode>();
            NodeCount = 0;
            RootLeafShapeIndex = -1;
            GarbageNodeCount = 0;
            return;
        }

        int requiredCapacity = Math.Max(4, shapeIndices.Length + shapeIndices.Length / 2);
        var nodes = Nodes.Length >= requiredCapacity ? Nodes : new QBVH2dNode[requiredCapacity];
        int nodeCount = 0;

        int rootEncoded = QBVH2dNode.Build(shapes, shapeIndices, ref nodes, ref nodeCount);

        if (rootEncoded < 0)
        {
            Nodes = Array.Empty<QBVH2dNode>();
            NodeCount = 0;
            RootLeafShapeIndex = ~rootEncoded;
            GarbageNodeCount = 0;
            return;
        }

        Nodes = nodes;
        NodeCount = nodeCount;
        RootLeafShapeIndex = -1;
        GarbageNodeCount = 0;
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
    /// Batch version of <see cref="QueryAABBAny(AABB)"/>: for each AABB in
    /// <paramref name="queries"/>, writes whether it intersects any shape into the same slot of
    /// <paramref name="results"/>. Reuses a single rented traversal stack across the whole batch,
    /// instead of the per-call ArrayPool Rent/Return that calling <see cref="QueryAABBAny(AABB)"/>
    /// once per query incurs - useful for callers that re-test many AABBs against the same tree in
    /// a tight loop (e.g. rasterizing a grid of cells against obstacle geometry).
    /// </summary>
    /// <param name="queries">The AABBs to test</param>
    /// <param name="results">
    /// Receives one bool per entry of <paramref name="queries"/>, in the same order. Must be at
    /// least as long as <paramref name="queries"/>.
    /// </param>
    /// <exception cref="ArgumentException"><paramref name="results"/> is shorter than <paramref name="queries"/>.</exception>
    public void QueryAABBAny(ReadOnlySpan<AABB> queries, Span<bool> results)
    {
        if (results.Length < queries.Length)
            throw new ArgumentException("results must be at least as long as queries.", nameof(results));

        if (RootLeafShapeIndex >= 0)
        {
            results[..queries.Length].Fill(true);
            return;
        }

        if (NodeCount == 0)
        {
            results[..queries.Length].Fill(false);
            return;
        }

        int[] stack = ArrayPool<int>.Shared.Rent(64);
        try
        {
            for (int q = 0; q < queries.Length; q++)
            {
                AABB aabb = queries[q];
                int sp = 0;
                stack[sp++] = RootIndex;
                bool hit = false;

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

                        if (node.IsChildLeaf(bit)) { hit = true; break; }

                        if (sp == stack.Length)
                        {
                            var bigger = ArrayPool<int>.Shared.Rent(stack.Length * 2);
                            stack.AsSpan(0, sp).CopyTo(bigger);
                            ArrayPool<int>.Shared.Return(stack);
                            stack = bigger;
                        }
                        stack[sp++] = node.GetChildIndex(bit);
                    }

                    if (hit) break;
                }

                results[q] = hit;
            }
        }
        finally
        {
            ArrayPool<int>.Shared.Return(stack);
        }
    }

    /// <summary>
    /// Inserts a single new shape into the tree in place, reusing every subtree not on the path
    /// from the root down to the new leaf - the true O(log n) incremental insert, as opposed to
    /// a full <see cref="Build{T}"/>/<c>Rebuild</c> from every shape.
    /// </summary>
    /// <remarks>
    /// Descends the tree choosing, at each node, either an empty child slot (free - enlarges
    /// nothing) or, failing that, the occupied slot whose bounds would grow least to include the
    /// new shape's AABB (the standard R-tree/dynamic-BVH "choose subtree" heuristic), recording
    /// the path taken. The new shape is then placed either directly into an empty slot, or by
    /// promoting an existing leaf slot into a brand-new 2-child node holding the old and new
    /// leaves (exactly one new node is ever created per insert). Finally the recorded path is
    /// walked back up, enlarging each ancestor's relevant child bounds to include the new shape.
    /// Only nodes on that path - depth O(log₄ n) for a balanced tree - are read or written; every
    /// sibling subtree, and its bounds, is left completely untouched.
    /// <para>
    /// This does not rebalance the tree. Many inserts in a row (especially into a small,
    /// clustered region) can degrade query performance versus a from-scratch <see cref="Build{T}"/>;
    /// periodically rebuilding is still worth doing for a tree that accumulates a lot of inserts
    /// over its lifetime. Removal/relocation of an existing shape is a separate, harder problem
    /// (freeing a slot without leaving the tree in an inconsistent state, and without shifting
    /// every other shape's index) and is not covered by this method.
    /// </para>
    /// </remarks>
    /// <typeparam name="T">Type of shape that implements IBounded</typeparam>
    /// <param name="shapes">
    /// The backing shape array. Only consulted for <paramref name="newShapeIndex"/>'s own bounds,
    /// and - in the rare case the whole tree is currently a single direct-encoded leaf - for that
    /// existing leaf's bounds too, since that's the one place in this tree where a leaf's bounds
    /// isn't already cached in a parent node.
    /// </param>
    /// <param name="newShapeIndex">The new shape's index into <paramref name="shapes"/>.</param>
    public void Insert<T>(T[] shapes, int newShapeIndex) where T : IBounded
    {
        AABB newBounds = shapes[newShapeIndex].GetAABB();

        // Tree currently empty: the new shape becomes the whole tree.
        if (NodeCount == 0 && RootLeafShapeIndex < 0)
        {
            RootLeafShapeIndex = newShapeIndex;
            return;
        }

        // Tree currently a single direct-encoded leaf: promote the root into a 2-child node.
        if (RootLeafShapeIndex >= 0)
        {
            int oldLeafIndex = RootLeafShapeIndex;
            AABB oldLeafBounds = shapes[oldLeafIndex].GetAABB();

            var nodes = Nodes;
            QBVH2dNode.EnsureCapacity(ref nodes, 1);
            Nodes = nodes;

            Nodes[RootIndex] = QBVH2dNode.CreateNode(
                ~oldLeafIndex, oldLeafBounds,
                ~newShapeIndex, newBounds,
                QBVH2dNode.NoChild, AABB.Empty,
                QBVH2dNode.NoChild, AABB.Empty);

            NodeCount = 1;
            RootLeafShapeIndex = -1;

            var parentSlot = ParentSlot;
            EnsureCapacityFilled(ref parentSlot, 1);
            ParentSlot = parentSlot;
            ParentSlot[RootIndex] = -1;

            var shapeLeafLocation = ShapeLeafLocation;
            EnsureCapacityFilled(ref shapeLeafLocation, Math.Max(oldLeafIndex, newShapeIndex) + 1);
            ShapeLeafLocation = shapeLeafLocation;
            ShapeLeafLocation[oldLeafIndex] = EncodeLocation(RootIndex, 0);
            ShapeLeafLocation[newShapeIndex] = EncodeLocation(RootIndex, 1);
            return;
        }

        // General case: descend from the root, recording (nodeIndex, slot) at every level.
        Span<(int nodeIndex, int slot)> path = stackalloc (int, int)[32];
        int pathLength = 0;
        int currentNodeIndex = RootIndex;

        while (true)
        {
            if (pathLength == path.Length)
            {
                // 32 levels of 4-way branching covers well over 4 billion shapes - a safety
                // valve against a corrupt tree, not an expected path in practice.
                throw new InvalidOperationException("QBVH2d.Insert exceeded the maximum supported tree depth.");
            }

            ref QBVH2dNode node = ref Nodes[currentNodeIndex];
            int chosenSlot = ChooseSlot(in node, in newBounds, out bool isEmptySlot);
            path[pathLength++] = (currentNodeIndex, chosenSlot);

            if (isEmptySlot)
            {
                node.SetChildLeaf(chosenSlot, newShapeIndex, in newBounds);

                var shapeLeafLocation = ShapeLeafLocation;
                EnsureCapacityFilled(ref shapeLeafLocation, newShapeIndex + 1);
                ShapeLeafLocation = shapeLeafLocation;
                ShapeLeafLocation[newShapeIndex] = EncodeLocation(currentNodeIndex, chosenSlot);
                break;
            }

            if (node.IsChildLeaf(chosenSlot))
            {
                int oldLeafIndex = node.GetChildIndex(chosenSlot);
                AABB oldLeafBounds = node.GetChildBounds(chosenSlot);

                // `node` (a ref into the current Nodes array) must not be written to after this
                // point: EnsureCapacity may reallocate the array, leaving `node` stale. Everything
                // still needed from it (above) has already been read.
                var nodes = Nodes;
                QBVH2dNode.EnsureCapacity(ref nodes, NodeCount + 1);
                Nodes = nodes;
                int newNodeIndex = NodeCount++;

                Nodes[newNodeIndex] = QBVH2dNode.CreateNode(
                    ~oldLeafIndex, oldLeafBounds,
                    ~newShapeIndex, newBounds,
                    QBVH2dNode.NoChild, AABB.Empty,
                    QBVH2dNode.NoChild, AABB.Empty);

                Nodes[currentNodeIndex].SetChildNode(chosenSlot, newNodeIndex, AABB.Union(oldLeafBounds, newBounds));

                var parentSlot = ParentSlot;
                EnsureCapacityFilled(ref parentSlot, newNodeIndex + 1);
                ParentSlot = parentSlot;
                ParentSlot[newNodeIndex] = EncodeLocation(currentNodeIndex, chosenSlot);

                var shapeLeafLocation = ShapeLeafLocation;
                EnsureCapacityFilled(ref shapeLeafLocation, Math.Max(oldLeafIndex, newShapeIndex) + 1);
                ShapeLeafLocation = shapeLeafLocation;
                ShapeLeafLocation[oldLeafIndex] = EncodeLocation(newNodeIndex, 0);
                ShapeLeafLocation[newShapeIndex] = EncodeLocation(newNodeIndex, 1);
                break;
            }

            currentNodeIndex = node.GetChildIndex(chosenSlot);
        }

        // Refit every ancestor above the node the new shape was actually placed in - that node's
        // own slot bounds are already exact from SetChildLeaf/SetChildNode above.
        for (int i = pathLength - 2; i >= 0; i--)
        {
            (int nodeIndex, int slot) = path[i];
            AABB existing = Nodes[nodeIndex].GetChildBounds(slot);
            Nodes[nodeIndex].SetChildBounds(slot, AABB.Union(existing, newBounds));
        }
    }

    /// <summary>
    /// Removes a single shape from the tree in place, jumping straight to its leaf slot via
    /// <see cref="ShapeLeafLocation"/> instead of searching - the true O(log n) incremental
    /// counterpart to <see cref="Insert{T}"/>. Unlike Insert, this never needs the shapes array:
    /// every bounds value it touches is already cached in the tree.
    /// </summary>
    /// <remarks>
    /// Clears the shape's slot, then either refits ancestor bounds upward (if its node still has
    /// 2+ children) or collapses the now-single-child node into its parent's slot and refits from
    /// there - preserving the invariant, established by <see cref="Build{T}"/> and
    /// <see cref="Insert{T}"/>, that every node other than a lone leaf has at least 2 children.
    /// A collapsed node is left as an orphaned, unreferenced entry in <see cref="Nodes"/> rather
    /// than compacted; a periodic <see cref="Rebuild{T}"/> reclaims that space.
    /// </remarks>
    /// <param name="shapeIndex">The shape index to remove, as previously passed to Build/Rebuild/Insert.</param>
    /// <exception cref="ArgumentException">The shape is not currently present in this tree.</exception>
    public void Remove(int shapeIndex)
    {
        if (RootLeafShapeIndex == shapeIndex) { RootLeafShapeIndex = -1; return; }

        if ((uint)shapeIndex >= (uint)ShapeLeafLocation.Length || ShapeLeafLocation[shapeIndex] < 0)
            throw new ArgumentException("Shape is not present in this tree.", nameof(shapeIndex));

        int location = ShapeLeafLocation[shapeIndex];
        int nodeIndex = DecodeNode(location);
        Nodes[nodeIndex].ClearChild(DecodeSlot(location));
        ShapeLeafLocation[shapeIndex] = -1;

        CollapseOrRefit(nodeIndex);
    }

    /// <summary>
    /// Updates a shape's cached bounds in place after it moved or resized, then refits every
    /// ancestor's bounds upward - a lightweight alternative to Remove+Insert for a shape whose
    /// new bounds don't warrant relocating it to a different part of the tree.
    /// </summary>
    /// <remarks>
    /// This does not re-choose which slot/subtree the shape belongs in. A shape that moved far
    /// enough that it would now be placed elsewhere by <see cref="Insert{T}"/> still stays exactly
    /// where it is - just with looser ancestor bounds along the way up. For a shape that moved by
    /// more than roughly its own size, prefer <see cref="Remove"/> followed by <see cref="Insert{T}"/>
    /// to avoid the tree's bounds drifting loose over many such updates.
    /// <para>
    /// No-op when the whole tree is a single shape (<see cref="RootLeafShapeIndex"/> set): that
    /// case has no cached bounds to update in the first place, since queries read the shape's
    /// bounds live from the shapes array.
    /// </para>
    /// </remarks>
    /// <param name="shapeIndex">The shape index to update, as previously passed to Build/Rebuild/Insert.</param>
    /// <param name="newBounds">The shape's current AABB.</param>
    /// <exception cref="ArgumentException">The shape is not currently present in this tree.</exception>
    public void Update(int shapeIndex, in AABB newBounds)
    {
        if (RootLeafShapeIndex == shapeIndex) return;

        if ((uint)shapeIndex >= (uint)ShapeLeafLocation.Length || ShapeLeafLocation[shapeIndex] < 0)
            throw new ArgumentException("Shape is not present in this tree.", nameof(shapeIndex));

        int location = ShapeLeafLocation[shapeIndex];
        int nodeIndex = DecodeNode(location);
        Nodes[nodeIndex].SetChildBounds(DecodeSlot(location), in newBounds);
        RefitUpward(nodeIndex);
    }

    /// <summary>
    /// After a slot was cleared in <paramref name="nodeIndex"/> by <see cref="Remove"/>: refits
    /// ancestor bounds upward if the node still has 2+ children, or - if only one remains -
    /// collapses the node into its parent's slot (or, at the root, into
    /// <see cref="RootLeafShapeIndex"/>) and refits from there instead.
    /// </summary>
    private void CollapseOrRefit(int nodeIndex)
    {
        ref QBVH2dNode node = ref Nodes[nodeIndex];

        if (node.ChildCount() != 1)
        {
            RefitUpward(nodeIndex);
            return;
        }

        int soleSlot = node.SoleChildSlot();
        bool soleIsLeaf = node.IsChildLeaf(soleSlot);
        int soleValue = node.GetChildIndex(soleSlot);
        AABB soleBounds = node.GetChildBounds(soleSlot);

        if (nodeIndex == RootIndex)
        {
            if (soleIsLeaf)
            {
                RootLeafShapeIndex = soleValue;
                ShapeLeafLocation[soleValue] = -1;
                GarbageNodeCount++;
            }
            return;
        }

        int parentLocation = ParentSlot[nodeIndex];
        int parentNode = DecodeNode(parentLocation);
        int parentSlotIdx = DecodeSlot(parentLocation);

        if (soleIsLeaf)
        {
            Nodes[parentNode].SetChildLeaf(parentSlotIdx, soleValue, in soleBounds);
            ShapeLeafLocation[soleValue] = EncodeLocation(parentNode, parentSlotIdx);
        }
        else
        {
            Nodes[parentNode].SetChildNode(parentSlotIdx, soleValue, in soleBounds);
            ParentSlot[soleValue] = EncodeLocation(parentNode, parentSlotIdx);
        }

        ParentSlot[nodeIndex] = -1;
        GarbageNodeCount++;
        RefitUpward(parentNode);
    }

    /// <summary>
    /// Walks from <paramref name="nodeIndex"/> up to the root, recomputing each visited node's own
    /// bounds as the union of its (possibly just-changed) children and writing that into its
    /// parent's slot. Unlike <see cref="Insert{T}"/>'s refit - a simple growth union, since
    /// insertion only ever enlarges bounds - this recomputes from scratch at each level, since
    /// <see cref="Remove"/> and <see cref="Update"/> can shrink an ancestor's bounds.
    /// </summary>
    private void RefitUpward(int nodeIndex)
    {
        int current = nodeIndex;
        while (true)
        {
            int parentLocation = ParentSlot[current];
            if (parentLocation < 0) return;

            int parentNode = DecodeNode(parentLocation);
            int slot = DecodeSlot(parentLocation);
            Nodes[parentNode].SetChildBounds(slot, Nodes[current].UnionOfOccupiedChildren());
            current = parentNode;
        }
    }

    /// <summary>
    /// Picks which child slot of <paramref name="node"/> a shape with <paramref name="newBounds"/>
    /// should go into: any empty slot if one exists, otherwise the occupied slot whose bounds
    /// would grow least (by area) to include <paramref name="newBounds"/>.
    /// </summary>
    private static int ChooseSlot(in QBVH2dNode node, in AABB newBounds, out bool isEmptySlot)
    {
        for (int i = 0; i < 4; i++)
        {
            if (!node.HasChild(i))
            {
                isEmptySlot = true;
                return i;
            }
        }

        isEmptySlot = false;
        int bestSlot = 0;
        float bestGrowth = float.PositiveInfinity;

        for (int i = 0; i < 4; i++)
        {
            AABB slotBounds = node.GetChildBounds(i);
            float growth = Area(AABB.Union(slotBounds, newBounds)) - Area(slotBounds);
            if (growth < bestGrowth)
            {
                bestGrowth = growth;
                bestSlot = i;
            }
        }

        return bestSlot;
    }

    /// <summary>The 2D area (Size.X * Size.Y), used purely as a relative cost metric by <see cref="ChooseSlot"/>.</summary>
    private static float Area(in AABB aabb)
    {
        Vector2 size = aabb.Size;
        return size.X * size.Y;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int EncodeLocation(int nodeIndex, int slot) => (nodeIndex << 2) | slot;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int DecodeNode(int location) => location >> 2;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int DecodeSlot(int location) => location & 3;

    /// <summary>
    /// Grows <paramref name="array"/>, if needed, so index <paramref name="requiredCount"/> - 1 is
    /// valid, filling any newly added elements with -1. The -1-filling counterpart of
    /// <see cref="QBVH2dNode.EnsureCapacity"/>, shared by <see cref="ShapeLeafLocation"/> and
    /// <see cref="ParentSlot"/>, both of which use -1 as their "not present" sentinel.
    /// </summary>
    private static void EnsureCapacityFilled(ref int[] array, int requiredCount)
    {
        if (requiredCount <= array.Length) return;
        int oldLength = array.Length;
        int newSize = Math.Max(array.Length == 0 ? 4 : array.Length * 2, requiredCount);
        Array.Resize(ref array, newSize);
        Array.Fill(array, -1, oldLength, newSize - oldLength);
    }

    /// <summary>
    /// Walks a freshly built tree once, populating <paramref name="shapeLeafLocation"/> and
    /// <paramref name="parentSlot"/> from scratch. Called right after <see cref="Build{T}"/> or
    /// <see cref="Rebuild{T}"/> finishes, since <see cref="QBVH2dNode.Build"/> doesn't track either
    /// table as a byproduct of construction.
    /// </summary>
    private static void IndexTree(QBVH2dNode[] nodes, int nodeCount, int[] shapeLeafLocation, int[] parentSlot)
    {
        if (nodeCount == 0) return;
        parentSlot[RootIndex] = -1;

        int[] stack = ArrayPool<int>.Shared.Rent(64);
        try
        {
            int sp = 0;
            stack[sp++] = RootIndex;

            while (sp > 0)
            {
                int nodeIndex = stack[--sp];
                ref var node = ref nodes[nodeIndex];
                int mask = node.Flags & 0xF;

                while (mask != 0)
                {
                    int slot = BitOperations.TrailingZeroCount(mask);
                    mask &= mask - 1;

                    if (node.IsChildLeaf(slot))
                    {
                        shapeLeafLocation[node.GetChildIndex(slot)] = EncodeLocation(nodeIndex, slot);
                    }
                    else
                    {
                        int child = node.GetChildIndex(slot);
                        parentSlot[child] = EncodeLocation(nodeIndex, slot);
                        if (sp == stack.Length)
                        {
                            var bigger = ArrayPool<int>.Shared.Rent(stack.Length * 2);
                            stack.AsSpan(0, sp).CopyTo(bigger);
                            ArrayPool<int>.Shared.Return(stack);
                            stack = bigger;
                        }
                        stack[sp++] = child;
                    }
                }
            }
        }
        finally { ArrayPool<int>.Shared.Return(stack); }
    }
}