using System.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace QBVH2D;

/// <summary>
/// QBVH2D node - can be either a leaf or an internal node with up to 4 children
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 4)]
internal struct QBVH2dNode
{
    // Leaf properties
    public int ShapeIndex { get; set; }

    // Bit flags: bits 0-3 indicate which children exist, bit 4 indicates if this is a leaf
    public byte Flags { get; set; }

    // Padding for alignment
    private readonly byte _padding1;
    private readonly byte _padding2;
    private readonly byte _padding3;

    private unsafe fixed int _childIndices[4];

    private AABB _child0AABB;
    private AABB _child1AABB;
    private AABB _child2AABB;
    private AABB _child3AABB;

    /// <summary>
    /// Gets or sets whether this node is a leaf node
    /// </summary>
    public bool IsLeaf
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => (Flags & 0x10) != 0;
        set => Flags = value ? (byte)(Flags | 0x10) : (byte)(Flags & ~0x10);
    }

    /// <summary>
    /// Checks if a child at the specified index exists
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool HasChild(int index) => (Flags & (1 << index)) != 0;

    /// <summary>
    /// Sets the flag indicating a child exists at the specified index
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChildFlag(int index) => Flags |= (byte)(1 << index);

    /// <summary>
    /// Gets the node index of the child at the specified index
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly unsafe int GetChildIndex(int index)
    {
        if (index < 0 || index > 3) return -1;
        fixed (int* ptr = _childIndices)
        {
            return ptr[index];
        }
    }

    /// <summary>
    /// Sets the node index of the child at the specified index
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private unsafe void SetChildIndex(int index, int value)
    {
        if (index < 0 || index > 3) return;
        fixed (int* ptr = _childIndices)
        {
            ptr[index] = value;
        }
    }

    /// <summary>
    /// Sets the AABB of the child at the specified index
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChildAABB(int index, in AABB aabb)
    {
        switch (index)
        {
            case 0: _child0AABB = aabb; break;
            case 1: _child1AABB = aabb; break;
            case 2: _child2AABB = aabb; break;
            case 3: _child3AABB = aabb; break;
        }
    }

    /// <summary>
    /// Gets references to all 4 child AABBs for optimized SIMD operations
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly unsafe void GetChildAABBRefs(out AABB aabb0, out AABB aabb1, out AABB aabb2, out AABB aabb3)
    {
        aabb0 = _child0AABB;
        aabb1 = _child1AABB;
        aabb2 = _child2AABB;
        aabb3 = _child3AABB;
    }

    private const float Epsilon = 0.00001f;
    private const int MaxLeafSize = 4; // Maximum shapes in a leaf before splitting

    /// <summary>
    /// Creates a leaf node
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static QBVH2dNode CreateLeaf(int shapeIndex) => new()
    {
        IsLeaf = true,
        ShapeIndex = shapeIndex
    };

    /// <summary>
    /// Creates an internal node with up to 4 children
    /// </summary>
    public static QBVH2dNode CreateNode(
        int child0Idx, AABB child0AABB,
        int child1Idx, AABB child1AABB,
        int child2Idx, AABB child2AABB,
        int child3Idx, AABB child3AABB)
    {
        var node = new QBVH2dNode
        {
            IsLeaf = false
        };

        // Batch set child indices and AABBs using contiguous memory
        node.SetChildIndex(0, child0Idx);
        node.SetChildAABB(0, child0AABB);

        node.SetChildIndex(1, child1Idx);
        node.SetChildAABB(1, child1AABB);

        node.SetChildIndex(2, child2Idx);
        node.SetChildAABB(2, child2AABB);

        node.SetChildIndex(3, child3Idx);
        node.SetChildAABB(3, child3AABB);

        // Set flags for which children exist
        if (child0Idx >= 0) node.SetChildFlag(0);
        if (child1Idx >= 0) node.SetChildFlag(1);
        if (child2Idx >= 0) node.SetChildFlag(2);
        if (child3Idx >= 0) node.SetChildFlag(3);

        return node;
    }

    /// <summary>
    /// Builds a QBVH tree recursively using spatial subdivision
    /// </summary>
    public static int Build<T>(T[] shapes, ReadOnlySpan<int> indices, QBVH2dNode[] nodes, ref int nodeCount)
        where T : IBounded
    {
        // Create leaf for small groups
        if (indices.Length <= MaxLeafSize)
        {
            if (indices.Length == 1)
            {
                int shapeIndex = indices[0];
                int leafNodeIndex = nodeCount++;
                nodes[leafNodeIndex] = CreateLeaf(shapeIndex);
                return leafNodeIndex;
            }
            else
            {
                // Create multiple leaf nodes for remaining shapes
                return BuildMultipleLeaves(shapes, indices, nodes, ref nodeCount);
            }
        }

        // Compute bounds
        var aabbBounds = AABB.Empty;
        var centroidBounds = AABB.Empty;

        foreach (int index in indices)
        {
            var aabb = shapes[index].GetAABB();
            var center = aabb.Center;
            aabbBounds.JoinMut(in aabb);
            centroidBounds.GrowMut(in center);
        }

        // Reserve node index
        int nodeIndex = nodeCount++;

        // Split into 4 quadrants based on center
        var center2d = centroidBounds.Center;
        var size = centroidBounds.Size;

        // Check if we can actually split
        if (size.X < Epsilon && size.Y < Epsilon)
        {
            // All shapes at same location, split evenly
            return BuildBySplitting(shapes, indices, nodes, ref nodeCount, nodeIndex);
        }

        // Allocate buckets for 4 quadrants
        var bucket0 = ArrayPool<int>.Shared.Rent(indices.Length);
        var bucket1 = ArrayPool<int>.Shared.Rent(indices.Length);
        var bucket2 = ArrayPool<int>.Shared.Rent(indices.Length);
        var bucket3 = ArrayPool<int>.Shared.Rent(indices.Length);

        int count0 = 0, count1 = 0, count2 = 0, count3 = 0;

        try
        {
            // Distribute shapes into quadrants
            foreach (int idx in indices)
            {
                var shapeCenter = shapes[idx].GetAABB().Center;

                bool isRight = shapeCenter.X >= center2d.X;
                bool isTop = shapeCenter.Y >= center2d.Y;

                if (!isRight && !isTop)
                    bucket0[count0++] = idx; // Bottom-left
                else if (isRight && !isTop)
                    bucket1[count1++] = idx; // Bottom-right
                else if (!isRight && isTop)
                    bucket2[count2++] = idx; // Top-left
                else
                    bucket3[count3++] = idx; // Top-right
            }

            // Build children
            int child0Idx = -1, child1Idx = -1, child2Idx = -1, child3Idx = -1;
            AABB child0AABB = AABB.Empty, child1AABB = AABB.Empty;
            AABB child2AABB = AABB.Empty, child3AABB = AABB.Empty;

            if (count0 > 0)
            {
                child0AABB = Utils.JointAABBOfShapes(bucket0.AsSpan(0, count0), shapes);
                child0Idx = Build(shapes, bucket0.AsSpan(0, count0), nodes, ref nodeCount);
            }

            if (count1 > 0)
            {
                child1AABB = Utils.JointAABBOfShapes(bucket1.AsSpan(0, count1), shapes);
                child1Idx = Build(shapes, bucket1.AsSpan(0, count1), nodes, ref nodeCount);
            }

            if (count2 > 0)
            {
                child2AABB = Utils.JointAABBOfShapes(bucket2.AsSpan(0, count2), shapes);
                child2Idx = Build(shapes, bucket2.AsSpan(0, count2), nodes, ref nodeCount);
            }

            if (count3 > 0)
            {
                child3AABB = Utils.JointAABBOfShapes(bucket3.AsSpan(0, count3), shapes);
                child3Idx = Build(shapes, bucket3.AsSpan(0, count3), nodes, ref nodeCount);
            }

            // OPTIMIZATION: Node creation now uses contiguous memory layout
            nodes[nodeIndex] = CreateNode(
                child0Idx, child0AABB,
                child1Idx, child1AABB,
                child2Idx, child2AABB,
                child3Idx, child3AABB
            );

            return nodeIndex;
        }
        finally
        {
            ArrayPool<int>.Shared.Return(bucket0);
            ArrayPool<int>.Shared.Return(bucket1);
            ArrayPool<int>.Shared.Return(bucket2);
            ArrayPool<int>.Shared.Return(bucket3);
        }
    }

    /// <summary>
    /// Builds nodes by simply splitting indices into 4 groups
    /// </summary>
    private static int BuildBySplitting<T>(T[] shapes, ReadOnlySpan<int> indices,
        QBVH2dNode[] nodes, ref int nodeCount, int nodeIndex) where T : IBounded
    {
        int quarterSize = indices.Length / 4;
        int remainder = indices.Length % 4;

        var sizes = new int[4];
        sizes[0] = quarterSize + (remainder > 0 ? 1 : 0);
        sizes[1] = quarterSize + (remainder > 1 ? 1 : 0);
        sizes[2] = quarterSize + (remainder > 2 ? 1 : 0);
        sizes[3] = quarterSize;

        int offset = 0;
        int child0Idx = -1, child1Idx = -1, child2Idx = -1, child3Idx = -1;
        AABB child0AABB = AABB.Empty, child1AABB = AABB.Empty;
        AABB child2AABB = AABB.Empty, child3AABB = AABB.Empty;

        if (sizes[0] > 0)
        {
            var span = indices.Slice(offset, sizes[0]);
            child0AABB = Utils.JointAABBOfShapes(span, shapes);
            child0Idx = Build(shapes, span, nodes, ref nodeCount);
            offset += sizes[0];
        }

        if (sizes[1] > 0)
        {
            var span = indices.Slice(offset, sizes[1]);
            child1AABB = Utils.JointAABBOfShapes(span, shapes);
            child1Idx = Build(shapes, span, nodes, ref nodeCount);
            offset += sizes[1];
        }

        if (sizes[2] > 0)
        {
            var span = indices.Slice(offset, sizes[2]);
            child2AABB = Utils.JointAABBOfShapes(span, shapes);
            child2Idx = Build(shapes, span, nodes, ref nodeCount);
            offset += sizes[2];
        }

        if (sizes[3] > 0)
        {
            var span = indices.Slice(offset, sizes[3]);
            child3AABB = Utils.JointAABBOfShapes(span, shapes);
            child3Idx = Build(shapes, span, nodes, ref nodeCount);
        }

        nodes[nodeIndex] = CreateNode(
            child0Idx, child0AABB,
            child1Idx, child1AABB,
            child2Idx, child2AABB,
            child3Idx, child3AABB
        );

        return nodeIndex;
    }

    /// <summary>
    /// Creates multiple leaf nodes for a small group of shapes
    /// </summary>
    private static int BuildMultipleLeaves<T>(T[] shapes, ReadOnlySpan<int> indices,
        QBVH2dNode[] nodes, ref int nodeCount) where T : IBounded
    {
        int nodeIndex = nodeCount++;

        int child0Idx = -1, child1Idx = -1, child2Idx = -1, child3Idx = -1;
        AABB child0AABB = AABB.Empty, child1AABB = AABB.Empty;
        AABB child2AABB = AABB.Empty, child3AABB = AABB.Empty;

        for (int i = 0; i < indices.Length && i < 4; i++)
        {
            var idx = indices[i];
            var aabb = shapes[idx].GetAABB();
            int leafIdx = nodeCount++;
            nodes[leafIdx] = CreateLeaf(idx);

            switch (i)
            {
                case 0:
                    child0Idx = leafIdx;
                    child0AABB = aabb;
                    break;
                case 1:
                    child1Idx = leafIdx;
                    child1AABB = aabb;
                    break;
                case 2:
                    child2Idx = leafIdx;
                    child2AABB = aabb;
                    break;
                case 3:
                    child3Idx = leafIdx;
                    child3AABB = aabb;
                    break;
            }
        }

        nodes[nodeIndex] = CreateNode(
            child0Idx, child0AABB,
            child1Idx, child1AABB,
            child2Idx, child2AABB,
            child3Idx, child3AABB
        );

        return nodeIndex;
    }
}