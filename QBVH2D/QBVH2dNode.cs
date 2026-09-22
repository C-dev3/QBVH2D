using System.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace QBVH2D;

/// <summary>
/// QBVH2D node - an internal node with up to 4 children. Each child slot holds either the index
/// of another <see cref="QBVH2dNode"/>, or - when <see cref="IsChildLeaf"/> is set for that slot -
/// a shape index directly (a "direct-encoded" leaf). Direct-encoded leaves don't consume a node
/// of their own, so a group of up to <see cref="MaxLeafSize"/> shapes fits in a single node with
/// no children nodes at all.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 4)]
internal struct QBVH2dNode
{
    // Bit flags: bits 0-3 indicate which of the 4 child slots are occupied, bits 4-7 indicate
    // (for each occupied slot) whether it's a direct-encoded leaf - in which case the
    // corresponding child index field holds a shape index - rather than another node's index.
    public byte Flags { get; set; }

    // Padding for alignment
    private readonly byte _padding1;
    private readonly byte _padding2;
    private readonly byte _padding3;

    private int _childIndex0;
    private int _childIndex1;
    private int _childIndex2;
    private int _childIndex3;

    // Child bounds stored SoA: one lane per child, 4-wide, so a query loads these vectors
    // directly instead of gathering/transposing 4 separate AABB structs at query time.
    private Vector128<float> _minX;
    private Vector128<float> _minY;
    private Vector128<float> _maxX;
    private Vector128<float> _maxY;

    /// <summary>
    /// Checks if a child at the specified index exists
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool HasChild(int index) => (Flags & (1 << index)) != 0;

    /// <summary>
    /// Checks whether the child at the specified slot is a direct-encoded leaf (its
    /// <see cref="GetChildIndex"/> value is a shape index) rather than another node's index.
    /// Only meaningful when <see cref="HasChild"/> is <see langword="true"/> for that slot.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool IsChildLeaf(int index) => (Flags & (1 << (index + 4))) != 0;

    /// <summary>
    /// Sets the flag indicating a child exists at the specified index
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChildFlag(int index) => Flags |= (byte)(1 << index);

    /// <summary>
    /// Sets the flag indicating the child at the specified index is a direct-encoded leaf
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChildLeafFlag(int index) => Flags |= (byte)(1 << (index + 4));

    /// <summary>
    /// Gets the value stored for the child at the specified slot: a node index when
    /// <see cref="IsChildLeaf"/> is <see langword="false"/>, or a shape index when it's
    /// <see langword="true"/>. Check <see cref="HasChild"/> first.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly int GetChildIndex(int index) => index switch
    {
        0 => _childIndex0,
        1 => _childIndex1,
        2 => _childIndex2,
        3 => _childIndex3,
        _ => -1
    };

    /// <summary>
    /// Sets the raw value (node index or shape index) of the child at the specified index
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChildIndex(int index, int value)
    {
        switch (index)
        {
            case 0: _childIndex0 = value; break;
            case 1: _childIndex1 = value; break;
            case 2: _childIndex2 = value; break;
            case 3: _childIndex3 = value; break;
        }
    }

    /// <summary>
    /// Packs the 4 children's bounds into the SoA layout (one lane per child)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChildBoundsSoA(in AABB c0, in AABB c1, in AABB c2, in AABB c3)
    {
        _minX = Vector128.Create(c0.Min.X, c1.Min.X, c2.Min.X, c3.Min.X);
        _minY = Vector128.Create(c0.Min.Y, c1.Min.Y, c2.Min.Y, c3.Min.Y);
        _maxX = Vector128.Create(c0.Max.X, c1.Max.X, c2.Max.X, c3.Max.X);
        _maxY = Vector128.Create(c0.Max.Y, c1.Max.Y, c2.Max.Y, c3.Max.Y);
    }

    /// <summary>
    /// Gets the 4 children's bounds already in SoA form (one lane per child), ready to pass
    /// straight into the <see cref="AABB"/> SIMD query helpers with no further transposing.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void GetChildBoundsSoA(out Vector128<float> minX, out Vector128<float> minY, out Vector128<float> maxX, out Vector128<float> maxY)
    {
        minX = _minX;
        minY = _minY;
        maxX = _maxX;
        maxY = _maxY;
    }

    private const float Epsilon = 0.00001f;
    private const int MaxLeafSize = 4; // Maximum shapes in a leaf group before splitting further

    /// <summary>
    /// Sentinel returned/passed for an empty child slot. Distinguishable from every real
    /// encoded value: a node index is always &gt;= 0, and a leaf encoding (~shapeIndex) never
    /// reaches <see cref="int.MinValue"/> for any realistic shape count.
    /// </summary>
    private const int NoChild = int.MinValue;

    /// <summary>
    /// Writes one child slot from an encoded value produced by <see cref="Build"/>:
    /// non-negative means an internal node index, negative (other than <see cref="NoChild"/>)
    /// means a direct-encoded leaf whose shape index is <c>~encoded</c>. <see cref="NoChild"/>
    /// leaves the slot's exist/leaf flags unset.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetChild(int slot, int encoded)
    {
        if (encoded == NoChild) return;

        SetChildFlag(slot);

        if (encoded < 0)
        {
            SetChildLeafFlag(slot);
            SetChildIndex(slot, ~encoded);
        }
        else
        {
            SetChildIndex(slot, encoded);
        }
    }

    /// <summary>
    /// Creates an internal node with up to 4 children. Each child parameter is an encoded value
    /// as returned by <see cref="Build"/>: <see cref="NoChild"/> for an empty slot, a
    /// non-negative node index, or a negative direct-encoded leaf (<c>~shapeIndex</c>).
    /// </summary>
    public static QBVH2dNode CreateNode(
        int child0, AABB child0AABB,
        int child1, AABB child1AABB,
        int child2, AABB child2AABB,
        int child3, AABB child3AABB)
    {
        var node = new QBVH2dNode();

        node.SetChild(0, child0);
        node.SetChild(1, child1);
        node.SetChild(2, child2);
        node.SetChild(3, child3);

        node.SetChildBoundsSoA(in child0AABB, in child1AABB, in child2AABB, in child3AABB);

        return node;
    }

    /// <summary>
    /// Ensures the node array can hold at least <paramref name="requiredCount"/> nodes,
    /// doubling its size (or growing to <paramref name="requiredCount"/>, whichever is
    /// larger) when it's too small. No-op when the array already has enough room.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void EnsureCapacity(ref QBVH2dNode[] nodes, int requiredCount)
    {
        if (requiredCount <= nodes.Length) return;

        int newSize = nodes.Length == 0 ? 4 : nodes.Length * 2;
        if (newSize < requiredCount) newSize = requiredCount;
        Array.Resize(ref nodes, newSize);
    }

    /// <summary>
    /// Builds a QBVH (sub)tree recursively using spatial subdivision.
    /// </summary>
    /// <returns>
    /// An encoded value identifying the root of what was just built: a non-negative
    /// <c>nodes</c> index for an internal node, or a negative direct-encoded leaf
    /// (<c>~shapeIndex</c>) when <paramref name="indices"/> held a single shape and no node
    /// needed to be created at all. Callers that store this value in a parent's child slot can
    /// pass it straight to <see cref="CreateNode"/> unchanged.
    /// </returns>
    public static int Build<T>(T[] shapes, ReadOnlySpan<int> indices, ref QBVH2dNode[] nodes, ref int nodeCount)
    where T : IBounded
    {
        if (indices.Length == 1)
        {
            // Single shape: encode it directly, no node consumed.
            return ~indices[0];
        }

        if (indices.Length <= MaxLeafSize)
        {
            return BuildLeafGroup(shapes, indices, ref nodes, ref nodeCount);
        }

        var aabbBounds = AABB.Empty;
        var centroidBounds = AABB.Empty;

        foreach (int index in indices)
        {
            var aabb = shapes[index].GetAABB();
            var center = aabb.Center;
            aabbBounds.JoinMut(in aabb);
            centroidBounds.GrowMut(in center);
        }

        // Reserve node index (may grow the array)
        EnsureCapacity(ref nodes, nodeCount + 1);
        int nodeIndex = nodeCount++;

        var center2d = centroidBounds.Center;
        var size = centroidBounds.Size;

        if (size.X < Epsilon && size.Y < Epsilon)
        {
            return BuildBySplitting(shapes, indices, ref nodes, ref nodeCount, nodeIndex);
        }

        var bucket0 = ArrayPool<int>.Shared.Rent(indices.Length);
        var bucket1 = ArrayPool<int>.Shared.Rent(indices.Length);
        var bucket2 = ArrayPool<int>.Shared.Rent(indices.Length);
        var bucket3 = ArrayPool<int>.Shared.Rent(indices.Length);

        int count0 = 0, count1 = 0, count2 = 0, count3 = 0;

        try
        {
            foreach (int idx in indices)
            {
                var shapeCenter = shapes[idx].GetAABB().Center;

                bool isRight = shapeCenter.X >= center2d.X;
                bool isTop = shapeCenter.Y >= center2d.Y;

                if (!isRight && !isTop)
                    bucket0[count0++] = idx;
                else if (isRight && !isTop)
                    bucket1[count1++] = idx;
                else if (!isRight && isTop)
                    bucket2[count2++] = idx;
                else
                    bucket3[count3++] = idx;
            }

            int child0 = NoChild, child1 = NoChild, child2 = NoChild, child3 = NoChild;
            AABB child0AABB = AABB.Empty, child1AABB = AABB.Empty;
            AABB child2AABB = AABB.Empty, child3AABB = AABB.Empty;

            if (count0 > 0)
            {
                child0AABB = Utils.JointAABBOfShapes(bucket0.AsSpan(0, count0), shapes);
                child0 = Build(shapes, bucket0.AsSpan(0, count0), ref nodes, ref nodeCount);
            }

            if (count1 > 0)
            {
                child1AABB = Utils.JointAABBOfShapes(bucket1.AsSpan(0, count1), shapes);
                child1 = Build(shapes, bucket1.AsSpan(0, count1), ref nodes, ref nodeCount);
            }

            if (count2 > 0)
            {
                child2AABB = Utils.JointAABBOfShapes(bucket2.AsSpan(0, count2), shapes);
                child2 = Build(shapes, bucket2.AsSpan(0, count2), ref nodes, ref nodeCount);
            }

            if (count3 > 0)
            {
                child3AABB = Utils.JointAABBOfShapes(bucket3.AsSpan(0, count3), shapes);
                child3 = Build(shapes, bucket3.AsSpan(0, count3), ref nodes, ref nodeCount);
            }

            // nodes may have been reallocated by any of the recursive calls above;
            // `nodes` here always refers to the current array because it's a ref parameter.
            nodes[nodeIndex] = CreateNode(
                child0, child0AABB,
                child1, child1AABB,
                child2, child2AABB,
                child3, child3AABB
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
    /// Builds nodes by simply splitting indices into 4 groups (used when the centroids all
    /// coincide, so spatial bucketing can't separate them)
    /// </summary>
    private static int BuildBySplitting<T>(T[] shapes, ReadOnlySpan<int> indices,
    ref QBVH2dNode[] nodes, ref int nodeCount, int nodeIndex) where T : IBounded
    {
        int quarterSize = indices.Length / 4;
        int remainder = indices.Length % 4;

        var sizes = new int[4];
        sizes[0] = quarterSize + (remainder > 0 ? 1 : 0);
        sizes[1] = quarterSize + (remainder > 1 ? 1 : 0);
        sizes[2] = quarterSize + (remainder > 2 ? 1 : 0);
        sizes[3] = quarterSize;

        int offset = 0;
        int child0 = NoChild, child1 = NoChild, child2 = NoChild, child3 = NoChild;
        AABB child0AABB = AABB.Empty, child1AABB = AABB.Empty;
        AABB child2AABB = AABB.Empty, child3AABB = AABB.Empty;

        if (sizes[0] > 0)
        {
            var span = indices.Slice(offset, sizes[0]);
            child0AABB = Utils.JointAABBOfShapes(span, shapes);
            child0 = Build(shapes, span, ref nodes, ref nodeCount);
            offset += sizes[0];
        }

        if (sizes[1] > 0)
        {
            var span = indices.Slice(offset, sizes[1]);
            child1AABB = Utils.JointAABBOfShapes(span, shapes);
            child1 = Build(shapes, span, ref nodes, ref nodeCount);
            offset += sizes[1];
        }

        if (sizes[2] > 0)
        {
            var span = indices.Slice(offset, sizes[2]);
            child2AABB = Utils.JointAABBOfShapes(span, shapes);
            child2 = Build(shapes, span, ref nodes, ref nodeCount);
            offset += sizes[2];
        }

        if (sizes[3] > 0)
        {
            var span = indices.Slice(offset, sizes[3]);
            child3AABB = Utils.JointAABBOfShapes(span, shapes);
            child3 = Build(shapes, span, ref nodes, ref nodeCount);
        }

        nodes[nodeIndex] = CreateNode(
            child0, child0AABB,
            child1, child1AABB,
            child2, child2AABB,
            child3, child3AABB
        );

        return nodeIndex;
    }

    /// <summary>
    /// Builds a single node holding up to <see cref="MaxLeafSize"/> shapes as direct-encoded
    /// leaves in its child slots - no child nodes are created at all.
    /// </summary>
    private static int BuildLeafGroup<T>(T[] shapes, ReadOnlySpan<int> indices,
    ref QBVH2dNode[] nodes, ref int nodeCount) where T : IBounded
    {
        EnsureCapacity(ref nodes, nodeCount + 1);
        int nodeIndex = nodeCount++;

        int child0 = NoChild, child1 = NoChild, child2 = NoChild, child3 = NoChild;
        AABB child0AABB = AABB.Empty, child1AABB = AABB.Empty;
        AABB child2AABB = AABB.Empty, child3AABB = AABB.Empty;

        for (int i = 0; i < indices.Length && i < MaxLeafSize; i++)
        {
            int idx = indices[i];
            var aabb = shapes[idx].GetAABB();
            int encoded = ~idx; // direct-encoded leaf

            switch (i)
            {
                case 0: child0 = encoded; child0AABB = aabb; break;
                case 1: child1 = encoded; child1AABB = aabb; break;
                case 2: child2 = encoded; child2AABB = aabb; break;
                case 3: child3 = encoded; child3AABB = aabb; break;
            }
        }

        nodes[nodeIndex] = CreateNode(
            child0, child0AABB,
            child1, child1AABB,
            child2, child2AABB,
            child3, child3AABB
        );

        return nodeIndex;
    }
}