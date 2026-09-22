using System.Runtime.CompilerServices;

namespace QBVH2D;

/// <summary>
/// A read-only view over a single QBVH2D node, exposing its topology and child bounds without
/// leaking the internal <see cref="QBVH2dNode"/> representation (which stays free to change its
/// memory layout). Obtained via <see cref="QBVH2d.GetNode(int)"/>.
/// </summary>
/// <remarks>
/// Intended for custom traversals that the built-in Query* methods don't cover, e.g.:
/// k-nearest-neighbor / best-first search, queries against custom shapes (circles, capsules,
/// convex polygons), debug visualization of the tree, tree statistics/diagnostics, or
/// serialization. For anything expressible as "does this shape/point/segment intersect a
/// region", prefer the existing <see cref="QBVH2d.QueryPoint(System.Numerics.Vector2)"/>,
/// <see cref="QBVH2d.QueryAABB(AABB)"/>, or a ray query instead.
/// </remarks>
public readonly struct QBVHNodeView
{
    private readonly QBVH2dNode _node;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal QBVHNodeView(in QBVH2dNode node)
    {
        _node = node;
    }

    /// <summary>
    /// Whether this node is a leaf (holds a single shape) rather than an internal node with children.
    /// </summary>
    public bool IsLeaf
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _node.IsLeaf;
    }

    /// <summary>
    /// The shape index this node refers to. Only meaningful when <see cref="IsLeaf"/> is <see langword="true"/>.
    /// </summary>
    public int ShapeIndex
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _node.ShapeIndex;
    }

    /// <summary>
    /// Checks if a child exists at the given slot. Only meaningful when <see cref="IsLeaf"/> is
    /// <see langword="false"/>.
    /// </summary>
    /// <param name="slot">Child slot index, 0-3</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasChild(int slot) => _node.HasChild(slot);

    /// <summary>
    /// Gets the node index of the child at the given slot, for use with <see cref="QBVH2d.GetNode(int)"/>
    /// to descend into it. Check <see cref="HasChild"/> first; a slot with no child returns a negative index.
    /// </summary>
    /// <param name="slot">Child slot index, 0-3</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetChildIndex(int slot) => _node.GetChildIndex(slot);

    /// <summary>
    /// Gets the bounding boxes of all 4 child slots at once, matching the layout used internally
    /// for SIMD queries. A slot without a child (see <see cref="HasChild"/>) has an unspecified
    /// AABB and should be ignored.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void GetChildAABBs(out AABB aabb0, out AABB aabb1, out AABB aabb2, out AABB aabb3) =>
        _node.GetChildAABBRefs(out aabb0, out aabb1, out aabb2, out aabb3);
}