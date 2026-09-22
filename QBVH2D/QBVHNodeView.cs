using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

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
    /// Checks if a child exists at the given slot.
    /// </summary>
    /// <param name="slot">Child slot index, 0-3</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasChild(int slot) => _node.HasChild(slot);

    /// <summary>
    /// Checks whether the child at the given slot is a direct-encoded leaf - i.e.
    /// <see cref="GetChildIndex"/> for that slot returns a shape index rather than another
    /// node's index. Only meaningful when <see cref="HasChild"/> is <see langword="true"/> for
    /// that slot; a leaf shape doesn't have its own node to descend into.
    /// </summary>
    /// <param name="slot">Child slot index, 0-3</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsChildLeaf(int slot) => _node.IsChildLeaf(slot);

    /// <summary>
    /// Gets the value stored for the child at the given slot: a node index (for use with
    /// <see cref="QBVH2d.GetNode(int)"/> to descend into it) when <see cref="IsChildLeaf"/> is
    /// <see langword="false"/>, or a shape index directly when it's <see langword="true"/>.
    /// Check <see cref="HasChild"/> first; a slot with no child returns an unspecified value.
    /// </summary>
    /// <param name="slot">Child slot index, 0-3</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetChildIndex(int slot) => _node.GetChildIndex(slot);

    /// <summary>
    /// Gets the bounding boxes of all 4 child slots at once, in SoA form (one lane per child)
    /// matching the layout used internally for SIMD queries: lane <c>i</c> across the four
    /// vectors is the AABB of child slot <c>i</c>. A slot without a child (see
    /// <see cref="HasChild"/>) has an unspecified AABB and should be ignored.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void GetChildBoundsSoA(out Vector128<float> minX, out Vector128<float> minY, out Vector128<float> maxX, out Vector128<float> maxY) =>
        _node.GetChildBoundsSoA(out minX, out minY, out maxX, out maxY);
}