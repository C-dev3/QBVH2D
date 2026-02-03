using System.Runtime.CompilerServices;

namespace QBVH2D;

internal static class Utils
{
    /// <summary>
    /// Computes the joint AABB of shapes at specified indices
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static AABB JointAABBOfShapes<T>(ReadOnlySpan<int> indices, T[] shapes) where T : IBounded
    {
        var aabb = AABB.Empty;
        foreach (int index in indices)
        {
            var shapeAabb = shapes[index].GetAABB();
            aabb.JoinMut(in shapeAabb);
        }
        return aabb;
    }
}