namespace QBVH2D;

/// <summary>
/// Interface for objects that have an AABB
/// </summary>
public interface IBounded
{
    /// <summary>
    /// Gets the axis-aligned bounding box (AABB) of this object.
    /// </summary>
    /// <returns>
    /// An <see cref="AABB"/> that represents the object's axis-aligned bounding box.
    /// </returns>
    AABB GetAABB();
}