using System.Buffers;
using System.Collections;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace QBVH2D;

/// <summary>
/// Iterator for traversing a QBVH2D tree to find shapes containing a point
/// </summary>
public sealed class QBVH2DTraverseIterator : IEnumerator<int>, IEnumerable<int>, IDisposable
{
    private readonly QBVH2d _qbvh2d;
    private readonly Vector2 _point;
    private int[] _stack;
    private int _stackSize;
    private int _current;
    private bool _disposed;

    private const int DefaultStackSize = 64;

    internal QBVH2DTraverseIterator(QBVH2d qbvh2d, Vector2 point)
    {
        _qbvh2d = qbvh2d;
        _point = point;
        _stack = ArrayPool<int>.Shared.Rent(DefaultStackSize); // Stack for traversal
        _stackSize = 0;
        _current = -1;
        _disposed = false;

        // Initialize with root if tree is not empty
        if (qbvh2d.NodeCount > 0)
        {
            _stack[_stackSize++] = 0;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsStackEmpty() => _stackSize == 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void StackPush(int node)
    {
        if (_stackSize >= _stack.Length)
        {
            var newStack = ArrayPool<int>.Shared.Rent(_stack.Length * 2);
            Array.Copy(_stack, newStack, _stack.Length);
            ArrayPool<int>.Shared.Return(_stack);
            _stack = newStack;
        }
        _stack[_stackSize++] = node;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int StackPop() => _stack[--_stackSize];

    /// <summary>
    /// Advances the iterator to the next shape whose bounding volume contains the query point.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the iterator successfully advanced to the next element;
    /// <see langword="false"/> if the traversal has completed.
    /// </returns>
    bool IEnumerator.MoveNext()
    {
        while (!IsStackEmpty())
        {
            int nodeIndex = StackPop();
            var node = _qbvh2d.Nodes[nodeIndex];

            if (node.IsLeaf)
            {
                // Found a leaf node
                _current = node.ShapeIndex;
                return true;
            }

            node.GetChildAABBRefs(out var aabb0, out var aabb1, out var aabb2, out var aabb3);
            int containsMask = AABB.Contains4(in _point, in aabb0, in aabb1, in aabb2, in aabb3);

            // Internal node - check all 4 children
            // Process in reverse order so that child 0 is processed first (LIFO stack)
            if ((containsMask & 8) != 0 && node.HasChild(3))
                StackPush(node.GetChildIndex(3));
            if ((containsMask & 4) != 0 && node.HasChild(2))
                StackPush(node.GetChildIndex(2));
            if ((containsMask & 2) != 0 && node.HasChild(1))
                StackPush(node.GetChildIndex(1));
            if ((containsMask & 1) != 0 && node.HasChild(0))
                StackPush(node.GetChildIndex(0));
        }

        return false;
    }

    /// <summary>
    /// Resets the iterator to its initial state, restarting the QBVH traversal from the root.
    /// </summary>
    public void Reset()
    {
        _stackSize = 0;
        _current = -1;

        if (_qbvh2d.NodeCount > 0)
        {
            _stack[_stackSize++] = 0;
        }
    }

    /// <summary>
    /// Gets the index of the current shape found during the traversal.
    /// </summary>
    public int Current
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _current;
    }

    object IEnumerator.Current => Current;

    /// <summary>
    /// <inheritdoc/>
    /// </summary>
    public void Dispose()
    {
        if (!_disposed)
        {
            if (_stack != null)
            {
                ArrayPool<int>.Shared.Return(_stack);
                _stack = null!;
            }
            _disposed = true;
        }
    }

    /// <summary>
    /// Returns an enumerator that iterates through the shape indices produced by this traversal.
    /// </summary>
    /// <returns>
    /// An <see cref="IEnumerator{T}"/> that iterates over shape indices.
    /// </returns>
    public IEnumerator<int> GetEnumerator() => this;

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}
