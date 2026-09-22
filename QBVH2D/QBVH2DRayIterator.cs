using System.Buffers;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace QBVH2D;

/// <summary>
/// Iterator for traversing a QBVH2D tree to find shapes whose AABB intersects a ray or segment
/// </summary>
public ref struct QBVH2DRayIterator
{
    private readonly QBVH2d _qbvh2d;
    private readonly Vector2 _origin;
    private readonly Vector2 _invDir;
    private readonly float _maxT;
    private int[] _stack;
    private int _stackSize;
    private int _current;
    private bool _disposed;

    private const int DefaultStackSize = 64;

    internal QBVH2DRayIterator(QBVH2d qbvh2d, Vector2 origin, Vector2 direction, float maxT)
    {
        _qbvh2d = qbvh2d;
        _origin = origin;
        _invDir = AABB.InvDir(direction);
        _maxT = maxT;
        _stack = ArrayPool<int>.Shared.Rent(DefaultStackSize); // Stack for traversal
        _stackSize = 0;
        _current = -1;
        _disposed = false;

        PushRoot();
    }

    // Stack entries use the same encoding as QBVH2dNode.Build's return value: a non-negative
    // value is an internal node index, a negative value is a direct-encoded leaf whose shape
    // index is ~value. Pushing the root this way means a single-shape tree (no nodes at all,
    // just QBVH2d.RootLeafShapeIndex) is handled by the exact same pop logic as everything else.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PushRoot()
    {
        if (_qbvh2d.RootLeafShapeIndex >= 0)
        {
            _stack[_stackSize++] = ~_qbvh2d.RootLeafShapeIndex;
        }
        else if (_qbvh2d.NodeCount > 0)
        {
            _stack[_stackSize++] = QBVH2d.RootIndex;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly bool IsStackEmpty() => _stackSize == 0;

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
    /// Advances the iterator to the next shape whose bounding volume intersects the ray/segment.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the iterator successfully advanced to the next element;
    /// <see langword="false"/> if the traversal has completed.
    /// </returns>
    public bool MoveNext()
    {
        while (!IsStackEmpty())
        {
            int popped = StackPop();

            if (popped < 0)
            {
                // Direct-encoded leaf: the shape index is already known, no node to fetch.
                _current = ~popped;
                return true;
            }

            ref readonly var node = ref _qbvh2d.Nodes[popped]; // avoid copying the node struct

            node.GetChildBoundsSoA(out var minX, out var minY, out var maxX, out var maxY);
            int hitMask = AABB.IntersectsRay4(in _origin, in _invDir, _maxT, minX, minY, maxX, maxY);

            // Fold existence check into the hit mask with a single AND (Flags bits 0-3 mark which children exist)
            int mask = hitMask & node.Flags & 0xF;

            // Push children in reverse bit order so that child 0 is popped first (LIFO stack).
            // A child that's a direct-encoded leaf is pushed as ~shapeIndex so the next pop
            // resolves it immediately above, without ever touching the node array.
            if ((mask & 8) != 0) StackPush(node.IsChildLeaf(3) ? ~node.GetChildIndex(3) : node.GetChildIndex(3));
            if ((mask & 4) != 0) StackPush(node.IsChildLeaf(2) ? ~node.GetChildIndex(2) : node.GetChildIndex(2));
            if ((mask & 2) != 0) StackPush(node.IsChildLeaf(1) ? ~node.GetChildIndex(1) : node.GetChildIndex(1));
            if ((mask & 1) != 0) StackPush(node.IsChildLeaf(0) ? ~node.GetChildIndex(0) : node.GetChildIndex(0));
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

        PushRoot();
    }

    /// <summary>
    /// Gets the index of the current shape found during the traversal.
    /// </summary>
    public readonly int Current
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _current;
    }

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
    public readonly QBVH2DRayIterator GetEnumerator() => this;
}