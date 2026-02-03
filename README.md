[![NuGet Version](https://img.shields.io/nuget/v/QBVH2D)](https://www.nuget.org/packages/QBVH2D/)
[![NuGet Downloads](https://img.shields.io/nuget/dt/QBVH2D)](https://www.nuget.org/packages/QBVH2D/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# QBVH2D

A high-performance 2D Quad Bounding Volume Hierarchy (QBVH) library for .NET, optimized for efficient spatial queries with SIMD acceleration.

## Features

- **4-way branching** instead of binary trees for better cache utilization and reduced tree depth
- **SIMD-accelerated** collision detection using SSE instructions
- **Zero-allocation** query APIs using `Span<T>` and stackalloc
- **Memory-efficient** structure with ArrayPool for temporary allocations
- **Flexible querying** with point containment and AABB intersection tests
- **Iterator pattern** for lazy evaluation of query results

## Installation

Install via NuGet Package Manager:

```bash
dotnet add package QBVH2D
```

Or via Package Manager Console:

```powershell
Install-Package QBVH2D
```

## Quick Start

### 1. Define Your Shape

Implement the `IBounded` interface for your shapes:

```csharp
using QBVH2D;
using System.Numerics;

public class Circle : IBounded
{
    public Vector2 Position { get; set; }
    public float Radius { get; set; }

    public AABB GetAABB()
    {
        var min = new Vector2(Position.X - Radius, Position.Y - Radius);
        var max = new Vector2(Position.X + Radius, Position.Y + Radius);
        return new AABB(min, max);
    }
}
```

### 2. Build the QBVH

```csharp
// Create your shapes
var shapes = new Circle[]
{
    new Circle { Position = new Vector2(10, 10), Radius = 5 },
    new Circle { Position = new Vector2(30, 20), Radius = 8 },
    new Circle { Position = new Vector2(50, 15), Radius = 6 },
    // ... more shapes
};

// Build the QBVH
var qbvh = QBVH2d.Build(shapes);
```

### 3. Query Points

```csharp
var queryPoint = new Vector2(15, 12);

// Option 1: Get all results as a List
List<int> results = qbvh.QueryPoint(queryPoint);

// Option 2: Use a pre-allocated Span (zero allocation)
Span<int> resultBuffer = stackalloc int[16];
int count = qbvh.QueryPoint(queryPoint, resultBuffer);

// Option 3: Use an iterator for lazy evaluation
using var iterator = qbvh.ContainsIterator(queryPoint);
foreach (var shapeIndex in iterator)
{
    var shape = shapes[shapeIndex];
    // Process shape...
}
```

### 4. Query AABBs

```csharp
var queryAABB = new AABB(
    new Vector2(0, 0),   // min
    new Vector2(100, 100) // max
);

List<int> intersectingShapes = qbvh.QueryAABB(queryAABB);
```

## API Reference

### QBVH2d Class

#### Static Methods

- **`Build<T>(T[] shapes)`** - Builds a QBVH from an array of shapes implementing `IBounded`

#### Query Methods

- **`QueryPoint(Vector2 point)`** - Returns a `List<int>` of shape indices containing the point
- **`QueryPoint(Vector2 point, Span<int> results)`** - Writes results to a span, returns count
- **`QueryPoint(Vector2 point, ref List<int> results)`** - Appends results to an existing list
- **`QueryAABB(AABB aabb)`** - Returns shape indices intersecting with the given AABB
- **`ContainsIterator(Vector2 point)`** - Returns an iterator for lazy evaluation

### AABB Struct

Represents an axis-aligned bounding box in 2D space.

#### Properties

- **`Vector2 Min`** - Minimum corner (bottom-left)
- **`Vector2 Max`** - Maximum corner (top-right)
- **`Vector2 Size`** - Size of the AABB
- **`Vector2 Center`** - Center point of the AABB

#### Methods

- **`Contains(Vector2 point)`** - Checks if a point is inside the AABB
- **`Intersects(AABB other)`** - Checks if two AABBs intersect

#### Static Members

- **`AABB.Empty`** - Sentinel value representing an empty AABB

### IBounded Interface

Implement this interface for any shape you want to store in the QBVH.

```csharp
public interface IBounded
{
    AABB GetAABB();
}
```

## Performance Tips

### Use Span-based APIs

For hot paths, use the `Span<T>` overloads to avoid allocations:

```csharp
Span<int> results = stackalloc int[32];
int count = qbvh.QueryPoint(point, results);
```

### Reuse Lists

When querying multiple points, reuse the same list:

```csharp
var results = new List<int>(32);
foreach (var point in queryPoints)
{
    results.Clear();
    qbvh.QueryPoint(point, ref results);
    // Process results...
}
```

### Use Iterators for Early Exit

If you only need the first few results:

```csharp
using var iterator = qbvh.ContainsIterator(point);
foreach (var index in iterator)
{
    if (CheckCollision(shapes[index], point))
    {
        // Found what we need, iterator will be disposed
        break;
    }
}
```

### Batch Updates

If your shapes move frequently, rebuild the QBVH periodically rather than on every frame:

```csharp
// Update every N frames or when movement exceeds threshold
if (frameCount % rebuildInterval == 0)
{
    qbvh = QBVH2d.Build(shapes);
}
```

## System Requirements

- .NET 6.0 or higher
- x86/x64 processor with SSE support (automatic fallback for other architectures)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.txt) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

- Report bugs via [GitHub Issues](https://github.com/yourusername/QBVH2D/issues)

## Acknowledgments

- Inspired by QBVH implementations in real-time rendering and physics engines
- Optimized using best practices from the .NET performance team
