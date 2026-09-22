using BenchmarkDotNet.Running;

namespace QBVH2D.Bench;

internal static class Program
{
    private static void Main(string[] args)
    {
        BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
        Console.ReadKey();
    }
}