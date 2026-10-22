using System.Numerics;

namespace RaylibGameFramework.Character;

/// <summary>Managed pose snapshots; matrices are absolute model-space row-vector transforms.</summary>
public static class RagdollPose
{
    public static RagdollSkeleton Activate(IReadOnlyList<string> names, IReadOnlyList<int> parents,
        IReadOnlyList<Matrix4x4> pose, IReadOnlyList<Matrix4x4> referencePose,
        Matrix4x4 world, Ragdoll ragdoll, Vector3 velocity, RagdollImpulse? impulse = null, float groundHeight = 0)
    {
        if (names.Count != pose.Count || names.Count != referencePose.Count || names.Count != parents.Count)
            throw new ArgumentException("Skeleton and pose counts must match.");
        var current = names.Select((name, i) => (name, matrix: pose[i] * world))
            .ToDictionary(p => p.name, p => p.matrix);
        var reference = names.Select((name, i) => (name, matrix: referencePose[i] * world))
            .ToDictionary(p => p.name, p => p.matrix);
        ragdoll.Activate(current, reference, velocity, impulse, groundHeight);
        try { return new(names, parents, pose, world, ragdoll); }
        catch { ragdoll.Deactivate(); throw; }
    }
}
