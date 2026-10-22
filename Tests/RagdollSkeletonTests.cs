using System.Numerics;
using RaylibGameFramework.Character;
using Xunit;

public sealed class RagdollSkeletonTests
{
    internal static (string[] Names, int[] Parents, Matrix4x4[] Pose) Rig()
    {
        var points = RagdollTests.Pose();
        string[] names = ["Root", .. Ragdoll.RequiredBones, "Finger.L"];
        int[] parents = [-1, 0, 1, 2, 3, 2, 5, 6, 2, 8, 9, 1, 11, 12, 1, 14, 15, 7];
        var pose = names.Select(n => n == "Root" ? Matrix4x4.Identity :
            n == "Finger.L" ? Matrix4x4.CreateTranslation(.8f, 1.3f, 0) : points[n]).ToArray();
        return (names, parents, pose);
    }

    // Managed counterparts of the source native-mesh test's pose/offset assertions.
    [Theory]
    [InlineData(37f, false)]
    [InlineData(-113f, false)]
    [InlineData(180f, false)]
    [InlineData(73f, true)]
    [InlineData(-47f, true)]
    public void ActivationPreservesPoseAndBodiesKeepCapturedOffsets(float yaw, bool nonuniform)
    {
        var (names, parents, initial) = Rig();
        var world = Matrix4x4.CreateScale(nonuniform ? new Vector3(1.1f, .9f, 1.2f) : Vector3.One) *
            Matrix4x4.CreateRotationY(yaw * MathF.PI / 180) * Matrix4x4.CreateTranslation(7, 3, -11);
        var doll = new Ragdoll();
        var bridge = RagdollPose.Activate(names, parents, initial, initial, world, doll, Vector3.Zero);
        Assert.Equal(12, bridge.BodyIndices.Count(i => i >= 0));
        var offsets = new Matrix4x4[names.Length];
        for (int i = 0; i < names.Length; i++)
        {
            Close(initial[i], bridge.ModelPose[i]);
            int body = bridge.BodyIndices[i];
            if (body < 0) continue;
            Assert.Equal(names[i] == "Root" ? "Hips" : names[i], doll.Bodies[body].Bone);
            Matrix4x4.Invert(RagdollSkeleton.BodyWorld(doll.Bodies[body]), out var inverse);
            offsets[i] = initial[i] * world * inverse;
        }
        doll.ApplyImpulse(new(1, new(80, 60, -90), doll.Bodies[1].Position + Vector3.UnitX * .2f));
        for (int step = 0; step < 1800; step++)
        {
            doll.Update(Ragdoll.FixedStep);
            if (step % 60 != 0 && step != 1799) continue;
            bridge.Evaluate(doll);
            for (int i = 0; i < names.Length; i++)
            {
                int body = bridge.BodyIndices[i];
                if (body >= 0)
                {
                    Matrix4x4.Invert(RagdollSkeleton.BodyWorld(doll.Bodies[body]), out var inverse);
                    Close(offsets[i], bridge.ModelPose[i] * world * inverse, .0005f);
                }
                else if (parents[i] >= 0)
                {
                    Matrix4x4.Invert(initial[parents[i]], out var originalParent);
                    Matrix4x4.Invert(bridge.ModelPose[parents[i]], out var currentParent);
                    Close(initial[i] * originalParent, bridge.ModelPose[i] * currentParent, .0005f);
                }
            }
        }
        Assert.Equal(RagdollState.Settled, doll.State);
        Assert.True(Vector3.Distance(initial[4].Translation, bridge.ModelPose[4].Translation) > .2f);
        doll.Deactivate();
        Assert.Throws<InvalidOperationException>(() => bridge.Evaluate(doll));
    }

    [Fact]
    public void InvalidHierarchyAndStaleActivationAreRejected()
    {
        var (names, parents, pose) = Rig();
        var doll = new Ragdoll();
        var bridge = RagdollPose.Activate(names, parents, pose, pose, Matrix4x4.Identity, doll, Vector3.Zero);
        var cycle = parents.ToArray(); cycle[0] = 1;
        Assert.Throws<ArgumentException>(() => new RagdollSkeleton(names, cycle, pose, Matrix4x4.Identity, doll));
        var invalid = parents.ToArray(); invalid[0] = names.Length;
        Assert.Throws<ArgumentException>(() => new RagdollSkeleton(names, invalid, pose, Matrix4x4.Identity, doll));
        RagdollPose.Activate(names, parents, pose, pose, Matrix4x4.Identity, doll, Vector3.Zero);
        Assert.Throws<InvalidOperationException>(() => bridge.Evaluate(doll));
        var nan = Matrix4x4.Identity; nan.M14 = float.NaN;
        Assert.Throws<ArgumentException>(() => new RagdollSkeleton(names, parents, pose, nan, doll));
    }

    internal static void Close(Matrix4x4 expected, Matrix4x4 actual, float tolerance = .0001f)
    {
        foreach (var point in new[] { Vector3.Zero, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ })
            Assert.True(Vector3.Distance(Vector3.Transform(point, expected), Vector3.Transform(point, actual)) < tolerance,
                $"Transform mismatch: {expected} vs {actual}");
    }
}
