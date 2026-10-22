using System.Numerics;
using RaylibGameFramework.Character;
using Xunit;

public sealed class RagdollRecoveryTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BlendStartsExactlyAtPhysicsAndEndsAtSampledPose(bool shear)
    {
        var startRoot = Matrix4x4.CreateRotationZ(.4f) * Matrix4x4.CreateTranslation(2, 1, 0);
        if (shear) startRoot.M12 += .3f;
        Matrix4x4[] start = [startRoot, Matrix4x4.CreateTranslation(0, 1, 0) * startRoot];
        Matrix4x4[] target = [Matrix4x4.Identity, Matrix4x4.CreateTranslation(0, 2, 0)];
        var recovery = new RagdollRecovery([-1, 0], start, (_, _) => target, 1, .5f, 2);
        for (int i = 0; i < 2; i++) RagdollSkeletonTests.Close(start[i], recovery.ModelPose[i]);
        start[0] = Matrix4x4.Identity; // constructor owns a snapshot
        recovery.Update(.25f);
        Assert.NotEqual(target[0], recovery.ModelPose[0]);
        Assert.All(recovery.ModelPose, p => Assert.True(float.IsFinite(p.GetDeterminant())));
        recovery.Update(.25f);
        Assert.Equal(target, recovery.ModelPose);
        recovery.Update(.5f);
        Assert.Equal(RecoveryPhase.GetUp, recovery.Phase);
        Assert.Equal(0, recovery.PhaseTime);
        recovery.Update(2);
        Assert.Equal(RecoveryPhase.Complete, recovery.Phase);
        Assert.Equal(target, recovery.ModelPose);
        recovery.Update(100);
        Assert.Equal(2, recovery.PhaseTime);
    }

    [Fact]
    public void LargeAndPartitionedUpdatesReachSameSampleTime()
    {
        IReadOnlyList<Matrix4x4> Sample(RecoveryPhase phase, float t) =>
            new[] { Matrix4x4.CreateTranslation(t, phase == RecoveryPhase.Down ? 0 : 1, 0) };
        var a = new RagdollRecovery([-1], [Matrix4x4.Identity], Sample, 1, .5f, 2);
        var b = new RagdollRecovery([-1], [Matrix4x4.Identity], Sample, 1, .5f, 2);
        a.Update(2.5f);
        for (int i = 0; i < 10; i++) b.Update(.25f);
        Assert.Equal(a.Phase, b.Phase);
        Assert.Equal(a.PhaseTime, b.PhaseTime);
        Assert.Equal(a.ModelPose, b.ModelPose);
    }

    [Fact]
    public void InvalidRecoveryInputsAreRejected()
    {
        Matrix4x4[] pose = [Matrix4x4.Identity];
        Assert.Throws<ArgumentOutOfRangeException>(() => new RagdollRecovery([-1], pose, (_, _) => pose, 1, 2, 1));
        Assert.Throws<ArgumentException>(() => new RagdollRecovery([0], pose, (_, _) => pose, 1, 1, 1));
        var recovery = new RagdollRecovery([-1], pose, (_, _) => pose, 1, 1, 1);
        Assert.Throws<ArgumentOutOfRangeException>(() => recovery.Update(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => recovery.Update(-1));
        Assert.Throws<ArgumentException>(() => new RagdollRecovery([-1], pose, (_, _) => [], 1, 1, 1));
    }
}
