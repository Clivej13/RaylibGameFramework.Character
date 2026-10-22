using System.Numerics;
using RaylibGameFramework.Character;
using Xunit;

public sealed class RagdollPoseTests
{
    [Fact]
    public void MissingSingularNonfiniteAndProjectivePosesLeaveActivationUnchanged()
    {
        var doll = RagdollContactTests.Create(new(0, 3, 0), Vector3.One);
        var original = doll.Bodies.ToArray();
        var reference = RagdollTests.Pose();
        var missing = RagdollTests.Pose(); missing.Remove("Hand.L");
        Assert.Throws<ArgumentException>(() => doll.Activate(missing, reference, Vector3.Zero));
        var nan = Matrix4x4.Identity; nan.M14 = float.NaN;
        var projective = Matrix4x4.Identity; projective.M24 = .1f;
        foreach (var invalid in new[] { default(Matrix4x4), nan, projective, Matrix4x4.CreateScale(-1, 1, 1) })
        {
            var pose = RagdollTests.Pose(); pose["Hips"] = invalid;
            Assert.Throws<ArgumentException>(() => doll.Activate(pose, reference, Vector3.Zero));
            Assert.Equal(original, doll.Bodies);
        }
        Assert.Throws<ArgumentException>(() => doll.Activate(reference, reference, new(float.PositiveInfinity, 0, 0)));
        Assert.Throws<ArgumentException>(() => doll.Activate(reference, reference, Vector3.Zero, groundHeight: float.NaN));
        Assert.Throws<ArgumentException>(() => doll.Activate(reference, reference, Vector3.Zero, new(99, Vector3.Zero)));
        Assert.Equal(original, doll.Bodies);
    }

    [Fact]
    public void InvalidSettingsImpulsesTimesAndDriveInputsAreRejected()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new Ragdoll(new() { Gravity = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Ragdoll(new() { ChestMass = 0 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Ragdoll(new() { DriveBlendIn = 0 }));
        Assert.Throws<ArgumentException>(() => new Ragdoll(new() { KneeLimits = new() { MinimumDegrees = Vector3.One } }));
        var doll = RagdollContactTests.Create(new(0, 10, 0), Vector3.Zero);
        Assert.Throws<ArgumentException>(() => doll.ApplyImpulse(new(-1, Vector3.Zero)));
        Assert.Throws<ArgumentException>(() => doll.ApplyImpulse(new(0, new(float.NaN, 0, 0))));
        Assert.Throws<ArgumentException>(() => doll.ApplyImpulse(new(0, Vector3.Zero, new(0, float.PositiveInfinity, 0))));
        Assert.Throws<ArgumentOutOfRangeException>(() => doll.Update(float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => doll.Update(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => doll.ReactToContact(0, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => doll.StartActiveDrive(_ => [], 99));
        Assert.False(doll.IsActivelyDriven);
        doll.StartActiveDrive(_ => []);
        Assert.Throws<ArgumentException>(() => doll.Update(Ragdoll.FixedStep));
        doll.StartActiveDrive(_ => Enumerable.Repeat(default(Quaternion), doll.Joints.Count).ToArray());
        Assert.Throws<ArgumentException>(() => doll.Update(Ragdoll.FixedStep));
    }

    [Fact]
    public void GroundContactIsGeneralAndLatchedUntilReset()
    {
        var doll = RagdollContactTests.Create(new(0, -.1f, 0), Vector3.Zero);
        Assert.True(doll.IsOnGround()); // feet count
        Assert.False(doll.HasGroundContact); // tick history starts empty
        doll.Update(Ragdoll.FixedStep);
        Assert.True(doll.HasGroundContact);
        Assert.False(doll.HasMeaningfulGroundContact);
        doll.Deactivate();
        Assert.False(doll.HasGroundContact);
        Assert.False(doll.IsOnGround());
    }

    [Fact]
    public void CatchUpIsBoundedToThirtyFixedSteps()
    {
        var a = RagdollContactTests.Create(new(0, 10, 0), Vector3.Zero);
        var b = RagdollContactTests.Create(new(0, 10, 0), Vector3.Zero);
        a.Update(10);
        for (int i = 0; i < 30; i++) b.Update(Ragdoll.FixedStep);
        Assert.Equal(a.Bodies.Select(x => x.Position), b.Bodies.Select(x => x.Position));
    }
}
