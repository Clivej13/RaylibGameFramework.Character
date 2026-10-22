using System.Numerics;
using RaylibGameFramework.Character;
using Xunit;

public sealed class RagdollRigTests
{
    // Synthetic poses replace asset clips; solver assertions retain the source rig tolerances.
    [Theory]
    [InlineData(.137f, false)]
    [InlineData(.21f, true)]
    [InlineData(.31f, false)]
    [InlineData(.25f, true)]
    [InlineData(.1f, false)]
    public void CapturedRigRemainsIndependentAndSettles(float angle, bool impulse)
    {
        var (names, parents, reference) = RagdollSkeletonTests.Rig();
        var pose = reference.Select(p => p * Matrix4x4.CreateRotationY(angle)).ToArray();
        var frozen = pose.ToArray();
        var other = new Ragdoll();
        var doll = new Ragdoll();
        var world = Matrix4x4.CreateTranslation(4, 0, -3);
        var velocity = new Vector3(1, 0, -1);
        var bridge = RagdollPose.Activate(names, parents, pose, reference, world, doll, velocity);
        RagdollPose.Activate(names, parents, pose, reference, world, other, Vector3.Zero);
        var otherPositions = other.Bodies.Select(b => b.Position).ToArray();
        Assert.True(Vector3.Distance(Vector3.Transform((pose[1].Translation + pose[2].Translation) / 2, world),
            doll.Bodies[0].Position) < .00001f);
        Assert.All(doll.Bodies, b => Assert.Equal(velocity, b.LinearVelocity));
        Assert.All(doll.Joints, j => Assert.InRange(doll.JointSeparation(j), 0, .00001f));
        if (impulse) doll.ApplyImpulse(new(1, new(0, 120, -180)));
        for (int i = 0; i < 3600; i++)
        {
            doll.Update(Ragdoll.FixedStep);
            Assert.All(doll.Bodies, b => Assert.True(b.Bottom >= -.00001f));
            Assert.All(doll.Joints, j => {
                var a = doll.JointAngles(j);
                Assert.InRange(a.X, j.MinimumAngles.X - .001f, j.MaximumAngles.X + .001f);
                Assert.InRange(a.Y, j.MinimumAngles.Y - .001f, j.MaximumAngles.Y + .001f);
                Assert.InRange(a.Z, j.MinimumAngles.Z - .001f, j.MaximumAngles.Z + .001f);
                Assert.True(doll.JointSeparation(j) < .04f, $"Joint {j.Child}: {doll.JointSeparation(j)}");
            });
        }
        Assert.Equal(RagdollState.Settled, doll.State);
        Assert.Equal(frozen, pose);
        Assert.Equal(otherPositions, other.Bodies.Select(b => b.Position));
        bridge.Evaluate(doll);
        doll.Deactivate();
        Assert.Equal(RagdollState.Inactive, doll.State);
    }
}
