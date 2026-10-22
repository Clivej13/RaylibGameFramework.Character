using System.Numerics;
using RaylibGameFramework.Character;
using Xunit;

public sealed class RagdollContactTests
{
    internal static Ragdoll Create(Vector3 position, Vector3 velocity, RagdollSettings? config = null)
    {
        var points = new Dictionary<string, Vector3> {
            ["Hips"] = new(0, 1, 0), ["Chest"] = new(0, 1.4f, 0),
            ["Neck"] = new(0, 1.7f, 0), ["Head"] = new(0, 1.8f, 0)
        };
        foreach (var (side, x) in new[] { ("L", .25f), ("R", -.25f) })
        {
            points["UpperArm." + side] = new(x, 1.6f, 0);
            points["LowerArm." + side] = new(x * 2, 1.45f, 0);
            points["Hand." + side] = new(x * 3, 1.3f, 0);
            points["UpperLeg." + side] = new(x * .5f, 1, 0);
            points["LowerLeg." + side] = new(x * .5f, .55f, .05f);
            points["Foot." + side] = new(x * .5f, .1f, 0);
        }
        var pose = points.ToDictionary(p => p.Key, p => Matrix4x4.CreateScale(1f) *
            Matrix4x4.CreateTranslation(p.Value * 1f + position));
        var doll = new Ragdoll(config);
        doll.Activate(pose, pose, velocity);
        return doll;
    }

    private static Vector3 Momentum(Ragdoll doll) =>
        doll.Bodies.Aggregate(Vector3.Zero, (p, b) => p + b.Mass * b.LinearVelocity);
    private static float Energy(Ragdoll doll) => doll.Bodies.Sum(b => .5f * b.Mass * b.LinearVelocity.LengthSquared());

    [Fact]
    public void TightCapsulesRejectEmptySpaceInsideTheFormerTorsoSpheres()
    {
        var a = Create(new(0, 3, -.65f), Vector3.Zero);
        var b = Create(new(0, 3, 0), Vector3.Zero);
        Assert.False(RagdollContact.Overlaps(a, b));
        Assert.Null(RagdollContact.Resolve(a, b));
        a = Create(new(0, 3, -.46f), Vector3.Zero);
        Assert.True(RagdollContact.Overlaps(a, b));
    }

    [Fact]
    public void HeadOnPushesCharacterBackwardAndBothCharactersReceiveOppositeMomentum()
    {
        var first = Create(new(0, 3, -.46f), new(0, 0, 6));
        var second = Create(new(0, 3, 0), new(0, 0, -2));
        Vector3 beforeD = Momentum(first), beforeC = Momentum(second);
        RagdollContact.Resolve(first, second);
        Vector3 push = Momentum(second) - beforeC;
        Assert.True(push.Z > 0);
        Assert.True(Math.Abs(push.X) + Math.Abs(push.Y) < push.Z * .05f);
        Assert.True((Momentum(first) - beforeD + push).Length() < .001f);
        Assert.True(Energy(first) + Energy(second) < 2000);
        Assert.All(second.Bodies.Skip(2), b => Assert.Equal(new Vector3(0, 0, -2), b.LinearVelocity));
    }

    [Theory]
    [InlineData(-1f)]
    [InlineData(1f)]
    public void SideAndAngledHitsFollowContactGeometry(float side)
    {
        // Same forward velocity for both sides: the normal must choose the lateral reaction.
        var first = Create(new(side * .28f, 3, -.36f), new(0, 0, 6));
        var second = Create(new(0, 3, 0), Vector3.Zero);
        RagdollContact.Resolve(first, second);
        Vector3 push = Momentum(second);
        Assert.True(push.X * side < -1);
        Assert.True(push.Z > 1);
        Assert.True(Math.Abs(push.X) > push.Z * .25f);
    }

    [Theory]
    [InlineData(-1f)]
    [InlineData(1f)]
    public void PureSideHitPushesSideways(float side)
    {
        var first = Create(new(side * .65f, 3, 0), new(-side * 6, 0, 0));
        var second = Create(new(0, 3, 0), Vector3.Zero);
        RagdollContact.Resolve(first, second);
        Assert.True(Momentum(second).X * side < -1);
        Assert.True(Math.Abs(Momentum(second).Z) < .001f);
    }

    [Fact]
    public void ClosingSpeedControlsImpulseAndSeparatingContactDoesNotPull()
    {
        float Push(float speed)
        {
            var first = Create(new(0, 3, -.46f), new(0, 0, speed));
            var second = Create(new(0, 3, 0), Vector3.Zero);
            RagdollContact.Resolve(first, second);
            return Momentum(second).Z;
        }
        Assert.True(Push(6) > Push(2) * 2);
        Assert.Equal(0, Push(-6));
        Assert.Equal(0, Push(0));
        Assert.InRange(Push(1000), 0, new RagdollSettings().ContactMaximumImpulse + .001f);
    }

    [Fact]
    public void ShoulderContactProducesMirroredAngularResponse()
    {
        Vector3 Spin(float side)
        {
            var first = Create(new(side * .28f, 3, -.36f), new(0, 0, 6));
            var second = Create(new(0, 3, 0), Vector3.Zero);
            RagdollContact.Resolve(first, second);
            Assert.True(second.Bodies[0].AngularVelocity.Length() > .01f);
            return second.Bodies[1].AngularVelocity;
        }
        Vector3 left = Spin(-1), right = Spin(1);
        Assert.True(Math.Abs(left.Y) > .01f);
        Assert.True(left.Y * right.Y < 0);
        Assert.All(new[] { left, right }, w => Assert.InRange(w.Length(), 0, new RagdollSettings().MaximumAngularSpeed + .001f));
    }

    [Fact]
    public void OverlapCorrectionPreservesJointsAndDoesNotGenerateVelocity()
    {
        var first = Create(new(0, 3, 0), Vector3.Zero);
        var second = Create(new(0, 3, 0), Vector3.Zero);
        RagdollContact.Resolve(first, second);
        AssertClear(first, second);
        foreach (var doll in new[] { first, second })
        {
            Assert.Equal(Vector3.Zero, Momentum(doll));
            Assert.All(doll.Joints, j => Assert.InRange(doll.JointSeparation(j), 0, .00001f));
        }
    }

    [Fact]
    public void MovingPairStaysSeparatedGroundedAndEventuallySettles()
    {
        var first = Create(new(0, 1, -.4f), new(0, 0, 10));
        var second = Create(new(0, 1, 0), new(0, 0, -3));
        float initialEnergy = Energy(first) + Energy(second);
        RagdollContact.Resolve(first, second);
        Assert.True(Energy(first) + Energy(second) <= initialEnergy + .01f);
        for (int i = 0; i < 3600; i++)
        {
            RagdollContact.Resolve(first, second);
            first.Update(Ragdoll.FixedStep); second.Update(Ragdoll.FixedStep);
            RagdollContact.Resolve(first, second);
            AssertClear(first, second);
            foreach (var doll in new[] { first, second })
                Assert.All(doll.Bodies, b => {
                    Assert.True(float.IsFinite(b.Position.LengthSquared()));
                    Assert.True(b.Bottom >= -.0001f);
                    Assert.InRange(b.LinearVelocity.Length(), 0, 25);
                    Assert.InRange(b.AngularVelocity.Length(), 0, new RagdollSettings().MaximumAngularSpeed + .001f);
                });
        }
        Assert.True(second.HasMeaningfulGroundContact);
        Assert.Equal(RagdollState.Settled, second.State);
        Assert.Equal(RagdollState.Settled, first.State);
    }

    private static Vector3 Centre(Ragdoll doll) =>
        doll.Bodies.Aggregate(Vector3.Zero, (sum, b) => sum + b.Mass * b.Position) / doll.Bodies.Sum(b => b.Mass);

    private static float TotalEnergy(Ragdoll doll) => Energy(doll) + doll.Bodies.Sum(b =>
        .5f * b.Mass * (b.HalfSegment.LengthSquared() / 3 + .4f * b.Radius * b.Radius) *
        b.AngularVelocity.LengthSquared());

    private static void AssertClear(Ragdoll a, Ragdoll b)
    {
        foreach (int i in new[] { 0, 1, 3, 4, 5, 6, 7, 8, 9, 10 })
        foreach (int j in new[] { 0, 1, 3, 4, 5, 6, 7, 8, 9, 10 })
            Assert.True(RagdollContact.SurfaceGap(a.Bodies[i], b.Bodies[j]) >=
                -new RagdollSettings().ContactPenetrationSlop - .002f, $"Bodies {i}/{j} interpenetrated");
    }
}
