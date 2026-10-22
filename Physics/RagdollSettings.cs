using System.Numerics;
namespace RaylibGameFramework.Character;

public sealed class RagdollSettings
{
    public float Gravity { get; init; } = 9.81f;
    public float LinearDamping { get; init; } = 0.35f;
    public float AngularDamping { get; init; } = 2.5f;
    public float GroundFriction { get; init; } = 8f;
    public float MaximumAngularSpeed { get; init; } = 12f;
    public float SettleLinearSpeed { get; init; } = 0.12f;
    public float SettleAngularSpeed { get; init; } = 0.25f;
    public float SettleDuration { get; init; } = 0.8f;
    public float GroundContactTolerance { get; init; } = .005f;
    public float GroundedTorsoRadiusMultiplier { get; init; } = 3.5f;
    public float GroundImpactMinimumSpeed { get; init; } = .5f;
    public float GroundContactHoldSeconds { get; init; } = .08f;
    public float DriveDuration { get; init; } = 1.25f;
    public float DriveBlendIn { get; init; } = .12f;
    public float DriveFadeOut { get; init; } = .5f;
    public float MotorStiffness { get; init; } = 100f;
    public float MotorDamping { get; init; } = 12f;
    public float MaximumMotorAcceleration { get; init; } = 90f;
    public float MotorPoseResponse { get; init; } = 35f;
    public float MaximumMotorCorrectionSpeed { get; init; } = 6f;
    public float FootSupportDistance { get; init; } = .12f;
    public float SupportStiffness { get; init; } = 32f;
    public float SupportDamping { get; init; } = 6f;
    public float MaximumSupportAcceleration { get; init; } = 14f;
    public float BalanceStiffness { get; init; } = 18f;
    public float MaximumBalanceAcceleration { get; init; } = 12f;
    public float HitLimbMotorStrength { get; init; } = .08f;
    public float HitTorsoMotorStrength { get; init; } = .15f;
    public float ContactYieldSpeed { get; init; } = .75f;
    public float PelvisRadius { get; init; } = .15f;
    public float PelvisMass { get; init; } = 18f;
    public float ChestRadius { get; init; } = .19f;
    public float ChestMass { get; init; } = 25f;
    public float HeadRadius { get; init; } = .13f;
    public float HeadMass { get; init; } = 6f;
    public float UpperArmRadius { get; init; } = .065f;
    public float UpperArmMass { get; init; } = 3f;
    public float LowerArmRadius { get; init; } = .055f;
    public float LowerArmMass { get; init; } = 2f;
    public float UpperLegRadius { get; init; } = .095f;
    public float UpperLegMass { get; init; } = 9f;
    public float LowerLegRadius { get; init; } = .07f;
    public float LowerLegMass { get; init; } = 5f;
    public float HeadSegmentLength { get; init; } = .12f;
    public float BalanceTiltLimitRadians { get; init; } = MathF.PI * .45f;
    public float MinimumSupportStrength { get; init; } = .5f;
    public float BalanceDamping { get; init; } = 3f;
    public float ContactRestitution { get; init; } = .05f;
    public float ContactFriction { get; init; } = .2f;
    public float ContactImpulseScale { get; init; } = 1f;
    public float ContactMaximumImpulse { get; init; } = 160f;
    public float ContactTorsoRadius { get; init; } = .24f;
    public float ContactPelvisRadius { get; init; } = .19f;
    public float ContactChestShare { get; init; } = .6f;
    public float ContactPenetrationSlop { get; init; } = .005f;
    public float ContactLimbRadiusScale { get; init; } = 1f;
    public JointAngleLimits TorsoLimits { get; init; } = new()
    { MinimumDegrees = new(-35, -30, -25), MaximumDegrees = new(35, 30, 25) };
    public JointAngleLimits NeckLimits { get; init; } = new()
    { MinimumDegrees = new(-45, -65, -35), MaximumDegrees = new(45, 65, 35) };
    public JointAngleLimits ElbowLimits { get; init; } = new()
    { MinimumDegrees = new(-145, -8, -8), MaximumDegrees = new(5, 8, 8) };
    public JointAngleLimits KneeLimits { get; init; } = new()
    { MinimumDegrees = new(-5, -6, -6), MaximumDegrees = new(145, 6, 6) };
    public JointAngleLimits HipLimits { get; init; } = new()
    { MinimumDegrees = new(-110, -35, -45), MaximumDegrees = new(35, 35, 45) };
    public JointAngleLimits ShoulderLimits { get; init; } = new()
    { MinimumDegrees = new(-120, -70, -95), MaximumDegrees = new(120, 70, 95) };

    internal void ValidateRagdollPhysics()
    {
        Nonnegative(Gravity, nameof(Gravity));
        Nonnegative(LinearDamping, nameof(LinearDamping));
        Nonnegative(AngularDamping, nameof(AngularDamping));
        Nonnegative(GroundFriction, nameof(GroundFriction));
        Positive(MaximumAngularSpeed, nameof(MaximumAngularSpeed));
        Nonnegative(SettleLinearSpeed, nameof(SettleLinearSpeed));
        Nonnegative(SettleAngularSpeed, nameof(SettleAngularSpeed));
        Positive(SettleDuration, nameof(SettleDuration));
        Nonnegative(GroundContactTolerance, nameof(GroundContactTolerance));
        Nonnegative(GroundedTorsoRadiusMultiplier, nameof(GroundedTorsoRadiusMultiplier));
        Nonnegative(GroundImpactMinimumSpeed, nameof(GroundImpactMinimumSpeed));
        Nonnegative(GroundContactHoldSeconds, nameof(GroundContactHoldSeconds));
        Nonnegative(DriveDuration, nameof(DriveDuration));
        Positive(DriveBlendIn, nameof(DriveBlendIn));
        Positive(DriveFadeOut, nameof(DriveFadeOut));
        Nonnegative(MotorStiffness, nameof(MotorStiffness));
        Nonnegative(MotorDamping, nameof(MotorDamping));
        Nonnegative(MaximumMotorAcceleration, nameof(MaximumMotorAcceleration));
        Nonnegative(MotorPoseResponse, nameof(MotorPoseResponse));
        Nonnegative(MaximumMotorCorrectionSpeed, nameof(MaximumMotorCorrectionSpeed));
        Nonnegative(FootSupportDistance, nameof(FootSupportDistance));
        Nonnegative(SupportStiffness, nameof(SupportStiffness));
        Nonnegative(SupportDamping, nameof(SupportDamping));
        Nonnegative(MaximumSupportAcceleration, nameof(MaximumSupportAcceleration));
        Nonnegative(BalanceStiffness, nameof(BalanceStiffness));
        Nonnegative(MaximumBalanceAcceleration, nameof(MaximumBalanceAcceleration));
        Unit(HitLimbMotorStrength, nameof(HitLimbMotorStrength));
        Unit(HitTorsoMotorStrength, nameof(HitTorsoMotorStrength));
        Nonnegative(ContactYieldSpeed, nameof(ContactYieldSpeed));
        Unit(ContactRestitution, nameof(ContactRestitution));
        Nonnegative(ContactFriction, nameof(ContactFriction));
        Nonnegative(ContactImpulseScale, nameof(ContactImpulseScale));
        Nonnegative(ContactMaximumImpulse, nameof(ContactMaximumImpulse));
        Positive(ContactTorsoRadius, nameof(ContactTorsoRadius));
        Positive(ContactPelvisRadius, nameof(ContactPelvisRadius));
        Unit(ContactChestShare, nameof(ContactChestShare));
        Nonnegative(ContactPenetrationSlop, nameof(ContactPenetrationSlop));
        Nonnegative(ContactLimbRadiusScale, nameof(ContactLimbRadiusScale));
        Positive(PelvisRadius, nameof(PelvisRadius));
        Positive(PelvisMass, nameof(PelvisMass));
        Positive(ChestRadius, nameof(ChestRadius));
        Positive(ChestMass, nameof(ChestMass));
        Positive(HeadRadius, nameof(HeadRadius));
        Positive(HeadMass, nameof(HeadMass));
        Positive(UpperArmRadius, nameof(UpperArmRadius));
        Positive(UpperArmMass, nameof(UpperArmMass));
        Positive(LowerArmRadius, nameof(LowerArmRadius));
        Positive(LowerArmMass, nameof(LowerArmMass));
        Positive(UpperLegRadius, nameof(UpperLegRadius));
        Positive(UpperLegMass, nameof(UpperLegMass));
        Positive(LowerLegRadius, nameof(LowerLegRadius));
        Positive(LowerLegMass, nameof(LowerLegMass));
        Positive(HeadSegmentLength, nameof(HeadSegmentLength));
        Positive(BalanceTiltLimitRadians, nameof(BalanceTiltLimitRadians));
        Unit(MinimumSupportStrength, nameof(MinimumSupportStrength));
        Nonnegative(BalanceDamping, nameof(BalanceDamping));
        if (TorsoLimits is null) throw new ArgumentException(nameof(TorsoLimits));
        TorsoLimits.Validate(nameof(TorsoLimits));
        if (NeckLimits is null) throw new ArgumentException(nameof(NeckLimits));
        NeckLimits.Validate(nameof(NeckLimits));
        if (ElbowLimits is null) throw new ArgumentException(nameof(ElbowLimits));
        ElbowLimits.Validate(nameof(ElbowLimits));
        if (KneeLimits is null) throw new ArgumentException(nameof(KneeLimits));
        KneeLimits.Validate(nameof(KneeLimits));
        if (HipLimits is null) throw new ArgumentException(nameof(HipLimits));
        HipLimits.Validate(nameof(HipLimits));
        if (ShoulderLimits is null) throw new ArgumentException(nameof(ShoulderLimits));
        ShoulderLimits.Validate(nameof(ShoulderLimits));
    }
    private static void Finite(float value, string name)
    {
        if (!float.IsFinite(value)) throw new ArgumentOutOfRangeException(name, "Must be finite.");
    }
    private static void Nonnegative(float value, string name)
    {
        Finite(value, name);
        if (value < 0) throw new ArgumentOutOfRangeException(name, "Must be nonnegative.");
    }
    private static void Positive(float value, string name)
    {
        Finite(value, name);
        if (value <= 0) throw new ArgumentOutOfRangeException(name, "Must be positive.");
    }
    private static void Unit(float value, string name)
    {
        Nonnegative(value, name);
        if (value > 1) throw new ArgumentOutOfRangeException(name, "Must be between zero and one.");
    }
}

public sealed class JointAngleLimits
{
    public Vector3 MinimumDegrees { get; init; }
    public Vector3 MaximumDegrees { get; init; }
    public Vector3 Minimum => MinimumDegrees;
    public Vector3 Maximum => MaximumDegrees;
    internal void Validate(string name)
    {
        for (int i = 0; i < 3; i++)
            if (!float.IsFinite(MinimumDegrees[i]) || !float.IsFinite(MaximumDegrees[i]) ||
                MinimumDegrees[i] < -180 || MaximumDegrees[i] > 180 || MinimumDegrees[i] > MaximumDegrees[i])
                throw new ArgumentException($"{name} must have ordered finite angles within -180 to 180 degrees.");
    }
}
