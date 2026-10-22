using System.Numerics;

namespace RaylibGameFramework.Character;

public sealed partial class Ragdoll
{
    private float[] _motorStrength = [];
    private bool _torsoHit;
    public int? LastHitBody { get; private set; }
    public float MotorStrength(int body) => IsActivelyDriven ? _motorStrength[body] : 0f;

    public void ReactToContact(int body, float closingSpeed)
    {
        if (body < 0 || body >= _bodies.Length) throw new ArgumentOutOfRangeException(nameof(body));
        if (!float.IsFinite(closingSpeed) || closingSpeed < 0) throw new ArgumentOutOfRangeException(nameof(closingSpeed));
        if (!IsActivelyDriven || closingSpeed < _config.ContactYieldSpeed) return;
        LastHitBody = body;
        if (body <= 1)
        {
            _torsoHit = true;
            _motorStrength[0] = _motorStrength[1] = _config.HitTorsoMotorStrength;
            return; // Legs keep their gait while the torso falls.
        }
        if (body == 2) return;
        // Yield the entire struck limb, leaving other limbs and the torso braced.
        int first = body % 2 == 0 ? body - 1 : body;
        _motorStrength[first] = _motorStrength[first + 1] = _config.HitLimbMotorStrength;
    }
    private Func<float, IReadOnlyList<Quaternion>>? _drivePose;
    private Quaternion[] _driveStart = [], _driveTargets = [];
    private Quaternion _driveRoot;
    private float _driveHeight, _driveElapsed;
    public bool IsActivelyDriven => _drivePose is not null;
    public float ActiveDriveWeight { get; private set; }

    /// <summary>Joint-space animation targets; no animated root translation or velocity overwrite.</summary>
    public void StartActiveDrive(Func<float, IReadOnlyList<Quaternion>> pose, int? hitBody = null)
    {
        ArgumentNullException.ThrowIfNull(pose);
        if (hitBody is { } index && (index < 0 || index >= _bodies.Length)) throw new ArgumentOutOfRangeException(nameof(hitBody));
        if (!IsActive || State == RagdollState.Settled || HasMeaningfulGroundContact) return;
        _driveStart = _joints.Select(j => Quaternion.Normalize(
            Quaternion.Inverse(_bodies[j.Parent].Orientation) * _bodies[j.Child].Orientation)).ToArray();
        _driveTargets = _driveStart.ToArray();
        _driveRoot = _bodies[0].Orientation;
        _driveHeight = _bodies[0].Position.Y - GroundHeight;
        _driveElapsed = 0; ActiveDriveWeight = 0; _drivePose = pose;
        _motorStrength = Enumerable.Repeat(1f, _bodies.Length).ToArray();
        _torsoHit = false; LastHitBody = null;
        if (hitBody is { } hit) ReactToContact(hit, _config.ContactYieldSpeed);
    }

    private void StopActiveDrive()
    {
        _drivePose = null; _driveStart = []; _driveTargets = []; _motorStrength = [];
        _torsoHit = false; LastHitBody = null;
        _driveElapsed = 0; ActiveDriveWeight = 0;
    }

    private void SolveActiveMotor(int index)
    {
        if (!IsActivelyDriven) return;
        var joint = _joints[index];
        var a = _bodies[joint.Parent]; var b = _bodies[joint.Child];
        Vector3 error = Log(a.Orientation * _driveTargets[index] * Quaternion.Inverse(b.Orientation));
        // Compliant angular motor inside the position solver, so attachment projection
        // cannot erase the animation's small velocity impulse on every iteration.
        Vector3 correction = ClampLength(error * _config.MotorPoseResponse * FixedStep / SolverIterations,
            _config.MaximumMotorCorrectionSpeed * FixedStep / SolverIterations) *
            ActiveDriveWeight * _motorStrength[joint.Child];
        float inverse = a.InverseInertia + b.InverseInertia;
        a.Orientation = Quaternion.Normalize(Exp(-correction * a.InverseInertia / inverse) * a.Orientation);
        b.Orientation = Quaternion.Normalize(Exp(correction * b.InverseInertia / inverse) * b.Orientation);
    }

    private void UpdateActiveDrive()
    {
        if (_drivePose is null) return;
        _driveElapsed += FixedStep;
        if (_driveElapsed >= _config.DriveDuration || HasMeaningfulGroundContact)
        {
            StopActiveDrive(); return;
        }
        float blendIn = Math.Clamp(_driveElapsed / _config.DriveBlendIn, 0, 1);
        float fade = Math.Clamp((_config.DriveDuration - _driveElapsed) / _config.DriveFadeOut, 0, 1);
        ActiveDriveWeight = blendIn * fade;
        var targets = _drivePose(_driveElapsed);
        if (targets.Count != _joints.Length) throw new ArgumentException("One drive target per ragdoll joint is required.");
        for (int i = 0; i < _joints.Length; i++)
        {
            var j = _joints[i]; var a = _bodies[j.Parent]; var b = _bodies[j.Child];
            if (!float.IsFinite(targets[i].LengthSquared()) || targets[i].LengthSquared() < 1e-8f)
                throw new ArgumentException("Invalid drive rotation.");
            Quaternion target = Quaternion.Slerp(_driveStart[i], Quaternion.Normalize(targets[i]), blendIn);
            Vector3 angles = Vector3.Clamp(Log(Quaternion.Inverse(j.ReferenceRotation) * target), j.MinimumAngles, j.MaximumAngles);
            _driveTargets[i] = Quaternion.Normalize(j.ReferenceRotation * Exp(angles));
            Quaternion desired = Quaternion.Normalize(a.Orientation * _driveTargets[i]);
            Vector3 acceleration = ClampLength(
                Log(desired * Quaternion.Inverse(b.Orientation)) * _config.MotorStiffness -
                (b.AngularVelocity - a.AngularVelocity) * _config.MotorDamping, _config.MaximumMotorAcceleration) * ActiveDriveWeight * _motorStrength[j.Child];
            // Equal/opposite internal torque: animation works against the collision, never teleports bones.
            Vector3 angularImpulse = acceleration * FixedStep / (a.InverseInertia + b.InverseInertia);
            a.AngularVelocity -= angularImpulse * a.InverseInertia;
            b.AngularVelocity += angularImpulse * b.InverseInertia;
        }

        // Feet can briefly support body weight and brace the pelvis. No support in mid-air,
        // no horizontal root motion, and no strength after a large roll/knockdown.
        var root = _bodies[0];
        float tilt = Log(root.Orientation * Quaternion.Inverse(_driveRoot)).Length();
        float balance = Math.Clamp(1 - tilt / _config.BalanceTiltLimitRadians, 0, 1);
        bool supported = (_motorStrength[8] > _config.MinimumSupportStrength && _bodies[8].Bottom <= GroundHeight + _config.FootSupportDistance) ||
            (_motorStrength[10] > _config.MinimumSupportStrength && _bodies[10].Bottom <= GroundHeight + _config.FootSupportDistance);
        if (!supported || _torsoHit) return;
        float weight = ActiveDriveWeight * balance;
        float support = Math.Clamp(_config.Gravity + _config.SupportStiffness * (_driveHeight - (root.Position.Y - GroundHeight)) -
            _config.SupportDamping * root.LinearVelocity.Y, 0, _config.MaximumSupportAcceleration) * weight;
        foreach (var body in _bodies) body.LinearVelocity += Vector3.UnitY * support * FixedStep;
        Vector3 balanceAcceleration = ClampLength(
            Log(_driveRoot * Quaternion.Inverse(root.Orientation)) * _config.BalanceStiffness -
            root.AngularVelocity * _config.BalanceDamping, _config.MaximumBalanceAcceleration);
        root.AngularVelocity += balanceAcceleration * weight * FixedStep;
    }
}
