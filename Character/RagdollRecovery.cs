using System.Numerics;

namespace RaylibGameFramework.Character;

public enum RecoveryPhase { Down, GetUp, Complete }

/// <summary>Caller-controlled stationary recovery. Samples absolute model-space poses without owning animation or rendering.</summary>
public sealed class RagdollRecovery
{
    private readonly int[] _parents;
    private readonly Matrix4x4[] _startLocal, _modelPose;
    private readonly Func<RecoveryPhase, float, IReadOnlyList<Matrix4x4>> _sample;
    private readonly float _downDuration, _blendDuration, _getUpDuration;
    private float _elapsed;
    public RecoveryPhase Phase { get; private set; } = RecoveryPhase.Down;
    public float PhaseTime => _elapsed;
    public IReadOnlyList<Matrix4x4> ModelPose { get; }

    /// <param name="settledPose">Snapshot already expressed in the caller's chosen recovery model space.</param>
    /// <param name="sample">Pure sampler taking phase and seconds within that phase. Must return a complete absolute pose.</param>
    public RagdollRecovery(IReadOnlyList<int> parents, IReadOnlyList<Matrix4x4> settledPose,
        Func<RecoveryPhase, float, IReadOnlyList<Matrix4x4>> sample,
        float downDuration, float blendDuration, float getUpDuration)
    {
        ArgumentNullException.ThrowIfNull(sample);
        if (!float.IsFinite(downDuration) || !float.IsFinite(blendDuration) || !float.IsFinite(getUpDuration) ||
            downDuration <= 0 || blendDuration <= 0 || blendDuration > downDuration || getUpDuration <= 0)
            throw new ArgumentOutOfRangeException(nameof(downDuration), "Durations must be finite, positive, and blend must fit within down.");
        if (parents.Count == 0 || parents.Count != settledPose.Count)
            throw new ArgumentException("Complete parent-first skeleton required.");
        _parents = parents.ToArray();
        for (int i = 0; i < _parents.Length; i++)
            if (_parents[i] < -1 || _parents[i] >= i) throw new ArgumentException("Parents must precede children.");
        ValidatePose(settledPose);
        _startLocal = Enumerable.Range(0, parents.Count).Select(i => Local(settledPose, i)).ToArray();
        _modelPose = settledPose.ToArray();
        ModelPose = Array.AsReadOnly(_modelPose);
        _sample = sample; _downDuration = downDuration; _blendDuration = blendDuration; _getUpDuration = getUpDuration;
        Evaluate();
    }

    private void ValidatePose(IReadOnlyList<Matrix4x4> pose)
    {
        if (pose.Count != _parents.Length || pose.Any(p => !AffinePose.Decompose(p, out _, out _, out _)))
            throw new ArgumentException("Invalid recovery pose.");
    }

    private Matrix4x4 Local(IReadOnlyList<Matrix4x4> pose, int i)
    {
        if (_parents[i] < 0) return pose[i];
        if (!Matrix4x4.Invert(pose[_parents[i]], out var inverse)) throw new ArgumentException("Singular parent.");
        return pose[i] * inverse;
    }

    public void Update(float elapsed)
    {
        if (!float.IsFinite(elapsed) || elapsed < 0) throw new ArgumentOutOfRangeException(nameof(elapsed));
        if (Phase == RecoveryPhase.Complete) return;
        while (Phase != RecoveryPhase.Complete)
        {
            float duration = Phase == RecoveryPhase.Down ? _downDuration : _getUpDuration;
            float step = Math.Min(elapsed, Math.Max(0, duration - _elapsed));
            _elapsed += step; elapsed -= step;
            if (_elapsed + .000001f < duration) break;
            if (Phase == RecoveryPhase.Down) { Phase = RecoveryPhase.GetUp; _elapsed = 0; }
            else { Phase = RecoveryPhase.Complete; _elapsed = _getUpDuration; break; }
            if (elapsed <= 0) break;
        }
        Evaluate();
    }

    private void Evaluate()
    {
        var target = _sample(Phase == RecoveryPhase.Complete ? RecoveryPhase.GetUp : Phase, _elapsed);
        ValidatePose(target);
        float blend = Phase == RecoveryPhase.Down ? Math.Clamp(_elapsed / _blendDuration, 0, 1) : 1;
        blend = blend * blend * (3 - 2 * blend);
        for (int i = 0; i < target.Count; i++)
        {
            var start = _startLocal[i];
            var end = Local(target, i);
            if (blend == 0) { _modelPose[i] = start * (_parents[i] >= 0 ? _modelPose[_parents[i]] : Matrix4x4.Identity); continue; }
            if (blend == 1) { _modelPose[i] = target[i]; continue; }
            if (!AffinePose.Decompose(start, out var ss, out var sq, out var sp) ||
                !AffinePose.Decompose(end, out var es, out var eq, out var ep))
                throw new ArgumentException("Invalid local recovery pose.");
            var startTrs = Matrix4x4.CreateScale(ss) * Matrix4x4.CreateFromQuaternion(sq) * Matrix4x4.CreateTranslation(sp);
            var endTrs = Matrix4x4.CreateScale(es) * Matrix4x4.CreateFromQuaternion(eq) * Matrix4x4.CreateTranslation(ep);
            Matrix4x4.Invert(startTrs, out var inverseStart);
            Matrix4x4.Invert(endTrs, out var inverseEnd);
            var trs = Matrix4x4.CreateScale(Vector3.Lerp(ss, es, blend)) *
                Matrix4x4.CreateFromQuaternion(Quaternion.Slerp(sq, eq, blend)) *
                Matrix4x4.CreateTranslation(Vector3.Lerp(sp, ep, blend));
            _modelPose[i] = Matrix4x4.Lerp(start * inverseStart, end * inverseEnd, blend) * trs *
                (_parents[i] >= 0 ? _modelPose[_parents[i]] : Matrix4x4.Identity);
        }
    }
}
