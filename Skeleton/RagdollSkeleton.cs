using System.Numerics;

namespace RaylibGameFramework.Character;

/// <summary>Per-activation bridge between world-space bodies and absolute
/// model-space bone poses. Owns only managed snapshots, never rendering resources.</summary>
public sealed class RagdollSkeleton
{
    private readonly Ragdoll _owner;
    private readonly RagdollBody _activationBody;
    private readonly int[] _parents, _bodyIndices, _order;
    private readonly Matrix4x4[] _offsets, _modelPose;
    private readonly Matrix4x4 _inverseWorld;
    public Matrix4x4 ModelWorld { get; }
    public IReadOnlyList<int> BodyIndices { get; }
    public IReadOnlyList<Matrix4x4> ModelPose { get; }

    public RagdollSkeleton(IReadOnlyList<string> names, IReadOnlyList<int> parents,
        IReadOnlyList<Matrix4x4> animatedModelPose, Matrix4x4 modelWorld, Ragdoll ragdoll)
    {
        _owner = ragdoll;
        int count = names.Count;
        if (!ragdoll.IsActive || count == 0 || parents.Count != count || animatedModelPose.Count != count ||
            names.Distinct().Count() != count || !Matrix4x4.Invert(modelWorld, out _inverseWorld))
            throw new ArgumentException("A complete skeleton, invertible world transform and active ragdoll are required.");
        if (!AffinePose.Decompose(modelWorld, out _, out _, out _) ||
            animatedModelPose.Any(p => !AffinePose.Decompose(p, out _, out _, out _)))
            throw new ArgumentException("Invalid skeleton transform.");
        _activationBody = ragdoll.Bodies[0];
        ModelWorld = modelWorld;
        _parents = parents.ToArray();
        _bodyIndices = Enumerable.Repeat(-1, count).ToArray();
        _offsets = new Matrix4x4[count]; _modelPose = new Matrix4x4[count];
        var byName = names.Select((name, index) => (name, index)).ToDictionary(p => p.name, p => p.index);
        for (int b = 0; b < ragdoll.Bodies.Count; b++)
        {
            if (!byName.TryGetValue(ragdoll.Bodies[b].Bone, out int bone))
                throw new ArgumentException($"Missing mapped bone: {ragdoll.Bodies[b].Bone}");
            _bodyIndices[bone] = b;
        }
        // Root is an ancestor of Hips, so it cannot inherit from Hips through the hierarchy.
        // Give it the same rigid-body delta with its own captured offset.
        if (byName.TryGetValue("Root", out int root)) _bodyIndices[root] = 0;
        var order = new List<int>();
        var visited = new byte[count];
        void Visit(int i)
        {
            if (visited[i] == 2) return;
            if (visited[i] == 1) throw new ArgumentException("Cyclic skeleton.");
            visited[i] = 1;
            int parent = _parents[i];
            if (parent < -1 || parent >= count) throw new ArgumentException("Invalid skeleton parent.");
            if (parent >= 0) Visit(parent);
            visited[i] = 2; order.Add(i);
        }
        for (int i = 0; i < count; i++) Visit(i);
        _order = order.ToArray();
        for (int i = 0; i < count; i++)
        {
            if (_bodyIndices[i] >= 0)
            {
                Matrix4x4.Invert(BodyWorld(ragdoll.Bodies[_bodyIndices[i]]), out var inverseBody);
                _offsets[i] = animatedModelPose[i] * modelWorld * inverseBody;
            }
            else if (_parents[i] >= 0)
            {
                if (!Matrix4x4.Invert(animatedModelPose[_parents[i]], out var inverseParent))
                    throw new ArgumentException("Singular parent pose.");
                _offsets[i] = animatedModelPose[i] * inverseParent;
            }
            else _offsets[i] = animatedModelPose[i];
        }
        BodyIndices = Array.AsReadOnly(_bodyIndices);
        ModelPose = Array.AsReadOnly(_modelPose);
        Evaluate(ragdoll);
    }

    public static Matrix4x4 BodyWorld(RagdollBody body) =>
        Matrix4x4.CreateFromQuaternion(body.Orientation) * Matrix4x4.CreateTranslation(body.Position);

    public void Evaluate(Ragdoll ragdoll)
    {
        if (!ReferenceEquals(ragdoll, _owner) || !ragdoll.IsActive || !ReferenceEquals(_activationBody, ragdoll.Bodies[0]))
            throw new InvalidOperationException("Skeleton belongs to a different or inactive activation.");
        foreach (int i in _order)
        {
            int body = _bodyIndices[i];
            _modelPose[i] = body >= 0
                ? _offsets[i] * BodyWorld(ragdoll.Bodies[body]) * _inverseWorld
                : _parents[i] >= 0 ? _offsets[i] * _modelPose[_parents[i]] : _offsets[i];
            if (!AffinePose.Decompose(_modelPose[i], out _, out _, out _))
                throw new InvalidOperationException("Ragdoll produced an invalid bone pose.");
        }
    }
}
