using System.Numerics;

namespace RaylibGameFramework.Character;

public readonly record struct RagdollHit(int BodyA, int BodyB, Vector3 Point, Vector3 Normal);

/// <summary>Bone-aligned torso and limb capsules, without self collision.</summary>
public static class RagdollContact
{
    // Iteration count is solver accuracy, not a gameplay tuning value.
    public const int SeparationIterations = 8;

    private static readonly int[] CollisionBodies = [0, 1, 3, 4, 5, 6, 7, 8, 9, 10];
    public static float Radius(RagdollBody body) => body.ContactRadius;

    public static RagdollHit FindContact(Ragdoll aDoll, Ragdoll bDoll, Vector3? relativeVelocity = null)
    {
        if (!aDoll.IsActive || !bDoll.IsActive || ReferenceEquals(aDoll, bDoll))
            throw new ArgumentException("Two distinct active ragdolls are required.");
        if (relativeVelocity is { } v && !float.IsFinite(v.LengthSquared()))
            throw new ArgumentException("Invalid relative velocity.");
        int ai = 0, bi = 0;
        float nearest = float.PositiveInfinity;
        bool foundClosing = false;
        Vector3 relative = relativeVelocity ?? (CentreVelocity(aDoll) - CentreVelocity(bDoll));
        foreach (int a in CollisionBodies)
        foreach (int b in CollisionBodies)
        {
            float gap = SurfaceGap(aDoll.Bodies[a], bDoll.Bodies[b]);
            bool closing = Vector3.Dot(relative, Geometry(aDoll.Bodies[a], bDoll.Bodies[b], relative).Normal) > 1e-5f;
            if ((!foundClosing || closing) && (closing && !foundClosing || gap < nearest))
            { nearest = gap; ai = a; bi = b; foundClosing = closing; }
        }
        var geometry = Geometry(aDoll.Bodies[ai], bDoll.Bodies[bi], relative);
        return new(ai, bi, geometry.Point, geometry.Normal);
    }

    public static bool Overlaps(Ragdoll a, Ragdoll b)
    {
        var _config = a.Config;
        if (!a.IsActive || !b.IsActive) return false;
        foreach (int ai in CollisionBodies)
        foreach (int bi in CollisionBodies)
            if (SurfaceGap(a.Bodies[ai], b.Bodies[bi]) <= _config.ContactPenetrationSlop) return true;
        return false;
    }

    public static float SurfaceGap(RagdollBody a, RagdollBody b)
    {
        ClosestPoints(a, b, out var pa, out var pb);
        return Vector3.Distance(pa, pb) - Radius(a) - Radius(b);
    }

    private static void ClosestPoints(RagdollBody a, RagdollBody b, out Vector3 pa, out Vector3 pb)
    {
        Vector3 p = a.SegmentStart;
        Vector3 q = b.SegmentStart;
        Vector3 u = a.SegmentEnd - p;
        Vector3 v = b.SegmentEnd - q;
        Vector3 r = p - q;
        float aa = Vector3.Dot(u, u), bb = Vector3.Dot(u, v), cc = Vector3.Dot(v, v);
        float d = Vector3.Dot(u, r), e = Vector3.Dot(v, r);
        float s = 0, t = 0;
        if (aa <= 1e-8f) t = cc > 1e-8f ? Math.Clamp(e / cc, 0, 1) : 0;
        else if (cc <= 1e-8f) s = Math.Clamp(-d / aa, 0, 1);
        else
        {
            float denominator = aa * cc - bb * bb;
            s = denominator > 1e-8f ? Math.Clamp((bb * e - cc * d) / denominator, 0, 1) : 0;
            t = (bb * s + e) / cc;
            if (t < 0) { t = 0; s = Math.Clamp(-d / aa, 0, 1); }
            else if (t > 1) { t = 1; s = Math.Clamp((bb - d) / aa, 0, 1); }
        }
        pa = p + u * s; pb = q + v * t;
    }

    private static (Vector3 Normal, Vector3 Point) Geometry(RagdollBody a, RagdollBody b, Vector3 closing)
    {
        ClosestPoints(a, b, out var pa, out var pb);
        Vector3 normal = Normal(pb - pa, closing);
        return (normal, (pa + normal * Radius(a) + pb - normal * Radius(b)) * .5f);
    }

    public static RagdollHit? Resolve(Ragdoll aDoll, Ragdoll bDoll)
    {
        var _config = aDoll.Config;
        if (!aDoll.IsActive || !bDoll.IsActive || ReferenceEquals(aDoll, bDoll)) return null;
        int nearestA = 0, nearestB = 0;
        float nearestGap = float.PositiveInfinity;
        bool foundClosing = false;
        Vector3 closingVelocity = CentreVelocity(aDoll) - CentreVelocity(bDoll);
        foreach (int a in CollisionBodies)
        foreach (int b in CollisionBodies)
        {
            float gap = SurfaceGap(aDoll.Bodies[a], bDoll.Bodies[b]);
            var geometry = Geometry(aDoll.Bodies[a], bDoll.Bodies[b], closingVelocity);
            bool closing = Vector3.Dot(closingVelocity, geometry.Normal) > 1e-5f;
            // Prefer the nearest approaching capsule.
            if ((!foundClosing || closing) && (closing && !foundClosing || gap < nearestGap))
            {
                nearestGap = gap; nearestA = a; nearestB = b; foundClosing = closing;
            }
            if (gap > _config.ContactPenetrationSlop) continue;
            Contact(aDoll, bDoll, a, b);
        }
        var hitGeometry = Geometry(aDoll.Bodies[nearestA], bDoll.Bodies[nearestB], closingVelocity);
        var hit = new RagdollHit(nearestA, nearestB, hitGeometry.Point, hitGeometry.Normal);

        // Split positional correction: never turn spawn overlap into kinetic energy.
        // Translate whole articulated poses so existing joints and angular limits are preserved.
        for (int iteration = 0; iteration < SeparationIterations; iteration++)
        foreach (int a in CollisionBodies)
        foreach (int b in CollisionBodies)
        {
            var da = aDoll.Bodies[a]; var cb = bDoll.Bodies[b];
            float depth = -SurfaceGap(da, cb) - _config.ContactPenetrationSlop;
            if (depth <= 0) continue;
            Vector3 normal = Geometry(da, cb, Velocity(aDoll, a, da.Position) - Velocity(bDoll, b, cb.Position)).Normal;
            float wa = 1 / Mass(aDoll), wb = 1 / Mass(bDoll);
            Translate(aDoll, -normal * depth * wa / (wa + wb));
            Translate(bDoll, normal * depth * wb / (wa + wb));
        }
        return nearestGap <= _config.ContactPenetrationSlop ? hit : null;
    }

    private static Vector3 CentreVelocity(Ragdoll doll) =>
        doll.Bodies.Aggregate(Vector3.Zero, (sum, b) => sum + b.Mass * b.LinearVelocity) / Mass(doll);

    private static Vector3 Normal(Vector3 delta, Vector3 closing) =>
        delta.LengthSquared() > 1e-8f ? Vector3.Normalize(delta) :
        closing.LengthSquared() > 1e-8f ? Vector3.Normalize(closing) : Vector3.UnitX;

    private static void Contact(Ragdoll a, Ragdoll b, int ai, int bi)
    {
        var _config = a.Config;
        var ab = a.Bodies[ai]; var bb = b.Bodies[bi];
        var (normal, point) = Geometry(ab, bb, Velocity(a, ai, ab.Position) - Velocity(b, bi, bb.Position));
        Vector3 relative = Velocity(b, bi, point) - Velocity(a, ai, point);
        float closing = Vector3.Dot(relative, normal);
        if (closing >= -1e-5f) return; // Never attract separating bodies or reapply a canned kick.
        float limit = _config.ContactMaximumImpulse;
        float impulse = Math.Min(limit,
            -(1 + _config.ContactRestitution) * closing * _config.ContactImpulseScale / (EffectiveInverseMass(a, ai, point, normal) + EffectiveInverseMass(b, bi, point, normal)));
        Vector3 total = normal * impulse;
        Vector3 tangent = relative - normal * closing;
        if (tangent.LengthSquared() > 1e-8f)
        {
            Vector3 direction = Vector3.Normalize(tangent);
            float friction = Math.Min(_config.ContactFriction * impulse, tangent.Length() /
                (EffectiveInverseMass(a, ai, point, direction) + EffectiveInverseMass(b, bi, point, direction)));
            total -= direction * friction;
        }
        if (total.Length() > limit) total = Vector3.Normalize(total) * limit;
        a.LastContactImpulse = b.LastContactImpulse = total.Length();
        Apply(a, ai, -total, point);
        Apply(b, bi, total, point);
        a.ReactToContact(ai, -closing);
        b.ReactToContact(bi, -closing);
    }

    private static float Share(int i, RagdollSettings config) => i == 1 ? config.ContactChestShare : 1 - config.ContactChestShare;
    private static Vector3 Velocity(Ragdoll doll, int hit, Vector3 point)
    {
        if (hit >= 3)
        {
            var b = doll.Bodies[hit];
            return b.LinearVelocity + Vector3.Cross(b.AngularVelocity, point - b.Position);
        }
        Vector3 velocity = Vector3.Zero;
        for (int i = 0; i < 2; i++)
        {
            var body = doll.Bodies[i];
            velocity += Share(i, doll.Config) * (body.LinearVelocity + Vector3.Cross(body.AngularVelocity, point - body.Position));
        }
        return velocity;
    }

    private static float EffectiveInverseMass(Ragdoll doll, int hit, Vector3 point, Vector3 direction)
    {
        if (hit >= 3)
        {
            var b = doll.Bodies[hit];
            return 1 / b.Mass + Vector3.Cross(point - b.Position, direction).LengthSquared() * b.InverseInertia;
        }
        float inverse = 0;
        for (int i = 0; i < 2; i++)
        {
            var body = doll.Bodies[i];
            float share = Share(i, doll.Config);
            inverse += share * share * (1 / body.Mass +
                Vector3.Cross(point - body.Position, direction).LengthSquared() * body.InverseInertia);
        }
        return inverse;
    }

    private static void Apply(Ragdoll doll, int hit, Vector3 impulse, Vector3 point)
    {
        if (hit >= 3) { doll.ApplyContactImpulse(new(hit, impulse, point)); return; }
        for (int i = 0; i < 2; i++) doll.ApplyContactImpulse(new(i, impulse * Share(i, doll.Config), point));
    }

    private static float Mass(Ragdoll doll) => doll.Bodies.Sum(b => b.Mass);
    private static void Translate(Ragdoll doll, Vector3 offset)
    {
        // Keep every capsule above its existing ground plane without changing the pose.
        float clearance = doll.Bodies.Min(b => b.Bottom - doll.GroundHeight);
        offset.Y = Math.Max(offset.Y, -Math.Max(0, clearance));
        foreach (var body in doll.Bodies) body.Position += offset;
    }
}
