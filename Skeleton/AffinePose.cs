using System.Numerics;

namespace RaylibGameFramework.Character;

// Nonuniform body proportions followed by bone rotation can create shear.
// Physics needs a rigid orientation; pose mapping keeps the original affine matrix.
internal static class AffinePose
{
    public static bool Decompose(Matrix4x4 m, out Vector3 scale, out Quaternion rotation, out Vector3 position)
    {
        scale = default; rotation = Quaternion.Identity; position = default;
        if (!float.IsFinite(m.M11) || !float.IsFinite(m.M12) || !float.IsFinite(m.M13) ||
            !float.IsFinite(m.M21) || !float.IsFinite(m.M22) || !float.IsFinite(m.M23) ||
            !float.IsFinite(m.M31) || !float.IsFinite(m.M32) || !float.IsFinite(m.M33) ||
            !float.IsFinite(m.M41) || !float.IsFinite(m.M42) || !float.IsFinite(m.M43) ||
            !float.IsFinite(m.M14) || !float.IsFinite(m.M24) || !float.IsFinite(m.M34) || !float.IsFinite(m.M44) ||
            Math.Abs(m.M14) > 1e-6f || Math.Abs(m.M24) > 1e-6f || Math.Abs(m.M34) > 1e-6f || Math.Abs(m.M44 - 1) > 1e-6f ||
            !float.IsFinite(m.GetDeterminant()) || m.GetDeterminant() <= 1e-12f) return false;
        if (Matrix4x4.Decompose(m, out scale, out rotation, out position)) return true;
        position = m.Translation;
        Vector3 x = new(m.M11, m.M12, m.M13), y = new(m.M21, m.M22, m.M23), z = new(m.M31, m.M32, m.M33);
        scale = new(x.Length(), y.Length(), z.Length());
        rotation = Quaternion.Identity;
        if (!float.IsFinite(scale.LengthSquared()) || Math.Min(scale.X, Math.Min(scale.Y, scale.Z)) < 1e-6f) return false;
        x = Vector3.Normalize(x);
        y -= x * Vector3.Dot(y, x);
        if (y.LengthSquared() < 1e-12f) return false;
        y = Vector3.Normalize(y);
        z = Vector3.Normalize(Vector3.Cross(x, y));
        rotation = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(new(
            x.X, x.Y, x.Z, 0, y.X, y.Y, y.Z, 0, z.X, z.Y, z.Z, 0, 0, 0, 0, 1)));
        return float.IsFinite(position.LengthSquared()) && m.GetDeterminant() > 0;
    }

}
