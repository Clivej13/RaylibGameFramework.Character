# RaylibGameFramework.Character

A small .NET 9 library for reusable humanoid ragdoll physics and pose handoff.
Version 0.1.0 uses only System.Numerics and the .NET runtime; Raylib is not required.
All public types are in the RaylibGameFramework.Character namespace.

## Supported functionality

- Eleven articulated capsule bodies and ten mass-weighted joints, with configurable anatomical limits.
- Named-bone activation from absolute world-space matrices and a separate anatomical reference pose.
- Fixed 120 Hz simulation, gravity, damping, planar ground collision, settling and impulse wake-up.
- Linear and point impulses, bounded angular velocity, capsule-pair contact impulses and overlap correction.
- Optional joint-space active drive with contact yielding, support, blend-in and fade-out.
- Managed skeleton mapping that preserves captured body offsets and unmapped child-local transforms.
- Stationary parent-local recovery blending, including affine residuals, with Down/GetUp/Complete phases.
- Validation of missing bones, malformed transforms, non-finite inputs, settings and skeleton hierarchy.

RagdollSettings supplies immutable initialization settings for masses, radii, joint limits,
solver forces, settling, active drive and contacts. JointAngleLimits uses Vector3 degree
bounds. The humanoid topology is intentionally fixed in this release, not a general physics engine.

Required landmarks are Hips, Chest, Neck, Head, and UpperArm, LowerArm, Hand, UpperLeg,
LowerLeg and Foot with .L/.R suffixes. Bodies are ordered Hips, Chest, Head, left upper/lower
arm, right upper/lower arm, left upper/lower leg, right upper/lower leg. Hands, feet and neck
are endpoints, not separate bodies. RagdollSkeleton also maps an optional Root bone to the
pelvis delta. It accepts arbitrary acyclic bone ordering; recovery requires parent-first order.

## Deliberate non-goals

No RPG, combat, football, AI, scoring, levels, controller, animation player, asset loading,
rendering, debug drawing, native skinning or automatic recovery eligibility rules.
No self collision, terrain/mesh collision, arbitrary body topology, continuous collision
detection or full rigid-body inertia tensor. Head-to-character collision is excluded from
the inherited contact layout (the head still collides with the ground).

## Units and coordinates

Distances and radii are metres, masses kilograms, time seconds, impulses N·s, velocities m/s,
angular velocities radians/s. Joint settings are degrees; reported joint rotation vectors
are radians. +Y is up; the ground is a horizontal world-space Y plane. There is no required
forward direction.

Matrices follow System.Numerics row-vector conventions: local * parent, model * world.
Activation dictionaries contain absolute world transforms; RagdollPose.Activate accepts
absolute model-space snapshots plus world. Reference matrices must describe anatomical zero,
not the current animation pose. Positive nonsingular affine transforms are supported;
reflection and projective transforms are rejected. Capsule radii scale from the bone Y scale,
while masses remain those configured in kilograms. Nonuniform affine poses are preserved in
the skeleton mapping but approximated by rigid capsules in physics.

## Minimal activation and update

This self-contained example builds a simple upright reference pose. Real applications supply
their own animated pose and bind/reference pose under the same bone names.

```csharp
using System.Numerics;
using RaylibGameFramework.Character;

var points = new Dictionary<string, Vector3>
{
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
var reference = points.ToDictionary(p => p.Key,
    p => Matrix4x4.CreateTranslation(p.Value));
var pose = reference.ToDictionary(p => p.Key,
    p => p.Value * Matrix4x4.CreateTranslation(0, 2, 0));

var ragdoll = new Ragdoll(new RagdollSettings { Gravity = 9.81f });
ragdoll.Activate(pose, reference, Vector3.Zero, groundHeight: 0);
ragdoll.ApplyImpulse(new RagdollImpulse(1, new Vector3(25, 0, 0)));
ragdoll.Update(1f / 60f); // two internal fixed steps
Vector3 chestPosition = ragdoll.Bodies[1].Position;
```

Update accumulates elapsed seconds and always advances in 1/120 s increments with a stable
solver order and 24 constraint iterations. A call accepts at most 0.25 s of catch-up; excess
time is discarded. Zero elapsed time never projects the activation snapshot.
Equal fixed-step histories are deterministic on the tested runtime; cross-platform bitwise
determinism is not promised.

For interacting ragdolls, maintain a shared fixed-step accumulator in the host. For each tick,
resolve each pair in a stable order, update all dolls with Ragdoll.FixedStep, then resolve the
pairs again. Calling contacts only once per rendered frame can miss fast collisions.
RagdollContact.Resolve returns a RagdollHit with BodyA/BodyB, a world point and a normal from
A to B. FindContact queries the nearest approaching pair even if separated; it requires two
distinct active dolls. Resolve and Overlaps ignore inactive dolls. Contact material settings
come from the first doll; use shared settings and a stable pair order.

HasGroundContact latches any ground contact observed during simulation.
HasMeaningfulGroundContact latches a low torso impact or sustained torso contact and stops
active drive. IsOnGround() queries current capsule geometry. Activation/deactivation clear
both history flags. Settled bodies stop integrating until an impulse wakes them.
Small resting-contact impulses preserve the quiet timer; explicit ApplyImpulse always wakes.
LastContactImpulse is the latest pair impulse, not a frame total, and resets on activation.

For pose handoff, RagdollPose.Activate returns a RagdollSkeleton; call Evaluate after physics
and consume its read-only ModelPose. A bridge belongs to one activation and cannot be reused
after deactivation/reactivation. Pose snapshots never advance animation or write rendering data.

Recovery is explicitly initiated by the host, usually once State is Settled. Convert the final
skeleton pose into the desired stationary recovery model space, then construct RagdollRecovery
with parent indices, that snapshot, a pure pose sampler and positive down/blend/get-up durations.
The sampler receives a phase and seconds within that phase. Blend duration must fit inside
down duration. Update carries elapsed time across phase boundaries. The host chooses world
placement, facing, animation clips, playback rates and when to return control. There are no
character-role rules or automatic world alignment.

## Build, test and package

From the repository root with the .NET 9 SDK:

```text
dotnet build -c Release
dotnet test -c Release
dotnet pack -c Release
```

Character.sln includes the library in Character/ and the xUnit project in Tests/.
The package is emitted to Character/bin/Release/RaylibGameFramework.Character.0.1.0.nupkg.
Tests run without a window, native libraries or assets.

## Extraction provenance and exclusions

Source inspected read-only: RaylibTackleAlley/Game/Ragdoll.cs, Ragdoll.Active.cs,
RagdollContact.cs, RagdollPose.cs, RagdollRecovery.cs, RagdollSkeleton.cs,
RagdollDebugControls.cs, AffinePose.cs and PlayerPhysicalAttributes.cs.
Physics defaults and their validation were extracted from TackleAlleyConfig.Tuning.cs;
no game configuration files or runtime configuration dependency were copied.

The solver, active motors and ordinary capsule contacts were retained. Managed matrix mapping,
affine decomposition and parent-local recovery blending were adapted from their original
Raylib adapters. Game-specific confirmed-contact momentum, wrap control, down-region rules,
animation gait selection and automatic recovery placement were left behind.

PlayerPhysicalAttributes depends on player profiles, visual proportions, ratings, reach and
role-specific forces; it was not copied. Set generic masses and radii through RagdollSettings.
RagdollDebugControls depends on opponents, input and drawing and was entirely excluded.
Native pose application, mesh skinning, clip ownership and drawing remain adapter responsibilities.

Tests: reusable cases from RagdollTests, ActiveRagdollTests and RagdollContactTests were ported
with original assertions/tolerances. The moving-pair case now starts with ordinary collision
instead of the removed confirmed-contact path. Rig and skeleton tests use synthetic managed
poses with the source joint/offset tolerances. Native mesh, pixel, asset and gameplay assertions
are intentionally excluded, not claimed as verified. The requested RagdollPoseTests.cs and
RagdollRecoveryTests.cs were absent in the source repository; new managed tests cover these APIs.

Follow-up: validate an application's real rig through a separate rendering adapter; profile
larger crowds before adding broad-phase acceleration; add swept contact detection if required
for high speeds. This is a compact position-based approximation, not a replacement for a
general-purpose physics engine.
