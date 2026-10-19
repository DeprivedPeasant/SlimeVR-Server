# Experimental contact-aware hip stabilization

This fork prototypes a small vertical correction for the virtual hip tracker sent
to VRChat. It is intended for a chest + hip + two thighs + two ankles + two feet
setup. It has not yet been validated against real motion capture or in a headset.

## Compare it in VRChat

1. Close the normal SlimeVR application before starting the experimental build.
   Only one server should run at a time. Keep your existing SteamVR driver installed.
   This portable build uses the same tracker settings as the normal app. Before
   first use, copy `%APPDATA%\dev.slimevr.SlimeVR\vrconfig.yml` somewhere safe.
   Extract the entire ZIP, then run `SlimeVR Contact Hips.exe` from that folder.
   The package includes its own Java runtime; no installation is required.
2. Start the experimental application and perform your normal standing full reset
   and mounting calibration. A full reset establishes the floor used by detection.
3. Open **Settings > General > Tracking settings** and enable
   **Contact-aware hip stabilization (experimental)**. The setting is saved and
   defaults to **off** for existing and new configurations.
4. Lie on your back **on the floor** and stay still for about half a second.
   Slowly look around while keeping your pelvis still. Compare with the toggle off.
5. Repeat on hands and knees, then deliberately raise, lower, and rock your hips.
   Check that intentional motion is retained and transitions release naturally.
6. Compare the SlimeVR skeleton and VRChat avatar. If the skeleton improves while
   the avatar still bounces, note your avatar and VRChat tracking lock settings.

Use the same avatar, calibration, and tracking settings for each comparison.
Turning the toggle off restores the original output on the next tracking frame;
there is no residual smoothing from this feature. Other existing SlimeVR
corrections remain governed by their own settings.

## What this prototype does

- Detects a horizontal, face-up torso and low pelvis/chest as likely supine contact.
  It holds the acquired hip height rather than guessing body thickness or snapping
  the pelvis to the floor.
- Detects a horizontal, face-down torso with both knees near the calibrated floor
  as likely kneeling support. Available tracked hand positions must also be low.
  It cancels common knee/hip height drift while preserving hip height changes
  relative to the knees.
- Requires a settled candidate for 0.4 seconds, with wider thresholds for retaining
  an established contact. Blends the correction in and out over an 80 ms time constant.
- Caps the vertical hip correction at 4 cm. Feet, knees, chest, and tracker rotations
  are not modified by this feature. The game's IK reconciles the changed hip target.
- Reads original bone positions every frame, before any feedback from its own output.
  The correction runs after existing leg corrections, so their hip push is included.
- Clears contact on resets, body-proportion changes, tracker reassignment, missing
  required tracking, long frame gaps, and tracking-origin discontinuities. Substantial
  translation or rotation releases contact; disabling the toggle clears it immediately.
- Requires a positional headset and active chest, hip, thigh, and ankle trackers.
  It bypasses self-localization and an already positional hip tracker.

This is a heuristic: contact is inferred, not measured. Beds, sofas, raised platforms,
side-lying poses, and arbitrary support surfaces are outside this version's scope.
Slow, small deliberate movement can resemble reconstruction error, particularly
when lying down. The correction limit bounds that ambiguity. Thresholds need tuning
from actual use; automated synthetic tests do not establish real-world accuracy.

## Useful feedback

For each pose, record toggle off/on, whether the pelvis was actually moving, whether
SlimeVR's skeleton also bounced, whether stabilization engaged, and any sticking or
snapping during transitions. Server logs report acquired/released contact states.
Try looking left/right, nodding, lifting your pelvis, rolling over, standing up,
and briefly disconnecting a tracker. Also verify ordinary standing and walking.

## Build from source

Use Java 17 or newer (with a Java 17 toolchain available), Node, and the repository's
pinned pnpm 10.33.0. Clone this fork with submodules; the SolarXR submodule points
to a companion fork containing an appended optional settings field.

```powershell
git submodule update --init --recursive
npx --yes pnpm@10.33.0 install --frozen-lockfile
.\gradlew.bat :server:spotlessCheck :server:core:test :server:desktop:shadowJar
npx --yes pnpm@10.33.0 --dir gui run build
```

Build the bindings provider as described in the normal build workflow, using the
output folder `bindings-provider/build/win-x64`. Generate the portable Java runtime
with a JDK's `jlink` before packaging (the crypto and ZIP providers supplement the
modules found by `jdeps`):

```powershell
jlink --add-modules java.base,java.desktop,java.management,java.sql,jdk.unsupported,jdk.crypto.ec,jdk.zipfs,jdk.accessibility --strip-debug --no-header-files --no-man-pages --output build/contact-portable-runtime
npx --yes pnpm@10.33.0 --dir gui exec electron-builder --config electron-builder.contact.yml --win --x64
```

The portable ZIP is written under `gui/dist/artifacts/win/`. The experimental
package is unsigned. Close it and reopen your normal app to return to the installed
version; restore your settings backup if needed.

The implementation and tests in this experimental fork were generated with OpenAI
Codex. Upstream's contribution policy does not accept AI-generated contributions;
this experiment is maintained in the fork and is not submitted upstream.
