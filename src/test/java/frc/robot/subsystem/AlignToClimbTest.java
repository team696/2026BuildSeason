package frc.robot.subsystem;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * Layer B — static contract test for {@link Swerve#alignToClimb()}.
 *
 * <p>The original bug: {@code Swerve} had a {@code Pose2d targetPose = Field.Alliance_Find
 * .climb_tower} cached at class-construction time. Since {@code Swerve} is a singleton built
 * before the FMS reports the alliance, this captured the default (blue) value. {@code
 * alignToClimb} then passed that stale value to {@code AutoBuilder.pathfindToPose}, so on the red
 * alliance the robot would drive to the blue climb tower.
 *
 * <p>The fix is to wrap {@code alignToClimb} in {@code Commands.defer} so the lookup happens at
 * schedule-time (after alliance is known), and to delete the cached field. This test pins both:
 *
 * <ol>
 *   <li>No field of type {@code Pose2d} is declared on {@code Swerve} that names "target" /
 *       "climb" (would imply someone re-added the cache).
 *   <li>{@code Swerve.java} source contains {@code Commands.defer(}, indicating the deferred
 *       binding is in place.
 * </ol>
 */
class AlignToClimbTest {
    @Test
    void swerve_does_not_cache_climb_target_pose_at_construction() {
        for (Field f : Swerve.class.getDeclaredFields()) {
            int mods = f.getModifiers();
            if (Modifier.isStatic(mods)) continue;
            String type = f.getType().getSimpleName();
            if (!"Pose2d".equals(type)) continue;
            String name = f.getName().toLowerCase();
            assertThat(name)
                    .as(
                            "Swerve.%s is a cached Pose2d — alignToClimb()'s climb_tower lookup"
                                    + " must be deferred to schedule-time, not stored in a field"
                                    + " at construction (alliance isn't known yet then). See"
                                    + " AlignToClimbTest.",
                            f.getName())
                    .doesNotContain("target")
                    .doesNotContain("climb")
                    .doesNotContain("tower");
        }
    }

    @Test
    void alignToClimb_uses_Commands_defer_for_lazy_alliance_lookup() throws IOException {
        String src = Files.readString(Path.of("src/main/java/frc/robot/subsystem/Swerve.java"));
        // Strip comments to reduce false positives.
        String stripped = src.replaceAll("//[^\\n]*", "");
        // alignToClimb should call Commands.defer (which evaluates its supplier at schedule
        // time) instead of returning a pre-built AutoBuilder.pathfindToPose(...) command.
        Pattern alignToClimbBody =
                Pattern.compile(
                        "public\\s+Command\\s+alignToClimb\\s*\\(\\s*\\)\\s*\\{([^}]+)\\}",
                        Pattern.DOTALL);
        var m = alignToClimbBody.matcher(stripped);
        assertThat(m.find()).as("expected to find alignToClimb() in Swerve.java").isTrue();
        String body = m.group(1);
        assertThat(body)
                .as(
                        "alignToClimb() body should call Commands.defer(...) so the alliance"
                                + " lookup happens at schedule-time, not at Swerve construction"
                                + " time. See AlignToClimbTest.")
                .contains("Commands.defer(");
    }
}
