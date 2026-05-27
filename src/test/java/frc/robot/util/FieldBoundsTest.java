package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

/**
 * Layer B — static config validation. No HAL needed.
 *
 * <p>Walks every public static Translation2d / Pose2d declared on {@link frc.robot.util.Field}
 * and asserts the coordinates are on the field. Catches typos like {@code 5201} (should have been
 * {@code 5.201}).
 */
class FieldBoundsTest {
    private static final double FIELD_LENGTH_M =
            frc.robot.util.Field.LENGTH.in(Meters);
    private static final double FIELD_WIDTH_M =
            frc.robot.util.Field.WIDTH.in(Meters);

    @TestFactory
    List<DynamicTest> every_published_pose_is_inside_the_field() throws IllegalAccessException {
        List<DynamicTest> tests = new ArrayList<>();
        for (Field f : frc.robot.util.Field.class.getDeclaredFields()) {
            int mods = f.getModifiers();
            if (!Modifier.isStatic(mods) || !Modifier.isPublic(mods)) continue;
            Object value = f.get(null);
            if (value instanceof Translation2d t) {
                tests.add(DynamicTest.dynamicTest(
                        f.getName() + " inside field", () -> assertOnField(f.getName(), t)));
            } else if (value instanceof Pose2d p) {
                tests.add(DynamicTest.dynamicTest(
                        f.getName() + " inside field",
                        () -> assertOnField(f.getName(), p.getTranslation())));
            }
        }
        // Make sure we actually found some — guards against the test silently passing if Field
        // were refactored to private fields.
        assertThat(tests).as("no public Translation2d/Pose2d fields found on Field").isNotEmpty();
        return tests;
    }

    private static void assertOnField(String name, Translation2d t) {
        assertThat(t.getX())
                .as(name + ".x must be within field length [0, %.3f]", FIELD_LENGTH_M)
                .isBetween(0.0, FIELD_LENGTH_M);
        assertThat(t.getY())
                .as(name + ".y must be within field width [0, %.3f]", FIELD_WIDTH_M)
                .isBetween(0.0, FIELD_WIDTH_M);
    }
}
