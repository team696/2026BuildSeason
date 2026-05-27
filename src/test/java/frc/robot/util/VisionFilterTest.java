package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.junit.jupiter.api.Test;

/**
 * Layer A — pure-logic tests of the AprilTag vision filter that lives inside
 * {@code Swerve.acceptEstimate}. Asserts every threshold individually and locks down the standard-
 * deviation calculation.
 *
 * <p>No HAL, no mocks. Sub-millisecond per test.
 */
class VisionFilterTest {

    @Test
    void rejects_when_dist_to_tag_above_3_5_m() {
        var d = VisionFilter.evaluate(/*dist*/ 3.6, /*amb*/ 0.1, /*omega*/ 0.0);
        assertThat(d.accept()).isFalse();
    }

    @Test
    void rejects_when_ambiguity_above_0_7() {
        var d = VisionFilter.evaluate(2.0, 0.71, 0.0);
        assertThat(d.accept()).isFalse();
    }

    @Test
    void rejects_when_rotating_too_fast() {
        var d = VisionFilter.evaluate(2.0, 0.1, 1.6);
        assertThat(d.accept()).isFalse();
        // Symmetric: a fast negative rotation is also rejected.
        var d2 = VisionFilter.evaluate(2.0, 0.1, -1.6);
        assertThat(d2.accept()).isFalse();
    }

    @Test
    void accepts_close_target_within_thresholds() {
        var d = VisionFilter.evaluate(2.0, 0.2, 0.5);
        assertThat(d.accept()).isTrue();
    }

    @Test
    void uses_fixed_stddev_when_dist_under_half_meter() {
        var d = VisionFilter.evaluate(0.4, 0.3, 0.0);
        assertThat(d.accept()).isTrue();
        assertVec(d.stdDevs(), 0.3, 0.3, 50.0);
    }

    @Test
    void scales_stddev_quadratically_with_distance_outside_half_meter() {
        double dist = 2.0;
        double amb = 0.5;
        double expected = amb * dist * dist * 3.0; // == 0.5 * 4 * 3 = 6.0
        var d = VisionFilter.evaluate(dist, amb, 0.0);
        assertThat(d.accept()).isTrue();
        assertVec(d.stdDevs(), expected, expected, expected);
    }

    @Test
    void boundary_dist_3_5_exact_is_accepted() {
        // The original code uses strict inequality (> 3.5 rejects), so exactly 3.5 is in.
        var d = VisionFilter.evaluate(3.5, 0.1, 0.0);
        assertThat(d.accept()).isTrue();
    }

    @Test
    void boundary_ambiguity_0_7_exact_is_accepted() {
        var d = VisionFilter.evaluate(2.0, 0.7, 0.0);
        assertThat(d.accept()).isTrue();
    }

    @Test
    void boundary_omega_1_5_exact_is_accepted() {
        var d = VisionFilter.evaluate(2.0, 0.1, 1.5);
        assertThat(d.accept()).isTrue();
    }

    @Test
    void boundary_dist_0_5_exact_uses_quadratic_branch() {
        // Code path is `dist < 0.5 -> fixed`, so 0.5 falls through to quadratic.
        double dist = 0.5;
        double amb = 0.4;
        double expected = amb * dist * dist * 3.0;
        var d = VisionFilter.evaluate(dist, amb, 0.0);
        assertThat(d.accept()).isTrue();
        assertVec(d.stdDevs(), expected, expected, expected);
    }

    private static void assertVec(Matrix<N3, N1> v, double x, double y, double r) {
        assertThat(v).isNotNull();
        assertThat(v.get(0, 0)).isCloseTo(x, within(1e-9));
        assertThat(v.get(1, 0)).isCloseTo(y, within(1e-9));
        assertThat(v.get(2, 0)).isCloseTo(r, within(1e-9));
    }

    private static org.assertj.core.data.Offset<Double> within(double e) {
        return org.assertj.core.data.Offset.offset(e);
    }

    @SuppressWarnings("unused")
    private static Matrix<N3, N1> vec(double x, double y, double r) {
        return VecBuilder.fill(x, y, r);
    }
}
