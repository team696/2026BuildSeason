package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.BaseCam.AprilTagResult;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

/**
 * Layer A — locks down {@link BaseCam#addVisionEstimate} call-back semantics:
 *
 * <ul>
 *   <li>If the camera has no estimate, neither callback fires and the return is false.
 *   <li>If the estimate is rejected by the filter callback, the measurement callback does NOT
 *       fire and the return is false.
 *   <li>If the estimate is accepted, the measurement callback fires exactly once with the
 *       estimate's pose, time, and the configured stddev, and the return is true.
 * </ul>
 *
 * <p>Uses a small in-test {@code FakeCam} fixture; no HAL, no mocks.
 */
class BaseCamTest {

    @Test
    void no_estimate_means_no_callbacks_and_false_return() {
        FakeCam cam = new FakeCam(Optional.empty());
        AtomicInteger added = new AtomicInteger();
        boolean ret = cam.addVisionEstimate((p, t, v) -> added.incrementAndGet(), r -> true);
        assertThat(ret).isFalse();
        assertThat(added.get()).isEqualTo(0);
    }

    @Test
    void rejected_estimate_does_not_call_addVisionMeasurement() {
        FakeCam cam = new FakeCam(Optional.of(makeResult(cam())));
        AtomicInteger added = new AtomicInteger();
        boolean ret = cam.addVisionEstimate((p, t, v) -> added.incrementAndGet(), r -> false);
        assertThat(ret).isFalse();
        assertThat(added.get()).isEqualTo(0);
    }

    @Test
    void accepted_estimate_calls_addVisionMeasurement_exactly_once_with_pose_and_stddev() {
        AtomicBoolean called = new AtomicBoolean();
        Pose2d expectedPose = new Pose2d(1, 2, new edu.wpi.first.math.geometry.Rotation2d(0));
        double expectedTime = 12.5;
        FakeCam cam = new FakeCam(Optional.of(new FakeCam().new ResultWrapper(
                expectedPose, expectedTime, 1.0, 1, 0.1).inner));
        cam.setStdDeviations(0.5, 0.5, 5.0);

        boolean ret = cam.addVisionEstimate(
                (p, t, v) -> {
                    called.set(true);
                    assertThat(p).isEqualTo(expectedPose);
                    assertThat(t).isEqualTo(expectedTime);
                    assertThat(v.get(0, 0)).isEqualTo(0.5);
                    assertThat(v.get(1, 0)).isEqualTo(0.5);
                    assertThat(v.get(2, 0)).isEqualTo(5.0);
                },
                r -> true);

        assertThat(ret).isTrue();
        assertThat(called.get()).isTrue();
    }

    @Test
    void filter_callback_throwing_does_not_propagate_outside() {
        FakeCam cam = new FakeCam(Optional.of(makeResult(cam())));
        AtomicInteger added = new AtomicInteger();
        // Filter throws — addVisionEstimate must catch and still call the measurement callback
        // (this matches the current BaseCam contract: PLog.fatalException then fall through to
        // accept and feed the measurement).
        boolean ret = cam.addVisionEstimate(
                (p, t, v) -> added.incrementAndGet(),
                r -> {
                    throw new RuntimeException("filter blew up");
                });
        assertThat(ret).isTrue();
        assertThat(added.get()).isEqualTo(1);
    }

    private static FakeCam cam() {
        return new FakeCam();
    }

    private static AprilTagResult makeResult(FakeCam outer) {
        return outer.new ResultWrapper(
                        new Pose2d(0, 0, new edu.wpi.first.math.geometry.Rotation2d(0)),
                        0.0,
                        2.0,
                        1,
                        0.1)
                .inner;
    }

    /** Concrete subclass that lets us hand it a precomputed Optional<AprilTagResult>. */
    private static final class FakeCam extends BaseCam {
        private Optional<AprilTagResult> next = Optional.empty();

        FakeCam() {}

        FakeCam(Optional<AprilTagResult> next) {
            this.next = next;
        }

        @Override
        public Optional<AprilTagResult> getEstimate() {
            return next;
        }

        /** Helper: AprilTagResult is an inner class so we can only instantiate it via an outer. */
        final class ResultWrapper {
            final AprilTagResult inner;

            ResultWrapper(Pose2d pose, double time, double distToTag, int tagCount, double ambiguity) {
                inner = new AprilTagResult(pose, time, distToTag, tagCount, ambiguity);
            }
        }
    }
}
