package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Pure-logic AprilTag vision-measurement filter, extracted from {@code Swerve.acceptEstimate} so
 * it can be exhaustively unit-tested with no HAL and no mocks.
 *
 * <p>The original implementation:
 * <pre>
 *   if (distToTag &gt; 3.5)       return false;
 *   if (ambiguity &gt; 0.7)       return false;
 *   if (omegaRadPerSec &gt; 1.5)  return false;       // bug: not abs()-wrapped
 *   if (distToTag &lt; 0.5) setStdDevs(0.3, 0.3, 50)
 *   else                 setStdDevs(amb * dist^2 * 3, ...)
 * </pre>
 *
 * <p>This extraction preserves the original distance and ambiguity bounds exactly, and uses the
 * absolute value of omega so a fast <em>negative</em> rotation is also rejected (the original
 * comparison was a sign-sensitive {@code &gt; 1.5}; this is the conservative fix).
 */
public final class VisionFilter {
    private VisionFilter() {}

    public static final double MAX_DIST_M = 3.5;
    public static final double MAX_AMBIGUITY = 0.7;
    public static final double MAX_ABS_OMEGA_RAD_PER_SEC = 1.5;
    public static final double CLOSE_RANGE_DIST_M = 0.5;
    private static final Matrix<N3, N1> CLOSE_RANGE_STDDEV = VecBuilder.fill(0.3, 0.3, 50.0);

    /**
     * @param distToTag meters from camera to nearest tag
     * @param ambiguity 0..1 ambiguity score from the pose estimator
     * @param omegaRadPerSec robot angular velocity (sign-insensitive — abs() is taken internally)
     */
    public static Decision evaluate(double distToTag, double ambiguity, double omegaRadPerSec) {
        if (distToTag > MAX_DIST_M) return Decision.reject();
        if (ambiguity > MAX_AMBIGUITY) return Decision.reject();
        if (Math.abs(omegaRadPerSec) > MAX_ABS_OMEGA_RAD_PER_SEC) return Decision.reject();

        Matrix<N3, N1> stdDevs;
        if (distToTag < CLOSE_RANGE_DIST_M) {
            stdDevs = CLOSE_RANGE_STDDEV;
        } else {
            double s = ambiguity * distToTag * distToTag * 3.0;
            stdDevs = VecBuilder.fill(s, s, s);
        }
        return new Decision(true, stdDevs);
    }

    /** A decision returned by {@link #evaluate}: whether to accept, and (if so) what stddev to use. */
    public record Decision(boolean accept, Matrix<N3, N1> stdDevs) {
        public static Decision reject() {
            return new Decision(false, null);
        }
    }
}
