package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Pure-logic decision for the shooter:
 *
 * <ul>
 *   <li>Look up flywheel target velocity from a distance→velocity table.
 *   <li>Look up indexer speed from a distance→backspin table.
 *   <li>Decide whether to fire the indexer this loop using a hysteresis latch:
 *       not-firing → fire iff {@code |measured - target| < enterTol};
 *       firing → stop iff {@code |measured - target| > exitTol}.
 *       (Bands chosen so a momentary dip outside the tight entry band doesn't drop the indexer.)
 * </ul>
 *
 * <p>Extracted from {@link frc.robot.commands.ShootCommand} so the latch logic is testable
 * exhaustively with no HAL or mocks. The default enter/exit tolerances are exposed as constants
 * so {@code ShootCommand} and the test stay in lock-step.
 */
public final class ShooterControlPolicy {
    private ShooterControlPolicy() {}

    /** Default enter band — tight, matches the original 0.95 from ShootCommand. */
    public static final double DEFAULT_ENTER_TOL = 0.95;

    /** Default exit band — wider than enter so a brief sag doesn't drop the indexer. */
    public static final double DEFAULT_EXIT_TOL = 1.5;

    public static Decision compute(
            double distMeters,
            double measuredVel,
            InterpolatingDoubleTreeMap velTable,
            InterpolatingDoubleTreeMap indexerTable,
            double enterTol,
            double exitTol,
            boolean wasFiring) {
        double target = velTable.get(distMeters);
        double indexer = indexerTable.get(distMeters);
        double err = Math.abs(measuredVel - target);
        boolean fire = wasFiring ? (err < exitTol) : (err < enterTol);
        return new Decision(target, indexer, fire);
    }

    /**
     * @param flywheelTarget commanded flywheel velocity (rotations/second, signed)
     * @param indexerSpeed commanded indexer velocity (only meaningful when {@code fireIndexer})
     * @param fireIndexer true → run the indexer + hopper; false → keep them stopped
     */
    public record Decision(double flywheelTarget, double indexerSpeed, boolean fireIndexer) {}
}
