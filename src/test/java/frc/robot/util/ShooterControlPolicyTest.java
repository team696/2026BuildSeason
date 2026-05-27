package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.junit.jupiter.api.Test;

/**
 * Layer A — pure-logic tests of the shooter "should we fire the indexer?" decision, extracted
 * from {@link frc.robot.commands.ShootCommand} so it can be exhaustively tested with no HAL and
 * no mocks.
 *
 * <p>The contract:
 * <ul>
 *   <li>{@code flywheelTarget} is interpolated from the velocity table by distance.</li>
 *   <li>{@code indexerSpeed} is interpolated from the backspin table.</li>
 *   <li>{@code fireIndexer} is a hysteresis latch: not firing → fire iff |err| &lt; enterTol;
 *       firing → stop iff |err| &gt; exitTol.</li>
 * </ul>
 */
class ShooterControlPolicyTest {
    // Tables roughly matching BotConstants.Shooter.* but small and self-contained.
    private static final InterpolatingDoubleTreeMap VEL = treeMap(1.5, -25.0, 3.0, -30.0);
    private static final InterpolatingDoubleTreeMap IDX = treeMap(1.5, -20.0, 3.0, -28.0);
    private static final double ENTER = 0.95;
    private static final double EXIT = 1.5;

    @Test
    void flywheel_target_is_interpolated_from_distance() {
        var d = ShooterControlPolicy.compute(2.25, 0.0, VEL, IDX, ENTER, EXIT, false);
        assertThat(d.flywheelTarget()).isCloseTo(-27.5, within(1e-9));
        assertThat(d.indexerSpeed()).isCloseTo(-24.0, within(1e-9));
    }

    @Test
    void does_not_fire_when_far_from_target_and_not_currently_firing() {
        var d = ShooterControlPolicy.compute(1.5, 0.0, VEL, IDX, ENTER, EXIT, false);
        // target is -25, measured is 0, |err| = 25 -> not within enter band
        assertThat(d.fireIndexer()).isFalse();
    }

    @Test
    void fires_once_within_enter_band() {
        // target is -25, measured -24.5 -> |err| = 0.5 < 0.95
        var d = ShooterControlPolicy.compute(1.5, -24.5, VEL, IDX, ENTER, EXIT, false);
        assertThat(d.fireIndexer()).isTrue();
    }

    @Test
    void keeps_firing_while_inside_exit_band() {
        // target -25, measured -23.7 -> |err| = 1.3, between enter (0.95) and exit (1.5)
        var d = ShooterControlPolicy.compute(1.5, -23.7, VEL, IDX, ENTER, EXIT, /*wasFiring*/ true);
        assertThat(d.fireIndexer()).isTrue();
    }

    @Test
    void stops_firing_when_drifting_past_exit_band() {
        // target -25, measured -22 -> |err| = 3 > 1.5
        var d = ShooterControlPolicy.compute(1.5, -22.0, VEL, IDX, ENTER, EXIT, /*wasFiring*/ true);
        assertThat(d.fireIndexer()).isFalse();
    }

    @Test
    void exit_band_is_wider_than_enter_band_so_short_dips_do_not_drop_indexer() {
        // Enter -> firing
        var d1 = ShooterControlPolicy.compute(1.5, -24.5, VEL, IDX, ENTER, EXIT, false);
        assertThat(d1.fireIndexer()).isTrue();

        // Small dip: |err|=1.2, outside enter (0.95) but inside exit (1.5) -> keep firing
        var d2 = ShooterControlPolicy.compute(1.5, -23.8, VEL, IDX, ENTER, EXIT, true);
        assertThat(d2.fireIndexer()).isTrue();
    }

    @Test
    void enter_just_inside_band_fires() {
        // target -25, measured -24.1 -> err = 0.9 (clearly inside enter band of 0.95)
        var d = ShooterControlPolicy.compute(1.5, -24.1, VEL, IDX, ENTER, EXIT, false);
        assertThat(d.fireIndexer()).isTrue();
    }

    @Test
    void enter_just_outside_band_does_not_fire() {
        // target -25, measured -24 -> err = 1.0 (just outside enter band)
        var d = ShooterControlPolicy.compute(1.5, -24.0, VEL, IDX, ENTER, EXIT, false);
        assertThat(d.fireIndexer()).isFalse();
    }

    @Test
    void exit_just_inside_band_keeps_firing() {
        // target -25, measured -23.6 -> err = 1.4 (clearly inside exit band of 1.5)
        var d = ShooterControlPolicy.compute(1.5, -23.6, VEL, IDX, ENTER, EXIT, true);
        assertThat(d.fireIndexer()).isTrue();
    }

    @Test
    void exit_just_outside_band_stops_firing() {
        // target -25, measured -23.4 -> err = 1.6 (just outside exit band)
        var d = ShooterControlPolicy.compute(1.5, -23.4, VEL, IDX, ENTER, EXIT, true);
        assertThat(d.fireIndexer()).isFalse();
    }

    @Test
    void distance_below_table_min_clamps_to_lowest_velocity() {
        var d = ShooterControlPolicy.compute(0.5, 0.0, VEL, IDX, ENTER, EXIT, false);
        assertThat(d.flywheelTarget()).isEqualTo(-25.0);
    }

    @Test
    void distance_above_table_max_clamps_to_highest_velocity() {
        var d = ShooterControlPolicy.compute(99.0, 0.0, VEL, IDX, ENTER, EXIT, false);
        assertThat(d.flywheelTarget()).isEqualTo(-30.0);
    }

    private static InterpolatingDoubleTreeMap treeMap(double k1, double v1, double k2, double v2) {
        var m = new InterpolatingDoubleTreeMap();
        m.put(k1, v1);
        m.put(k2, v2);
        return m;
    }

    private static org.assertj.core.data.Offset<Double> within(double e) {
        return org.assertj.core.data.Offset.offset(e);
    }
}
