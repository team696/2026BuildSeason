package frc.robot.subsystem;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

/**
 * Layer A — pure-logic test for {@link Shooter#aggregateRollerVelocity(double, double)}.
 *
 * <p>The aggregation must report the leader motor's signed velocity unmodified. The follower
 * runs in {@code MotorAlignmentValue.Opposed} mode, so its signed rotor velocity is the negation
 * of the leader's; averaging cancels to ~0 at speed.
 *
 * <p>(Originally written as a Phoenix6 sim integration test, but Phoenix6 status-signal updates
 * don't propagate reliably in the headless JUnit sim environment. The behavior is small enough
 * that a pure-logic test gives equivalent confidence at sub-millisecond cost.)
 */
class ShooterRollerVelocityTest {

    @Test
    void at_speed_with_opposed_follower_reports_leader_velocity_not_zero() {
        // Leader spinning -25 rps; opposed follower's signed rotor velocity is +25 rps.
        // Buggy averaging: (-25 + 25)/2 = 0. Correct: -25.
        double agg = Shooter.aggregateRollerVelocity(-25.0, +25.0);
        assertThat(agg).isEqualTo(-25.0);
    }

    @Test
    void at_rest_returns_zero() {
        assertThat(Shooter.aggregateRollerVelocity(0.0, 0.0)).isEqualTo(0.0);
    }

    @Test
    void positive_leader_with_opposed_follower_reports_positive_leader() {
        // If the shooter were ever commanded forward, leader = +25, follower = -25.
        assertThat(Shooter.aggregateRollerVelocity(+25.0, -25.0)).isEqualTo(+25.0);
    }

    @Test
    void ignores_follower_value_entirely() {
        // Even if the follower reports a nonsense / desynced value, we trust the leader.
        assertThat(Shooter.aggregateRollerVelocity(-25.0, 0.0)).isEqualTo(-25.0);
        assertThat(Shooter.aggregateRollerVelocity(-25.0, +999.0)).isEqualTo(-25.0);
        assertThat(Shooter.aggregateRollerVelocity(-25.0, -999.0)).isEqualTo(-25.0);
    }

    @Test
    void leader_velocity_is_signed() {
        // Negative target velocities are the normal case; verify sign is preserved.
        assertThat(Shooter.aggregateRollerVelocity(-50.0, +50.0)).isNegative();
        assertThat(Shooter.aggregateRollerVelocity(+50.0, -50.0)).isPositive();
    }
}
