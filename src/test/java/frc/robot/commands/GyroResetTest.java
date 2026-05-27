package frc.robot.commands;

import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystem.Swerve;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

/**
 * Layer D — Mockito-driven unit test for {@link GyroReset}.
 *
 * <p>{@code Swerve} is constructed only by its no-arg singleton getter (which spins up real
 * Phoenix6 devices); we can't mock that cleanly. Instead we mock {@code Swerve} and
 * {@code Pigeon2} and verify {@link GyroReset#initialize()} calls {@code pigeon.setYaw(angle)}.
 *
 * <p>Also verifies {@link GyroReset#isFinished()} is {@code true} (one-shot).
 */
class GyroResetTest {

    @Test
    void initialize_calls_setYaw_with_default_zero_angle() {
        Swerve swerve = mock(Swerve.class, Mockito.RETURNS_DEEP_STUBS);
        Pigeon2 pigeon = mock(Pigeon2.class);
        when(swerve.getPigeon2()).thenReturn(pigeon);

        GyroReset cmd = new GyroReset(swerve);
        cmd.initialize();

        verify(pigeon).setYaw(eq(0.0));
    }

    @Test
    void initialize_calls_setYaw_with_specified_angle() {
        Swerve swerve = mock(Swerve.class);
        Pigeon2 pigeon = mock(Pigeon2.class);
        when(swerve.getPigeon2()).thenReturn(pigeon);

        GyroReset cmd = new GyroReset(swerve, 90.0);
        cmd.initialize();

        verify(pigeon).setYaw(eq(90.0));
    }

    @Test
    void isFinished_is_true_immediately() {
        Swerve swerve = mock(Swerve.class);
        GyroReset cmd = new GyroReset(swerve);
        // No HAL init needed for isFinished — it's a constant return.
        // (Construction adds requirements; that just registers with the CommandScheduler in
        // production but doesn't need anything special here.)
        // Skip directly to the assertion via the contract.
        // We intentionally do NOT call initialize here.
        // isFinished() should be true regardless of state.
        org.assertj.core.api.Assertions.assertThat(cmd.isFinished()).isTrue();
    }
}
