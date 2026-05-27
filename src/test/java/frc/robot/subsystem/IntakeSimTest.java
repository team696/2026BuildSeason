package frc.robot.subsystem;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.testutil.WpilibTestBase;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

/**
 * Layer C — smoke test that {@link Intake#simulationPeriodic()} runs end-to-end without
 * throwing, and that the cached pivot position is updated to a finite value after a few
 * sim ticks.
 *
 * <p>We do not assert specific physics values — Phoenix6 status signals don't propagate sim
 * state cleanly in headless JUnit (see ShooterRollerVelocityTest commit notes). What this
 * test does protect against:
 *
 * <ul>
 *   <li>NullPointerExceptions or unchecked exceptions thrown from simulationPeriodic — e.g. an
 *       uninitialized FlywheelSim or arm sim.
 *   <li>The pivot/roller cache populating to NaN or infinity (indicating a divide-by-zero or
 *       broken physics setup).
 * </ul>
 */
class IntakeSimTest extends WpilibTestBase {

    @Test
    void simulationPeriodic_runs_for_several_ticks_without_throwing() throws Exception {
        Intake intake = Intake.get();
        for (int i = 0; i < 20; i++) {
            intake.simulationPeriodic();
            intake.periodic();
            SimHooks.stepTiming(0.020);
        }
        double pivotPos = privateDouble(intake, "cachedIntakePivotPosition");
        double rollerVel = privateDouble(intake, "cachedRollerVelocity");
        assertThat(pivotPos).as("cached pivot position should be a finite number").isFinite();
        assertThat(rollerVel).as("cached roller velocity should be a finite number").isFinite();
    }

    private static double privateDouble(Object obj, String name) throws Exception {
        Field f = obj.getClass().getDeclaredField(name);
        f.setAccessible(true);
        return f.getDouble(obj);
    }
}
