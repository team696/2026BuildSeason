package frc.robot.testutil;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;

/**
 * Base class for tests that need the WPILib HAL initialized (DriverStation, Joystick, Pigeon2,
 * any subsystem construction). Layer-A pure-logic tests must <em>not</em> extend this — HAL init
 * adds ~1 second of JVM startup we don't want in the inner loop.
 */
public abstract class WpilibTestBase {
    @BeforeAll
    static void initHal() {
        // 500ms timeout, mode 0 (none). Idempotent within a JVM.
        HAL.initialize(500, 0);
    }

    @AfterAll
    static void shutdownHal() {
        HAL.shutdown();
    }
}
