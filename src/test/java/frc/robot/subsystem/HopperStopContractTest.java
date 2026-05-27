package frc.robot.subsystem;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * Layer A + Layer B — locks down the {@link Hopper#stop()} contract and its callers.
 *
 * <ul>
 *   <li>API: {@code Hopper.stop()} must exist as a {@code public void} method (not a
 *       {@code Command}-returning one), so synchronous callers actually stop the motor.
 *   <li>Discipline: source files outside {@code Binds.java} must not call
 *       {@code Hopper.get().Stop()} (the capital-S, Command-returning version). That call only
 *       belongs at the chooser layer where its return value is wired into
 *       {@code setDefaultCommand(...)}. Any other caller that discards the returned Command
 *       fails to actually stop the motor — the historical bug.
 * </ul>
 *
 * <p>Pure source-text + reflection inspection. No HAL. Sub-millisecond.
 */
class HopperStopContractTest {
    @Test
    void hopper_has_a_synchronous_void_stop_method() throws NoSuchMethodException {
        Method m = Hopper.class.getDeclaredMethod("stop");
        assertThat(m.getReturnType()).isEqualTo(void.class);
    }

    @Test
    void no_call_site_outside_default_command_binding_uses_Command_returning_Stop()
            throws IOException {
        Pattern badPattern = Pattern.compile("Hopper\\.get\\(\\)\\.Stop\\(\\)");
        for (Path p : sourceFilesToCheck()) {
            String src = Files.readString(p);
            // Strip line comments to avoid false positives in commented-out code.
            String stripped = src.replaceAll("//[^\\n]*", "");
            Matcher matcher = badPattern.matcher(stripped);
            assertThat(matcher.find())
                    .as(
                            "%s must not call Hopper.get().Stop() (Command-returning; result is"
                                    + " discarded). Use Hopper.get().stop() (void) instead.",
                            p)
                    .isFalse();
        }
    }

    private static java.util.List<Path> sourceFilesToCheck() throws IOException {
        // Every .java file in the main source tree except Binds.java (which legitimately uses
        // Hopper.get().Stop() as a default-command binding) and Hopper.java itself (which
        // defines the method).
        Path root = Path.of("src/main/java");
        try (var s = Files.walk(root)) {
            return s.filter(p -> p.toString().endsWith(".java"))
                    .filter(p -> !p.endsWith("Binds.java"))
                    .filter(p -> !p.endsWith("Hopper.java"))
                    .sorted()
                    .toList();
        }
    }
}
