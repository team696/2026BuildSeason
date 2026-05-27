package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.util.Auto;
import frc.robot.util.Field;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Single source of truth for every PathPlanner "named command" the robot exposes.
 *
 * <p>{@link Robot#Robot()} should build its {@code Auto.NamedCommand[]} from {@link #all()} (so
 * adding a new named command happens in exactly one place), and {@link NamedCommandConsistencyTest}
 * cross-checks {@link #NAMES} against every {@code .auto} JSON file in the deploy directory so an
 * auto referencing a non-existent named command fails at build time instead of silently no-op'ing
 * during a match.
 */
public final class NamedCommandRegistry {
    private NamedCommandRegistry() {}

    /** Every name that {@link #all()} returns, exposed as a stable set for cross-checking. */
    public static final Set<String> NAMES =
            Set.of(
                    "Shoot",
                    "ShootForever",
                    "Shorter Shoot",
                    "Intake_",
                    "Intake Forever",
                    "Do stow",
                    "Rev up",
                    "Oscilate",
                    "OscilateForever",
                    "Outtake");

    /**
     * Build the live {@link Auto.NamedCommand} array. Lazily constructs each command on call (the
     * caller is expected to invoke this exactly once at robot init).
     */
    public static Auto.NamedCommand[] all() {
        Supplier<edu.wpi.first.math.geometry.Translation2d> hub = () -> Field.Alliance_Find.hub;
        return new Auto.NamedCommand[] {
            named("Shoot", new ShootCommand(hub).withTimeout(5.5)),
            named("ShootForever", new ShootCommand(hub)),
            named("Shorter Shoot", new ShootCommand(hub).withTimeout(2.5)),
            named("Intake_", Intake.get().doIntake().withTimeout(2)),
            named("Intake Forever", Intake.get().doIntake()),
            named("Do stow", Intake.get().doStow().withTimeout(0.5)),
            named("Rev up", Shooter.get().spinUpCommand()),
            named("Oscilate", Intake.get().doOscilateIntake().withTimeout(4.0)),
            named("OscilateForever", Intake.get().doOscilateIntake()),
            named("Outtake", Intake.get().doOuttake().withTimeout(3.5))
        };
    }

    private static Auto.NamedCommand named(String name, Command command) {
        return new Auto.NamedCommand(name, command);
    }
}
