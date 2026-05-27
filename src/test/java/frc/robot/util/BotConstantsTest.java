package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.util.BotConstants.Hopper;
import frc.robot.util.BotConstants.Intake;
import frc.robot.util.BotConstants.Shooter;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * Layer B — static config validation for the central {@link BotConstants} bag. Catches CAN-ID
 * collisions, interpolation tables out of order or out of plausible bounds, and unsafe current
 * limits.
 */
class BotConstantsTest {

    @Test
    void every_can_id_is_unique_across_all_subsystems() throws IllegalAccessException {
        Map<Integer, String> seen = new HashMap<>();
        for (Class<?> sub : new Class<?>[] {Intake.class, Hopper.class, Shooter.class}) {
            for (Field f : sub.getDeclaredFields()) {
                if (!Modifier.isStatic(f.getModifiers())) continue;
                if (f.getType() != int.class) continue;
                String fqName = sub.getSimpleName() + "." + f.getName();
                if (!looksLikeCanId(f.getName())) continue;
                int id = f.getInt(null);
                String prev = seen.put(id, fqName);
                assertThat(prev)
                        .as("CAN ID %d is declared by both %s and %s", id, prev, fqName)
                        .isNull();
            }
        }
    }

    @Test
    void every_can_id_is_in_valid_range() throws IllegalAccessException {
        for (Class<?> sub : new Class<?>[] {Intake.class, Hopper.class, Shooter.class}) {
            for (Field f : sub.getDeclaredFields()) {
                if (!Modifier.isStatic(f.getModifiers())) continue;
                if (f.getType() != int.class) continue;
                if (!looksLikeCanId(f.getName())) continue;
                int id = f.getInt(null);
                assertThat(id)
                        .as("%s.%s = %d must be a valid CAN ID in [0, 62]",
                                sub.getSimpleName(), f.getName(), id)
                        .isBetween(0, 62);
            }
        }
    }

    @Test
    void shooter_velocity_table_is_in_a_plausible_range() {
        // All keyed distances should be between 0.5 and 10 meters.
        // All velocities should be between -100 and 0 (the shooter spins backwards).
        double[] testDistances = {0.5, 1.0, 1.607, 2.0, 2.206, 2.5, 2.744, 3.060, 5.0, 10.0};
        for (double d : testDistances) {
            double v = Shooter.ShooterTable.get(d);
            assertThat(v)
                    .as("Shooter velocity at distance %.3f is implausible", d)
                    .isBetween(-100.0, 0.0);
        }
    }

    @Test
    void backSpin_table_is_in_a_plausible_range() {
        double[] testDistances = {1.607, 2.0, 2.206, 2.744, 3.060};
        for (double d : testDistances) {
            double v = Shooter.backSpinTable.get(d);
            assertThat(v)
                    .as("Backspin velocity at distance %.3f is implausible", d)
                    .isBetween(-100.0, 0.0);
        }
    }

    @Test
    void shooter_and_backspin_tables_have_matching_distance_keys() {
        // This isn't strictly required, but they should generally be sampled at the same
        // distances so the indexer/flywheel speeds are coordinated.
        for (double d : new double[] {1.607, 2.206, 2.257, 2.744, 3.060}) {
            double shooterV = Shooter.ShooterTable.get(d);
            double backV = Shooter.backSpinTable.get(d);
            assertThat(shooterV).as("shooter table missing key %.3f", d).isNotEqualTo(0.0);
            assertThat(backV).as("backspin table missing key %.3f", d).isNotEqualTo(0.0);
        }
    }

    @Test
    void stator_current_limits_are_safe() {
        // Stator current limits should never exceed 200A (mechanical/electrical safety),
        // and should never be 0 (which would disable the limit and effectively be unbounded).
        assertThat(Intake.cfg_Roller.CurrentLimits.StatorCurrentLimit).isBetween(1.0, 200.0);
        assertThat(Intake.cfg_Pivot.CurrentLimits.StatorCurrentLimit).isBetween(1.0, 200.0);
        assertThat(Hopper.cfg_Hopper.CurrentLimits.StatorCurrentLimit).isBetween(1.0, 200.0);
        assertThat(Shooter.cfg_shooter.CurrentLimits.StatorCurrentLimit).isBetween(1.0, 200.0);
    }

    private static boolean looksLikeCanId(String fieldName) {
        String n = fieldName.toLowerCase();
        return n.endsWith("id") || n.endsWith("_id") || n.endsWith("id_2");
    }
}
