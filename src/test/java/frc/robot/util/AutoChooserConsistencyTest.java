package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

/**
 * Layer B — static config validation.
 *
 * <p>Parses the source of {@link frc.robot.util.Auto} and asserts two things:
 *
 * <ol>
 *   <li>Every {@code new PathPlannerAuto("X", ...)} literal corresponds to an actual {@code X.auto}
 *       file in the {@code src/main/deploy/pathplanner/autos/} directory.
 *   <li>Every {@code autoChooser.addOption("Label", new PathPlannerAuto("Name", ...))} has matching
 *       label and name — i.e. the chooser entry is honest about which auto it actually loads. This
 *       catches the historical bug where three entries labeled "...Bravo..." silently loaded
 *       "...Alpha..." auto files.
 * </ol>
 *
 * <p>Source parsing (instead of reflection-on-chooser) keeps this Layer B — no HAL, no
 * PathPlanner GUI config required.
 */
class AutoChooserConsistencyTest {
    private static final Path AUTO_JAVA = Path.of("src/main/java/frc/robot/util/Auto.java");
    private static final Path AUTOS_DIR = Path.of("src/main/deploy/pathplanner/autos");
    private static final Pattern PATH_PLANNER_AUTO_CTOR =
            Pattern.compile("new\\s+PathPlannerAuto\\s*\\(\\s*\"([^\"]+)\"");
    // Matches: addOption("LABEL", new PathPlannerAuto("NAME", ...))
    private static final Pattern ADD_OPTION_WITH_AUTO =
            Pattern.compile(
                    "addOption\\s*\\(\\s*\"([^\"]+)\"\\s*,\\s*"
                            + "new\\s+PathPlannerAuto\\s*\\(\\s*\"([^\"]+)\"");

    @Test
    void every_explicit_PathPlannerAuto_name_has_a_matching_auto_file() throws IOException {
        String source = Files.readString(AUTO_JAVA);
        Set<String> referencedAutoNames = new HashSet<>();
        Matcher m = PATH_PLANNER_AUTO_CTOR.matcher(source);
        while (m.find()) {
            referencedAutoNames.add(m.group(1));
        }

        Set<String> autosOnDisk = new HashSet<>();
        try (var stream = Files.list(AUTOS_DIR)) {
            stream.filter(p -> p.toString().endsWith(".auto"))
                    .forEach(p -> {
                        String name = p.getFileName().toString();
                        autosOnDisk.add(name.substring(0, name.length() - ".auto".length()));
                    });
        }

        assertThat(autosOnDisk).as("should find some .auto files on disk").isNotEmpty();
        assertThat(referencedAutoNames)
                .as("every new PathPlannerAuto(\"X\", ...) must have X.auto on disk")
                .isSubsetOf(autosOnDisk);
    }

    @Test
    void addOption_label_matches_loaded_auto_name() throws IOException {
        String source = Files.readString(AUTO_JAVA);
        Matcher m = ADD_OPTION_WITH_AUTO.matcher(source);
        int found = 0;
        while (m.find()) {
            found++;
            String label = m.group(1);
            String autoName = m.group(2);
            assertThat(label)
                    .as("addOption label must match the underlying PathPlannerAuto name")
                    .isEqualTo(autoName);
        }
        // We don't require addOption calls to exist — buildAutoChooser auto-loads everything —
        // but if any exist, they must be honest. The "found" counter is informational only.
        assertThat(found).as("scanned addOption(...) calls (informational)").isGreaterThanOrEqualTo(0);
    }
}
