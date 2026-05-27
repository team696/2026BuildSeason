package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;

/**
 * Layer B — static config validation.
 *
 * <p>For every {@code .auto} file under {@code src/main/deploy/pathplanner/autos}, parses the JSON
 * and collects every {@code {"type":"named","data":{"name":"X"}}} reference. Asserts every
 * referenced name appears in {@link NamedCommandRegistry#NAMES} — the single source of truth for
 * which named commands {@link Robot#Robot()} registers with PathPlanner.
 *
 * <p>Catches the case where an auto JSON references a named command that nobody registered (or
 * a typo'd name like {@code "Shooter"} instead of {@code "Shoot"}). Currently latent in this
 * codebase: nothing fails at robot-init for unregistered named commands; they just silently no-op
 * when the auto runs.
 */
class NamedCommandConsistencyTest {
    private static final Path AUTOS_DIR = Path.of("src/main/deploy/pathplanner/autos");
    private static final Path PATHS_DIR = Path.of("src/main/deploy/pathplanner/paths");

    @Test
    void every_named_command_referenced_in_an_auto_is_registered() throws IOException {
        Set<String> referenced = new TreeSet<>();
        try (var stream = Files.list(AUTOS_DIR)) {
            for (Path p : stream.filter(x -> x.toString().endsWith(".auto")).toList()) {
                JsonNode root = new ObjectMapper().readTree(p.toFile());
                collectNamedCommandRefs(root, referenced);
            }
        }
        assertThat(referenced)
                .as("expected to find at least one named command reference across all autos")
                .isNotEmpty();

        Set<String> missing = new TreeSet<>(referenced);
        missing.removeAll(NamedCommandRegistry.NAMES);
        assertThat(missing)
                .as(
                        "named commands referenced in .auto JSON but never registered via"
                                + " NamedCommandRegistry / Robot.Robot()")
                .isEmpty();
    }

    @Test
    void every_path_referenced_in_an_auto_exists_on_disk() throws IOException {
        Set<String> referencedPaths = new TreeSet<>();
        try (var stream = Files.list(AUTOS_DIR)) {
            for (Path p : stream.filter(x -> x.toString().endsWith(".auto")).toList()) {
                JsonNode root = new ObjectMapper().readTree(p.toFile());
                collectPathRefs(root, referencedPaths);
            }
        }

        Set<String> pathsOnDisk = new HashSet<>();
        try (var stream = Files.list(PATHS_DIR)) {
            stream.filter(p -> p.toString().endsWith(".path"))
                    .forEach(p -> {
                        String name = p.getFileName().toString();
                        pathsOnDisk.add(name.substring(0, name.length() - ".path".length()));
                    });
        }

        assertThat(referencedPaths)
                .as("path names referenced in autos but missing .path file on disk")
                .isSubsetOf(pathsOnDisk);
    }

    private static void collectNamedCommandRefs(JsonNode node, Set<String> sink) {
        if (node == null || node.isNull()) return;
        if (node.isObject()) {
            JsonNode type = node.get("type");
            JsonNode data = node.get("data");
            if (type != null
                    && "named".equals(type.asText())
                    && data != null
                    && data.has("name")) {
                sink.add(data.get("name").asText());
            }
            node.fields().forEachRemaining(e -> collectNamedCommandRefs(e.getValue(), sink));
        } else if (node.isArray()) {
            node.forEach(child -> collectNamedCommandRefs(child, sink));
        }
    }

    private static void collectPathRefs(JsonNode node, Set<String> sink) {
        if (node == null || node.isNull()) return;
        if (node.isObject()) {
            JsonNode type = node.get("type");
            JsonNode data = node.get("data");
            if (type != null
                    && "path".equals(type.asText())
                    && data != null
                    && data.has("pathName")) {
                sink.add(data.get("pathName").asText());
            }
            node.fields().forEachRemaining(e -> collectPathRefs(e.getValue(), sink));
        } else if (node.isArray()) {
            node.forEach(child -> collectPathRefs(child, sink));
        }
    }
}
