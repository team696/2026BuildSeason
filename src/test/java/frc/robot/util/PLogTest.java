package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer A — tests {@link PLog}'s severity/category/message format by capturing System.out.
 *
 * <p>Catches regressions if someone changes the bracket format or drops one of the three fields.
 */
class PLogTest {
    private PrintStream originalOut;
    private ByteArrayOutputStream captured;

    @BeforeEach
    void captureStdout() {
        originalOut = System.out;
        captured = new ByteArrayOutputStream();
        System.setOut(new PrintStream(captured));
    }

    @AfterEach
    void restoreStdout() {
        System.setOut(originalOut);
    }

    @Test
    void debug_writes_severity_category_message_in_canonical_format() {
        PLog.debug("Cat", "msg");
        assertThat(captured.toString()).contains("[Debug] [Cat] msg");
    }

    @Test
    void info_writes_severity_category_message() {
        PLog.info("Cat", "hello");
        assertThat(captured.toString()).contains("[Info] [Cat] hello");
    }

    @Test
    void unusual_writes_severity_category_message() {
        PLog.unusual("Cat", "huh");
        assertThat(captured.toString()).contains("[Unusual] [Cat] huh");
    }

    @Test
    void fatalException_includes_exception_class_simple_name_and_stack_trace() {
        Exception e = new IllegalStateException("boom");
        PLog.fatalException("Cat", "user said something", e);
        String out = captured.toString();
        assertThat(out).contains("[Fatal]");
        assertThat(out).contains("[Cat]");
        assertThat(out).contains("user said something");
        assertThat(out).contains("IllegalStateException");
        assertThat(out).contains("boom");
        // At least one stacktrace element with 4-space indent.
        assertThat(out).contains("    ");
    }

    @Test
    void log_format_is_stable_across_severities() {
        PLog.debug("X", "a");
        PLog.info("Y", "b");
        PLog.unusual("Z", "c");
        String out = captured.toString();
        assertThat(out)
                .containsSubsequence(
                        "[Debug] [X] a", "[Info] [Y] b", "[Unusual] [Z] c");
    }
}
