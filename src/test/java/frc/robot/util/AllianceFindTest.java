package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.testutil.WpilibTestBase;
import frc.robot.util.Field.Alliance_Find;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Layer C — drives {@link Alliance_Find#setAlliance()} via {@link DriverStationSim}, asserting
 * the four hub/pass/climb references switch in lock-step with the simulated alliance.
 */
class AllianceFindTest extends WpilibTestBase {

    @BeforeEach
    void resetAllianceState() {
        // Reset the static field state between tests so prev_alliance != alliance triggers the
        // update branch on each setAlliance() call.
        Alliance_Find.alliance = null;
        Alliance_Find.prev_alliance = null;
        DriverStationSim.setDsAttached(true);
    }

    @AfterEach
    void clearDsAlliance() {
        DriverStationSim.setAllianceStationId(
                edu.wpi.first.hal.AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
    }

    @Test
    void unknown_alliance_defaults_to_blue_references() {
        DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Unknown);
        DriverStationSim.notifyNewData();
        Alliance_Find.setAlliance();
        assertThat(Alliance_Find.alliance).isEqualTo(DriverStation.Alliance.Blue);
        // Initial state of class fields is already blue, so they should remain blue references.
        assertThat(Alliance_Find.hub).isEqualTo(Field.hub_position_blue);
        assertThat(Alliance_Find.climb_tower).isEqualTo(Field.climb_tower_blue);
    }

    @Test
    void red_alliance_switches_all_four_references_to_red() {
        DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        Alliance_Find.setAlliance();
        assertThat(Alliance_Find.alliance).isEqualTo(DriverStation.Alliance.Red);
        assertThat(Alliance_Find.hub).isEqualTo(Field.hub_position_red);
        assertThat(Alliance_Find.Pass_1).isEqualTo(Field.pass_position_red_1);
        assertThat(Alliance_Find.Pass_2).isEqualTo(Field.pass_position_red_2);
        assertThat(Alliance_Find.climb_tower).isEqualTo(Field.climb_tower_red);
    }

    @Test
    void switching_back_from_red_to_blue_resets_all_four_references() {
        DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        Alliance_Find.setAlliance();
        assertThat(Alliance_Find.alliance).isEqualTo(DriverStation.Alliance.Red);

        DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        Alliance_Find.setAlliance();
        assertThat(Alliance_Find.alliance).isEqualTo(DriverStation.Alliance.Blue);
        assertThat(Alliance_Find.hub).isEqualTo(Field.hub_position_blue);
        assertThat(Alliance_Find.Pass_1).isEqualTo(Field.pass_position_blue_1);
        assertThat(Alliance_Find.Pass_2).isEqualTo(Field.pass_position_blue_2);
        assertThat(Alliance_Find.climb_tower).isEqualTo(Field.climb_tower_blue);
    }
}
