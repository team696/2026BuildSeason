package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/*
 *   Put Values relating to field in this field,
 *   such as positions of field elements
 */
public class Field {
	public static final Field2d SIM = new Field2d();

	public static final Distance LENGTH = Feet.of(57.53);
	public static final Distance WIDTH = Feet.of(26.75);

	public final static Translation2d hub_position_blue = new Translation2d(4.616, 4.048);
	public final static Translation2d pass_position_blue_1 = new Translation2d(1.060,7.369);
	public final static Translation2d pass_position_blue_2 = new Translation2d(0.825,0.740);
	
	// Auto climb position
	// Preston's values: 1.045+0.04,4.28-0.1
	public final static Pose2d climb_tower_blue = new Pose2d(1.004, 4.660, Rotation2d.fromDegrees(0.0)); // units are meters
	public final static Pose2d before_Tower_Blue = new Pose2d(2.4, 4.6, Rotation2d.fromDegrees(0.0)); // units are meters

	public final static Translation2d hub_position_red = new Translation2d(11.924, 4.048);
	public final static Translation2d pass_position_red_1 = new Translation2d(15.48, 7.369);
	public final static Translation2d pass_position_red_2 = new Translation2d(15.715, 0.740);
	public final static Pose2d climb_tower_red = new Pose2d(15.478,5201, Rotation2d.fromDegrees(0));



	public static class Alliance_Find{

		public static Alliance alliance;
		public static Alliance prev_alliance;
		public static Translation2d hub = Field.hub_position_blue;
		public static Translation2d Pass_1 = pass_position_blue_1;
		public static Translation2d Pass_2 = pass_position_blue_2;
		public static Pose2d climb_tower = climb_tower_blue;
		static boolean updated = false;
		public static void setAlliance() {
			// Initialize the alliance-based values
			prev_alliance = alliance;
			updated = false;
			Optional<Alliance> tempAlliance = DriverStation.getAlliance();
			if(tempAlliance.isEmpty()) {
				alliance = Alliance.Blue;
			} else {
				alliance = tempAlliance.get();
			}
			if (prev_alliance != alliance) {
				updated = true;
			}
			if (updated){
			if(alliance == Alliance.Red){
				hub = Field.hub_position_red;
				Pass_1 = Field.pass_position_red_1;
				Pass_2 = Field.pass_position_red_2;
				climb_tower = Field.climb_tower_red;
				System.out.println("Switched To Red!");
			}
			else if(alliance == Alliance.Blue){
				hub = Field.hub_position_blue;
				Pass_1 = Field.pass_position_blue_1;
				Pass_2 = Field.pass_position_blue_2;
				climb_tower = Field.climb_tower_blue;
				System.out.println("Switched To Blue!");
			}
		}
		}
	}
}
