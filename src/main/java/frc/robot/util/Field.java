package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;

import java.util.NoSuchElementException;
import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.path.TravelingSalesman;
import edu.wpi.first.math.util.Units;
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
	public final static Pose2d climb_tower_blue = new Pose2d(Units.inchesToMeters(91.055),Units.inchesToMeters(147.47), Rotation2d.fromDegrees(0));

	public final static Translation2d hub_position_red = new Translation2d(11.924, 4.048);
	public final static Translation2d pass_position_red_1 = new Translation2d(15.48, 7.369);
	public final static Translation2d pass_position_red_2 = new Translation2d(15.715, 0.740);
	public final static Pose2d climb_tower_red = FlippingUtil.flipFieldPose(climb_tower_blue);




	public static class Alliance_Find{

		public static Optional<Alliance> alliance = DriverStation.getAlliance();
		public static Translation2d hub;
		public static Translation2d Pass_1;
		public static Translation2d Pass_2;
		public static Pose2d climb_tower;

		public Alliance_Find(){

			try{
				if(alliance.get() == Alliance.Red){
					hub = Field.hub_position_red;
					Pass_1 = Field.pass_position_red_1;
					Pass_2 = Field.pass_position_red_2;
					climb_tower = Field.climb_tower_red;
				}
				else if(alliance.get() == Alliance.Blue){
					hub = Field.hub_position_blue;
					Pass_1 = Field.pass_position_blue_1;
					Pass_2 = Field.pass_position_blue_2;
					climb_tower = Field.climb_tower_blue;
				}
				else{
					System.out.print("You fucked up");
				}
			} catch (NoSuchElementException e){
					DriverStation.reportWarning("Could not get alliance from driver station/FMS!", e.getStackTrace());
					hub = Field.hub_position_blue;
					Pass_1 = Field.pass_position_blue_1;
					Pass_2 = Field.pass_position_blue_2;
					climb_tower = Field.climb_tower_blue;
			}

		}
	}
}
