package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
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

	public final static Translation2d hub_position_red = new Translation2d(11.924, 4.048);
	public final static Translation2d pass_position_red_1 = new Translation2d(15.48, 7.369);
	public final static Translation2d pass_position_red_2 = new Translation2d(15.715, 0.740);



}