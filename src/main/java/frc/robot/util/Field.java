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

	public final static Translation2d hub_position = new Translation2d(4.4, 3.2);
}