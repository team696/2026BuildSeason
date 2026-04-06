package frc.robot.util;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.TunerConstants;


public class constants {    
	
	public class FieldConstants{
	public static final Field2d SIM = new Field2d();

	public static final Distance LENGTH = Feet.of(57.53);
	public static final Distance WIDTH = Feet.of(26.75);

	// public final static Translation2d hub_position = new Translation2d(Units.inchesToMeters(182.11), 
	// 												Units.inchesToMeters(158.84));

	}

	public class DriveConstants {
	public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
	public final  static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
	public static final double kToleranceDegree = 0.5;
	public static final double kToleranceSpeed = 0.01;

		

	}

}    