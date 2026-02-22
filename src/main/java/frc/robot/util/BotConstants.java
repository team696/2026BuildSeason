// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.TunerConstants;


/** Add your docs here. */
public class BotConstants {
    public static CANBus riobus;
    public static CANBus Canivore;
    public static TalonFXConfiguration cfg_Roller = new TalonFXConfiguration();
    
    static{
        riobus = new CANBus("rio");
        Canivore = new CANBus("CANivore");
    }

//All these values are temporary.
    public static class Intake{
        public static final int pivotID = 22;
        public static final int intakeID = 21;
        public static TalonFXConfiguration cfg_Roller = new TalonFXConfiguration();
        public static TalonFXConfiguration cfg_Pivot = new TalonFXConfiguration();
        static{
            cfg_Roller.Slot0.kP = .45;
            cfg_Roller.Slot0.kV = .1;
            cfg_Roller.Slot0.kS = .1;
            cfg_Roller.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            cfg_Roller.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            cfg_Roller.MotionMagic.MotionMagicAcceleration = 3000.0 / 60.0;
            cfg_Roller.MotionMagic.MotionMagicCruiseVelocity = 6000.0 / 60.0;
            cfg_Roller.CurrentLimits.StatorCurrentLimitEnable = false;
            cfg_Roller.CurrentLimits.SupplyCurrentLimitEnable = true;
            cfg_Roller.CurrentLimits.StatorCurrentLimit = 30.;
    
            cfg_Pivot.Slot0.kP = -7.;
            cfg_Pivot.Slot0.kD = 0.0;

            cfg_Pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg_Pivot.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Pivot.CurrentLimits.StatorCurrentLimit = 60.;
        }
        static{
            cfg_Pivot.Slot0.kP = 0.0;
            cfg_Pivot.Slot0.kI = 0.0;
            cfg_Pivot.Slot0.kP = 0.0;
        }
    }

    public static class Hopper{
        public static final int HopperID = 3;
        public static final int HopperBeamBreakID = 0;

        static{
            
        }
    }

    public static class Shooter{
        public static final int shooterflywheel_ID = 17;
        public static final int shooterIntake_ID = 19;
        public static final TalonFXConfiguration cfg_shooter = new TalonFXConfiguration();
        public static final TalonFXConfiguration cfg_shooter_intake = new TalonFXConfiguration();
        /**
         * Interpolates between the robot's distance from the hub (in meters) and the velocity of the shooter rollers (in rotations/second)
         */
        public static final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();
        static{
            cfg_shooter.Slot0.kP = 0.3;
            cfg_shooter.Slot0.kV = 0.12;
            cfg_shooter_intake.Slot0.kP = 0.3;
            cfg_shooter_intake.Slot0.kV = 0.12;
            cfg_shooter.MotionMagic.MotionMagicAcceleration = 100;
            cfg_shooter.MotionMagic.MotionMagicCruiseVelocity = 160;
            cfg_shooter_intake.MotionMagic.MotionMagicCruiseVelocity = 160;
            cfg_shooter_intake.MotionMagic.MotionMagicAcceleration  = 100;
        }

        static{
            velocityTable.put(0.0, 0.0);
            velocityTable.put(1.0, 1.0);
            velocityTable.put(1.0, 1.0);
            velocityTable.put(1.0, 1.0);
            velocityTable.put(1.0, 1.0);
            velocityTable.put(1.0, 1.0);

        }

    }

    public static class Hood{
        public static final int Hood_ID = 16;
        public static final TalonFXConfiguration cfg_Hood = new TalonFXConfiguration();
        /**
         * Interpolates between distance from the hub (in meters) and the angle of the shooter (in degrees)
         */
        public static final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
        static{
            cfg_Hood.Slot0.kP = 5.;
        }

        static{
            shooterTable.put(0.0,0.0);
            shooterTable.put(1.0,1.0);
            shooterTable.put(1.0,1.0);
            shooterTable.put(1.0,1.0);
            shooterTable.put(1.0,1.0);
            
        }

    }

    public static class Climber{
        public static final int Climber_1_ID = 8;
        public static final int Climber_2_ID = 9;
        public static final TalonFXConfiguration cfg_Climber1 = new TalonFXConfiguration();
        public static final TalonFXConfiguration cfg_Climber2 = new TalonFXConfiguration();
        public static final double gearRatio = 1;
        public static final double chainLength = .5; // in meters
        public static final double sproketRadius = .253; // in meters
        public static final double sproketDiameter = 2*Math.PI*sproketRadius; // in meters

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    

    }

    public static class DriveConstants{
        public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
	    public final  static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
	    public static final double kToleranceDegree = 0.5;
	    public static final double kToleranceSpeed = 0.01;

    }

    

}
