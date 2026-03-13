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
    
    static{
        riobus = new CANBus("rio");
        Canivore = new CANBus("CANivore"); //need to reaname to "vore"
    }

//All these values are temporary.
    public static class Intake{
        public static final int pivotID = 22;
        public static final int intakeID = 21;
        public static TalonFXConfiguration cfg_Roller = new TalonFXConfiguration();
        public static TalonFXConfiguration cfg_Pivot = new TalonFXConfiguration();
        static{
            cfg_Roller.Slot0.kP =0.6;
            cfg_Roller.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            cfg_Roller.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            cfg_Roller.MotionMagic.MotionMagicAcceleration = 50;
            cfg_Roller.MotionMagic.MotionMagicCruiseVelocity = 100;
            cfg_Roller.CurrentLimits.StatorCurrentLimit = 60;

            //cfg_Roller.CurrentLimits.StatorCurrentLimitEnable = false;
            //cfg_Roller.CurrentLimits.SupplyCurrentLimitEnable = true;
            //cfg_Roller.CurrentLimits.StatorCurrentLimit = 30.;
            
        }
        static{
            cfg_Pivot.Slot0.kP = -12.; //What :skull:
            

            cfg_Pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg_Pivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            cfg_Pivot.MotionMagic.MotionMagicAcceleration = 20;
            cfg_Pivot.MotionMagic.MotionMagicCruiseVelocity = 20;
            //cfg_Pivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//Will try this out, if not work will go back to the that fucking negative pid value(wtf ctre)
            cfg_Pivot.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Pivot.CurrentLimits.StatorCurrentLimit = 60.;

        }
    }

    public static class Hopper{
        public static final int HopperID = 25;
        public static TalonFXConfiguration cfg_Hopper = new TalonFXConfiguration();
        static{
            cfg_Hopper.Slot0.kP = 0.5;
            cfg_Hopper.Slot0.kV = .1;
            cfg_Hopper.MotionMagic.MotionMagicCruiseVelocity = 100;
            cfg_Hopper.MotionMagic.MotionMagicAcceleration = 50;
            cfg_Hopper.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Hopper.CurrentLimits.StatorCurrentLimit = 100;
            

            

        }
    }

    public static class Shooter{
        public static final int shooterflywheel_ID = 17;
        public static final int shooterflywheel2_ID = 18;
        public static final int shooterIntake_ID = 50;
        public static final TalonFXConfiguration cfg_shooter = new TalonFXConfiguration();
        public static final TalonFXConfiguration cfg_shooter_intake = new TalonFXConfiguration();
        /**
         * Interpolates between the robot's distance from the hub (in meters) and the velocity of the shooter rollers (in rotations/second)
         */
        public static final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();
        static{
            cfg_shooter.Slot0.kP = 0.53;
            // cfg_shooter.Slot0.kS = 0.2;
            cfg_shooter.Slot0.kV = 0.16;
            cfg_shooter_intake.Slot0.kP = 0.3;
            cfg_shooter_intake.Slot0.kV = 0.12;
            cfg_shooter.MotionMagic.MotionMagicAcceleration = 200;
            cfg_shooter.MotionMagic.MotionMagicCruiseVelocity = 200;
            cfg_shooter_intake.MotionMagic.MotionMagicCruiseVelocity = 160;
            cfg_shooter_intake.MotionMagic.MotionMagicAcceleration  = 160;
        }

        static{
            velocityTable.put(2.153688, -23.0);
            velocityTable.put(2.69777, -24.0);
            velocityTable.put(2.73, -24.5);
            velocityTable.put(3.308, -28.0);
            velocityTable.put(3.45, -28.0);

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
            cfg_Hood.Slot0.kP = 7.; //very snappy olala
            cfg_Hood.MotionMagic.MotionMagicCruiseVelocity = 100;
            cfg_Hood.MotionMagic.MotionMagicAcceleration = 160;
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
        public static final int Climber_ID = 30; //Make sure to change to like 20 to make the code nicer
        public static final TalonFXConfiguration cfg_Climber = new TalonFXConfiguration();

        static{
            cfg_Climber.Slot0.kP = 7;
            cfg_Climber.Slot0.kV = 0.12; // Feedforward for velocity - needed for Motion Magic
            cfg_Climber.MotionMagic.MotionMagicAcceleration = 15;
            cfg_Climber.MotionMagic.MotionMagicCruiseVelocity = 250;//Random number, doesn't really matter too
            cfg_Climber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg_Climber.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Climber.CurrentLimits.StatorCurrentLimit = 20;
        }
    

    }

    public static class DriveConstants{
        public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
	    public final  static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
	    public static final double kToleranceDegree = 0.5;
	    public static final double kToleranceSpeed = 0.01;

    }

    

}
