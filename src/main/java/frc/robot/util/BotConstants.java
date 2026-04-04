// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
        public static final int pivotID = 23;
        public static final int intakeID = 21;
        public static final int intakeID_2 = 22;
        public static TalonFXConfiguration cfg_Roller = new TalonFXConfiguration();
        public static TalonFXConfiguration cfg_Pivot = new TalonFXConfiguration();
        static{
            cfg_Roller.Slot0.kP = 0.5;
            cfg_Roller.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            cfg_Roller.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            cfg_Roller.MotionMagic.MotionMagicAcceleration = 100;
            cfg_Roller.MotionMagic.MotionMagicCruiseVelocity = 200;
            cfg_Roller.CurrentLimits.StatorCurrentLimit = 120;

            //cfg_Roller.CurrentLimits.StatorCurrentLimitEnable = false;
            //cfg_Roller.CurrentLimits.SupplyCurrentLimitEnable = true;
            //cfg_Roller.CurrentLimits.StatorCurrentLimit = 30.;

            cfg_Pivot.Slot0.kP = 15.; //YES ITS NORNMAL PID VALUES NOW
            cfg_Pivot.Slot1.kP = 10; 
            cfg_Pivot.Slot1.kG = -.08*12;
            cfg_Pivot.Slot1.kI=.001; //Was 0
            cfg_Pivot.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
            
            cfg_Pivot.Feedback.SensorToMechanismRatio = 15;

            cfg_Pivot.Slot2.kP = 11;
            cfg_Pivot.Slot2.kG = -0.08*12;


            cfg_Pivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg_Pivot.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            cfg_Pivot.MotionMagic.MotionMagicAcceleration = 5;
            cfg_Pivot.MotionMagic.MotionMagicCruiseVelocity = 2;
            //cfg_Pivot.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//Will try this out, if not work will go back to the that fucking negative pid value(wtf ctre)
            cfg_Pivot.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Pivot.CurrentLimits.StatorCurrentLimit = 80;

        }
    }

    public static class Hopper{
        public static final int HopperID = 25;
        public static TalonFXConfiguration cfg_Hopper = new TalonFXConfiguration();
        static{
            cfg_Hopper.Slot0.kP = 1;
            cfg_Hopper.Slot0.kV = .1;
            cfg_Hopper.MotionMagic.MotionMagicCruiseVelocity = 100;
            cfg_Hopper.MotionMagic.MotionMagicAcceleration = 10;
            cfg_Hopper.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Hopper.CurrentLimits.StatorCurrentLimit = 100;
            

            

        }
    }

    public static class Shooter{
        public static final int shooterflywheel_ID = 17;
        public static final int shooterflywheel2_ID = 18;
        public static final int shooterIntake_ID = 19;
        public static final TalonFXConfiguration cfg_shooter = new TalonFXConfiguration();
        public static final TalonFXConfiguration cfg_shooter_intake = new TalonFXConfiguration();
        /**
         * Interpolates between the robot's distance from the hub (in meters) and the velocity of the shooter rollers (in rotations/second)
         */
        public static final InterpolatingDoubleTreeMap ShooterTable = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap backSpinTable = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap fixedVelocityTable = new InterpolatingDoubleTreeMap();

        static{
            cfg_shooter.Slot0.kP = 0.2;
            cfg_shooter.Slot0.kS = 0.1;
            cfg_shooter.Slot0.kV = 0.13;
            cfg_shooter_intake.Slot0.kP = 0.45;
            cfg_shooter_intake.Slot0.kV = 0.12;
            cfg_shooter.MotionMagic.MotionMagicAcceleration = 200;
            cfg_shooter.MotionMagic.MotionMagicCruiseVelocity = 200;
            cfg_shooter_intake.MotionMagic.MotionMagicCruiseVelocity = 160;
            cfg_shooter_intake.MotionMagic.MotionMagicAcceleration  = 160;
            cfg_shooter.CurrentLimits.StatorCurrentLimit = 150;
        }

        static{
            ShooterTable.put(2.257, -30.0);
            ShooterTable.put(2.206, -30.0);
            ShooterTable.put(1.607, -25.0);
            ShooterTable.put(3.060, -30.5);
            ShooterTable.put(2.744, -29.5);

            backSpinTable.put(2.257, -20.0); // key is distance to hub, value is the small roller velocity
            backSpinTable.put(2.206, -20.0);
            backSpinTable.put(1.607, -22.0);
            backSpinTable.put(3.060, -29.5);
            backSpinTable.put(2.744, -27.0);
        }

    }


    public static class Climber{
        public static final int Climber_ID = 30; //Make sure to change to like 20 to make the code nicer
        public static final TalonFXConfiguration cfg_Climber = new TalonFXConfiguration();

        static{
            cfg_Climber.Slot0.kP = 2.5;
            cfg_Climber.Slot0.kV = 0.12; // Feedforward for velocity - needed for Motion Magic
            cfg_Climber.MotionMagic.MotionMagicAcceleration = 160; //Bumped this up cuz oml it is so slow
            cfg_Climber.MotionMagic.MotionMagicCruiseVelocity = 200;//Random number, doesn't really matter too
            cfg_Climber.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg_Climber.CurrentLimits.StatorCurrentLimitEnable = true;
            cfg_Climber.CurrentLimits.StatorCurrentLimit = 20;
        }
    

    }

    public static class DriveConstants{
        public final static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
	    public final  static double MaxAngularRate = RotationsPerSecond.of(5.
        
        
        
        
        
        
        
        
        ).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	
	    public static final double kToleranceDegree = 0.5;
	    public static final double kToleranceSpeed = 0.01;

    }

    

}