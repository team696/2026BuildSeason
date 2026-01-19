// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


/** Add your docs here. */
public class BotConstants {
    public static CANBus riobus;
    public static CANBus Canivore;
    public static TalonFXConfiguration cfg = new TalonFXConfiguration();
    
    static{
        riobus = new CANBus("rio");
        Canivore = new CANBus("cv");
    }

//All these values are temporary.
    public static class Intake{
        public static final int pivotID = 1;
        public static final int intakeID = 2;
        public static TalonFXConfiguration cfg = new TalonFXConfiguration();
        public static TalonFXConfiguration cfg_2 = new TalonFXConfiguration();
        static{
            cfg.Slot0.kP = 0.0;
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            cfg.MotionMagic.MotionMagicAcceleration = 40;
            cfg.MotionMagic.MotionMagicCruiseVelocity = 20;
            cfg.CurrentLimits.StatorCurrentLimitEnable = false;
            cfg.CurrentLimits.SupplyCurrentLimitEnable = false;
            cfg.CurrentLimits.StatorCurrentLimit = 120.;
        }
        static{
            cfg_2.Slot0.kP = 0.0;
            cfg_2.Slot0.kI = 0.0;
            cfg_2.Slot0.kP = 0.0;
        }
    }

    public static class Hopper{
        public static final int HopperID = 3;
        public static final int MagazineID = 3;
        public static final int HopperBeamBreakID = 0;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    public static class Shooter{
        public static final int shooterflywheel_1_ID = 4;
        public static final int shooterflywheel_2_ID = 5;

        static{
            //Tis where the config will go, too lazy to write it rn
        }

    }

    public static class Hood{
        public static final  int Hood_ID = 6;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    public static class Turret{
        public static final int Turret_ID = 7;
        public static final int Turret_BeamBreakID = 1;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    public static class Climber{
        public static final int Climber_1_ID = 8;
        public static final int Climber_2_ID = 9;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    

}
