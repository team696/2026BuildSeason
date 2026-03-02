// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;
import frc.robot.util.BotConstants.Hood;

public class Shooter extends SubsystemBase {

  private static Shooter shooter = null;

  public static synchronized Shooter get(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }
  

  /** Creates a new Shooter. */
      //Motors
      private final TalonFX m_Shooter = new TalonFX(BotConstants.Shooter.shooterflywheel_ID);
      private final TalonFX m_Shooter_2 = new TalonFX(BotConstants.Shooter.shooterflywheel2_ID);
      private final TalonFX m_Hood = new TalonFX(BotConstants.Hood.Hood_ID);
      private final TalonFX m_ShooterIntake = new TalonFX(BotConstants.Shooter.shooterIntake_ID);
      //Controllers
      private final MotionMagicVelocityVoltage shooterVelocityController = new MotionMagicVelocityVoltage(0);
      private final MotionMagicVoltage hoodAngleController = new MotionMagicVoltage(0);
      private final MotionMagicVelocityVoltage intakeRollerController = new MotionMagicVelocityVoltage(0);
      //Data
      private StatusSignal<Angle> position_hood;
      private StatusSignal<AngularVelocity> velocity_roller;

  public Shooter() {
      //Config set up
      m_Shooter.getConfigurator().apply(BotConstants.Shooter.cfg_shooter);
      m_Hood.getConfigurator().apply(BotConstants.Hood.cfg_Hood);
      m_ShooterIntake.getConfigurator().apply(BotConstants.Shooter.cfg_shooter_intake);
      
      position_hood = m_Hood.getPosition();
      velocity_roller = m_Shooter.getVelocity();
  }

  //Sets the hood angle
  public Command setHoodAngle(double position_hood){
    return run(() -> {m_Hood.setControl( 
      hoodAngleController.withPosition(position_hood));});
  }
  //Sets the velocity
  public Command set_velocity(double velocity){
    return run(()->{ m_Shooter.setControl(shooterVelocityController.withVelocity(velocity)); m_Shooter_2.setControl(new Follower(BotConstants.Shooter.shooterflywheel_ID,MotorAlignmentValue.Opposed));});
  }
  //Intakes the ball for the shooter
  public Command intake_shooter(double speed){
    return run(()->{m_ShooterIntake.setControl(intakeRollerController.withVelocity(speed));});
  }

  public Command idle(){
    return setHoodAngle(0);
  }


  public Command Shoot(Translation2d desired_pose){
      return runEnd(()->{
        double distMeters=Swerve.get().distTo(desired_pose);
        setHoodAngle(BotConstants.Hood.shooterTable.get(distMeters));
        set_velocity(BotConstants.Shooter.velocityTable.get(distMeters));
        if(getRollerVelocity()<0.01){
          intake_shooter(80);//man why did they make it so high
          Hopper.get().run_Hopper();
        }

      },
      ()->{
        Stop();
        idle();
        Hopper.get().Stop();
      });
    }

   //Took sami's version for testing purposes. Might change
    public Command ShootDash(){
    return runEnd(
        () -> {
            double velocity=SmartDashboard.getNumber("Launch Speed", 0);
            double position_hood = SmartDashboard.getNumber("Hood angle", 0);
            //m_ShooterIntake.setVoltage(5.0);
            m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));
            //m_Shooter.setVoltage(5.0);
            m_Hood.setControl(hoodAngleController.withPosition(position_hood));
            m_Shooter_2.setControl(new Follower(17, MotorAlignmentValue.Opposed));
            System.out.println(getRollerVelocity()+","+ velocity);
            if((Math.abs(getRollerVelocity()-velocity)/velocity)<0.001){
              m_ShooterIntake.setControl(intakeRollerController.withVelocity(60));
            }

        },
        () -> {
            m_Shooter.stopMotor();
            m_Hood.stopMotor();
            m_ShooterIntake.stopMotor();
            m_Shooter_2.stopMotor();
        }
    );
}

  //Stops everything
  public Command Stop(){
    return run(()->{
      m_Shooter.stopMotor();
      m_Shooter_2.stopMotor();
      m_Hood.stopMotor();
      m_ShooterIntake.stopMotor();
    });
  }

  //Data stuff
  public double getHoodPosition(){
    return position_hood.refresh().getValueAsDouble();
  }
 
  public double getRollerVelocity(){
    return velocity_roller.refresh().getValueAsDouble();
  }

  @Override
  public void periodic() {
    //Data stuff used in Autoalign
    getHoodPosition();
    getRollerVelocity();
  }
}

  //  SmartDashboard.putNumber("Hood angle", 0);
//       SmartDashboard.putNumber("Launch Speed", 0);
//   }



//   //Sets the hood angle
//   public Command setHoodAngle(double position_hood){
//     return run(() -> {m_Hood.setControl( 
//       hoodAngleController.withPosition(position_hood));});
//   }
//   //Sets the velocity
//   public Command set_velocity(double velocity){
//     return run(()->{ m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));});
//   }
//   //Intakes the ball for the shooter
//   public Command intake_shooter(){
//     return run(()->{m_ShooterIntake.setControl(intakeRollerController.withVelocity(1));});
//   }

//   public Command idle(){
//     return setHoodAngle(0);
//   }
//   //Only used for auto
//   public Command ShootDash(){
//     return runEnd(
//         () -> {
//             double velocity=SmartDashboard.getNumber("Launch Speed", 0);
//             double position_hood = SmartDashboard.getNumber("Hood angle", 0);
//             //m_ShooterIntake.setVoltage(5.0);
//             m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));
//             //m_Shooter.setVoltage(5.0);
//             m_Hood.setControl(hoodAngleController.withPosition(position_hood));
//             m_Shooter_2.setControl(new Follower(17, MotorAlignmentValue.Opposed));
//             System.out.println(getRollerVelocity()+","+ velocity);
//             if((Math.abs(getRollerVelocity()-velocity)/velocity)<0.001){
//               m_ShooterIntake.setControl(intakeRollerController.withVelocity(60));
//             }

//         },
//         () -> {
//             m_Shooter.stopMotor();
//             m_Hood.stopMotor();
//             m_ShooterIntake.stopMotor();
//             m_Shooter_2.stopMotor();
//         }
//     );
// }
// public Command Shoot(double velocity, double position_hood){
//     return runEnd(
//         () -> {
//             //m_ShooterIntake.setVoltage(5.0);
//             m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));
//             //m_Shooter.setVoltage(5.0);
//             m_Hood.setControl(hoodAngleController.withPosition(position_hood));
//             m_Shooter_2.setControl(new Follower(17, MotorAlignmentValue.Opposed));
//             System.out.println(getRollerVelocity()+","+ velocity);
//             if((Math.abs(getRollerVelocity()-velocity)/velocity)<0.001){
//               m_ShooterIntake.setControl(intakeRollerController.withVelocity(60));
//             }

//         },
//         () -> {
//             m_Shooter.stopMotor();
//             m_Hood.stopMotor();
//             m_ShooterIntake.stopMotor();
//             m_Shooter_2.stopMotor();
//         }
//     );
// }

//   //Stops everything
//   public Command Stop(){
//     return run(()->{
//       m_Shooter.stopMotor();
//       m_Hood.stopMotor();
//       m_ShooterIntake.stopMotor();
//     });
//   }

//   //Data stuff
//   public double getHoodPosition(){
//     return position_hood.refresh().getValueAsDouble();
//   }
 
//   public double getRollerVelocity(){
//     return velocity_roller.refresh().getValueAsDouble();
//   }

//   @Override
//   public void periodic() {
//     //Data stuff used in Autoalign
//     getHoodPosition();
//     getRollerVelocity();
//   }
// }
