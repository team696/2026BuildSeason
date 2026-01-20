// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;

public class ShooterAndTurret extends SubsystemBase {

  private static ShooterAndTurret shooter_and_turret = null;

  public static synchronized ShooterAndTurret get(){
    if (shooter_and_turret == null){
      shooter_and_turret = new ShooterAndTurret();
    }
    return shooter_and_turret;
  }
  

  /** Creates a new ShooterAndTurret. */
      private final TalonFX m_Shooter = new TalonFX(BotConstants.Shooter.shooterflywheel_ID);
      private final TalonFX m_Hood = new TalonFX(BotConstants.Hood.Hood_ID);
      private final TalonFX m_Turret = new TalonFX(BotConstants.Turret.Turret_ID);
      
  public ShooterAndTurret() {
      m_Shooter.getConfigurator().apply(BotConstants.Shooter.cfg_shooter);
      m_Hood.getConfigurator().apply(BotConstants.Hood.cfg_Hood);
      m_Turret.getConfigurator().apply(BotConstants.Turret.cfg_turret);

  }

  public Rotation2d target_theta(){
      return new Rotation2d(Math.atan2(
        Field.hub_position.getY() - Swerve.get().getPose().getY(),
        Field.hub_position.getX() - Swerve.get().getPose().getX()
        )).minus(Swerve.get().getPose().getRotation());
    }


  public Command point_to_hub(){
    return run(()->{m_Turret.setControl(new MotionMagicVoltage(target_theta().getRotations()));});
  }

  //This is stupidly long lol
  //This is bascially the interpolation table with the input of distance formula
  public Command set_hood_angle(){
    return run(() -> {m_Hood.setControl(
      new MotionMagicVoltage(BotConstants.Hood.shooterTable.get(Math.sqrt(
      Math.pow(Field.hub_position.getX() - Swerve.get().getPose().getX(),2) 
      - Math.pow(Field.hub_position.getY() - Swerve.get().getPose().getY(),2)))));});

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
