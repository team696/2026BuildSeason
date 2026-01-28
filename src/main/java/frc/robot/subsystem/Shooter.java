// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;

public class Shooter extends SubsystemBase {

  private static Shooter shooter = null;

  public static synchronized Shooter get(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }
  

  /** Creates a new ShooterAndTurret. */
      private final TalonFX m_Shooter = new TalonFX(BotConstants.Shooter.shooterflywheel_ID);
      private final TalonFX m_Hood = new TalonFX(BotConstants.Hood.Hood_ID);

      private final MotionMagicVelocityVoltage shooterVelocityController = new MotionMagicVelocityVoltage(0);
      private final MotionMagicVoltage hoodAngleController = new MotionMagicVoltage(0);

  public Shooter() {
      m_Shooter.getConfigurator().apply(BotConstants.Shooter.cfg_shooter);
      m_Hood.getConfigurator().apply(BotConstants.Hood.cfg_Hood);

  }

    double getDistanceToHub() {
      return Swerve.get().getPose().getTranslation().getDistance(Field.hub_position);
    }

  public Command setHoodAngle(double position){
    return run(() -> {m_Hood.setControl( 
      hoodAngleController.withPosition(position));});
  }

  public Command setHoodAngleForHub(){
    return setHoodAngle(BotConstants.Hood.shooterTable.get(getDistanceToHub()));
  }

  

  public Command set_velocity(double velocity){
    return run(()->{ m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));});
  }

  public Command setVelocityForHub(){
    return set_velocity(BotConstants.Shooter.velocityTable.get(getDistanceToHub()));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
