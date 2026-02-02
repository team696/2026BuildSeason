// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
      private final TalonFX m_ShooterIntake = new TalonFX(BotConstants.Shooter.shooterIntake_ID);

      private final MotionMagicVelocityVoltage shooterVelocityController = new MotionMagicVelocityVoltage(0);
      private final MotionMagicVoltage hoodAngleController = new MotionMagicVoltage(0);
      private final MotionMagicVelocityVoltage intakeRollerController = new MotionMagicVelocityVoltage(0);

      private StatusSignal<Angle> position_hood;
      private StatusSignal<AngularVelocity> velocity_roller;

  public Shooter() {
      m_Shooter.getConfigurator().apply(BotConstants.Shooter.cfg_shooter);
      m_Hood.getConfigurator().apply(BotConstants.Hood.cfg_Hood);
      m_ShooterIntake.getConfigurator().apply(BotConstants.Shooter.cfg_shooter_intake);

      position_hood = m_Hood.getPosition();
      velocity_roller = m_Shooter.getVelocity();


  }

  double getDistanceToHub() {
      return Swerve.get().getPose().getTranslation().getDistance(Field.hub_position_blue);
    }

  public Command setHoodAngle(double position_hood){
    return run(() -> {m_Hood.setControl( 
      hoodAngleController.withPosition(position_hood));});
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

  public Command intake_shooter(){
    return run(()->{m_ShooterIntake.setControl(intakeRollerController.withVelocity(1));});
  }


  public Command Stop(){
    return run(()->{
      m_Shooter.stopMotor();
      m_Hood.stopMotor();
      m_ShooterIntake.stopMotor();
    });
  }

  public double getHoodPosition(){
    return position_hood.refresh().getValueAsDouble();
  }
 
  public double getRollerVelocity(){
    return velocity_roller.refresh().getValueAsDouble();
  }

  @Override
  public void periodic() {
    getHoodPosition();
    getRollerVelocity();
    // This method will be called once per scheduler run
  }
}
