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
    return run(()->{ m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));});
  }
  //Intakes the ball for the shooter
  public Command intake_shooter(){
    return run(()->{m_ShooterIntake.setControl(intakeRollerController.withVelocity(1));});
  }
  //Only used for auto
  public Command Shoot(double velocity, double position_hood){
    return run(()->{
      m_ShooterIntake.setControl(intakeRollerController.withVelocity(30));
      m_Shooter.setControl(shooterVelocityController.withVelocity(-velocity));
      m_Hood.setControl(hoodAngleController.withPosition(position_hood));
     });
    }
    public Command Shoot(){

      return run(()->{
        double distMeters=Swerve.get().getPose().getTranslation().getDistance(Field.Alliance_Find.hub);
        m_Shooter.setControl(shooterVelocityController.withVelocity(BotConstants.Shooter.velocityTable.get(distMeters)));
        m_Hood.setControl(hoodAngleController.withPosition(Hood.shooterTable.get(distMeters)));
        // TODO: wait for the shooter to go up to speed and the hood to reach the right angle before the line below runs (the line below releases the gamepiece).
        m_ShooterIntake.setControl(intakeRollerController.withVelocity(30));

      });
    }

  //Stops everything
  public Command Stop(){
    return run(()->{
      m_Shooter.stopMotor();
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
