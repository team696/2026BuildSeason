
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

public class Intake extends SubsystemBase {

  private static Intake intake = null;

  //Prevents the need of duplicate objects
  public static synchronized Intake get(){
    if(intake == null){
      intake = new Intake();
    } 
    return intake;
  }

  /** Creates a new Intake. */

  //Enum to determin state, values are temporary
  public static enum State{
    IDLE(0.0),
    INTAKE(3000.0),
    OUTTAKE(-3000.0);

    public double roller_velocity;  // Renamed
    State(double roller_velocity){
        this.roller_velocity = roller_velocity;
    }
}
  //Enum to determin pivot position, values are temporary
  public enum Pivot{
    STOW(0.00),
    DEPLOY(5.5);

    public double position;

    Pivot(double position){
      this.position = position;
    }
  }
  

  //Motors
  private final TalonFX m_IntakePivot = new TalonFX(BotConstants.Intake.pivotID, BotConstants.Canivore);
  private final TalonFX m_IntakeRoller = new TalonFX(BotConstants.Intake.intakeID, BotConstants.Canivore);
  //Motor Controller
  private final PositionVoltage PivotPositionControl = new PositionVoltage(0);
  private final MotionMagicVelocityVoltage intakeVelocityController  = new MotionMagicVelocityVoltage(0);

  public Intake() {
    m_IntakeRoller.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot);
    m_IntakePivot.setPosition(0.0);

    this.setDefaultCommand(doStow());
  }

  public void runIntake(State state) {
    m_IntakeRoller.setControl(intakeVelocityController.withVelocity(state.roller_velocity/60));
  }

    public void positionIntake(double state) {
    m_IntakePivot.setControl(PivotPositionControl.withPosition(state)); //
    //vout.withOutput(-1 * pidController.calculate(m_IntakePivot.getPosition().getValueAsDouble(), state))
  }

  public Command doIntake() {
    return this.run(() -> {
        this.runIntake(State.INTAKE); this.positionIntake(5.7);
    });
  }

public Command doStow() {
    return this.run(() -> {
        m_IntakeRoller.stopMotor(); this.positionIntake(0.);
    });
  }

@Override
public void periodic() {
    SmartDashboard.putNumber("Pivot Position", m_IntakePivot.getPosition().getValueAsDouble());
    SmartDashboard.putData(this);
}
}