// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

public class Intake extends SubsystemBase {

  private static Intake intake = null;

  private static synchronized Intake get(){
    if(intake == null){
      intake = new Intake();
    } 
    return intake;
  }

  /** Creates a new Intake. */
  private enum State{
    IDLE(0.0),
    INTAKE(1.0),
    OUTTAKE(-1.0);

    public double roller_voltage;

    State(double roller_voltage){
      this.roller_voltage= roller_voltage;
    }
  }

  private enum Pivot{
    STOW(0.0),
    DEPLOY(45.0);

    public double angle;

    Pivot(double angle){
      this.angle = angle;
    }
  }
  

  private final TalonFX m_IntakePivot = new TalonFX(BotConstants.Intake.pivotID);
  private final TalonFX m_IntakeRoller = new TalonFX(BotConstants.Intake.intakeID);

  private final MotionMagicVoltage PivotPositionControl = new MotionMagicVoltage(0);

  private State mState = State.IDLE;
  private Pivot mPivot = Pivot.STOW;

  public Intake() {
    m_IntakeRoller.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot);
  }

  public Command setState(State state, Pivot pivot){
    return runOnce(() -> {
      mState = state;
      mPivot = pivot;
    });
  }

  public Command stow(){
    return setState(State.IDLE, Pivot.STOW);
  }


  @Override
  public void periodic() {
    m_IntakePivot.setControl(PivotPositionControl.withPosition(mPivot.angle/360));
    m_IntakeRoller.setVoltage(mState.roller_voltage);
  }
}
