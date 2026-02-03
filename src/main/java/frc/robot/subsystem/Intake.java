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

  //Prevents the need of duplicate objects
  public static synchronized Intake get(){
    if(intake == null){
      intake = new Intake();
    } 
    return intake;
  }

  /** Creates a new Intake. */

  //Enum to determin state, values are temporary
  public enum State{
    IDLE(0.0),
    INTAKE(1.0),
    OUTTAKE(-1.0);

    public double roller_voltage;
    State(double roller_voltage){
      this.roller_voltage = roller_voltage;
    }
  }
  //Enum to determin pivot position, values are temporary
  public enum Pivot{
    STOW(0.0),
    DEPLOY(45.0);

    public double angle;

    Pivot(double angle){
      this.angle = angle;
    }
  }
  

  //Motors
  private final TalonFX m_IntakePivot = new TalonFX(BotConstants.Intake.pivotID);
  private final TalonFX m_IntakeRoller = new TalonFX(BotConstants.Intake.intakeID);
  //Motor Controller
  private final MotionMagicVoltage PivotPositionControl = new MotionMagicVoltage(0);
  //Variables getting the values
  private State mState = State.IDLE;
  private Pivot mPivot = Pivot.STOW;

  //Constructor, just sets up the config
  public Intake() {
    m_IntakeRoller.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot);
  }

  //Set roller and pivot state together
  public Command setState(State state, Pivot pivot){
    return runOnce(() -> {
      mState = state;
      mPivot = pivot;
    });
  }

  //Stows, basically sets everything to 0
  public Command stow(){
    return setState(State.IDLE, Pivot.STOW);
  }


  @Override
  //Sets the values
  public void periodic() {
    m_IntakePivot.setControl(PivotPositionControl.withPosition(mPivot.angle/360)); //Have to divide by 360 because the angle of the pivot is going to be set in degrees, but is up to changees
    m_IntakeRoller.setVoltage(mState.roller_voltage);
  }
}
