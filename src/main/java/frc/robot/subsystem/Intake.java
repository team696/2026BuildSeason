
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
    INTAKE(70.0);

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
  private final TalonFX m_IntakePivot = new TalonFX(BotConstants.Intake.pivotID);
  private final TalonFX m_IntakeRoller = new TalonFX(BotConstants.Intake.intakeID);
  //Motor Controller
   private final MotionMagicVoltage PivotPositionControl = new MotionMagicVoltage(0);
  private final PositionVoltage pivotPosition = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage intakeVelocityController  = new MotionMagicVelocityVoltage(0);


  private final FlywheelSim m_FlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.008, 1.0), DCMotor.getKrakenX60(2), 0.008);
  private final SingleJointedArmSim m_IntakePivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 15, 1.846666667, 0.28, -2.555, 0.0, false, 0.0);

  public Intake() {


    m_IntakeRoller.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot);
    m_IntakePivot.setPosition(0.0);

    //this.setDefaultCommand(doStow());
  }

  public Command test(){
    return this.runEnd(()->{
      m_IntakeRoller.setControl(intakeVelocityController.withVelocity(40));
      m_IntakePivot.setControl(pivotPosition.withPosition(-5));
    },
    ()->{m_IntakePivot.stopMotor();
        m_IntakeRoller.stopMotor();});
  }


  public void runIntake(State state) {
    m_IntakeRoller.setControl(intakeVelocityController.withVelocity(state.roller_velocity));
  }

  public void positionIntake(double state) {
    m_IntakePivot.setControl(PivotPositionControl.withPosition(state)); //
    //vout.withOutput(-1 * pidController.calculate(m_IntakePivot.getPosition().getValueAsDouble(), state))
  }

  public Command zeroEncoder() {
    return run(()->{m_IntakePivot.setPosition(0);});
}

public Command doStow() {
    return this.runEnd(() -> {
      m_IntakeRoller.stopMotor();
      m_IntakePivot.setControl(pivotPosition.withPosition(0));     
    },
    ()->{

    }
    );
  }
  public Command doIntake() {
    return this.runEnd(() -> {
        this.runIntake(State.INTAKE); 
        this.positionIntake(-6); 
    }, 
    ()->{
        this.doStow();});
  }




//Sim stuff need to delete
@Override
public void simulationPeriodic(){
    double rollervoltage = m_IntakeRoller.getSimState().getMotorVoltage();
    m_FlywheelSim.setInputVoltage(rollervoltage);
    m_FlywheelSim.update(0.020); 

    m_IntakeRoller.getSimState().setRotorVelocity(
        m_FlywheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI) 
    );
    m_IntakeRoller.getSimState().setSupplyVoltage(12.0);

    double pivotVoltage = m_IntakePivot.getSimState().getMotorVoltage();
    m_IntakePivotSim.setInput(pivotVoltage);
    
    m_IntakePivotSim.update(0.020);

    double motorPos = -1.0*(m_IntakePivotSim.getAngleRads() * 15.0) / (2 * Math.PI);
    m_IntakePivot.getSimState().setRawRotorPosition(motorPos);
    
    
    m_IntakePivot.getSimState().setSupplyVoltage(12.0);
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Intake Pivot current", m_IntakePivot.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake pose", m_IntakePivot.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Roller speed", m_IntakeRoller.getVelocity().getValueAsDouble());
}
}