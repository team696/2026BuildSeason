
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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


  public static enum State{
    IDLE(0.0),
    INTAKE(60.0),
    OUTTAKE(-75.0); //S P E E D

    public double roller_velocity; 
    State(double roller_velocity){
        this.roller_velocity = roller_velocity;
    }
}
  //Enum to determin pivot position, values are temporary
  public enum Pivot{
    //STOW(-0.558),
    STOW(0),
    DEPLOY(-0.42); //position flipped cuz now we counter clock wise positive

    public double position;

    Pivot(double position){
      this.position = position+0.08;
    }
  }
  

  //Motors
  private final TalonFX m_IntakeRoller = new TalonFX(BotConstants.Intake.intakeID);
  private final TalonFX m_IntakeRoller_2 = new TalonFX(BotConstants.Intake.intakeID_2);
  private final TalonFX m_IntakePivot = new TalonFX(BotConstants.Intake.pivotID);

  //Motor Controller
   private final MotionMagicVoltage PivotPositionControl = new MotionMagicVoltage(0);
  private final MotionMagicVoltage pivotPosition = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage intakeVelocityController  = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage intakeVelocityController2  = new MotionMagicVelocityVoltage(0);
  


  private final FlywheelSim m_FlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.008, 1.0), DCMotor.getKrakenX60(2), 0.008);
  private final SingleJointedArmSim m_IntakePivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 15, 1.846666667, 0.28, 0, 2.5555, false, 0.0);

  public Intake() {

    m_IntakeRoller.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakeRoller_2.getConfigurator().apply(BotConstants.Intake.cfg_Roller);
    m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot);

    m_IntakePivot.setPosition(0);
    //this.setDefaultCommand(doStow());
  }




  public void runIntake(State state) {
    m_IntakeRoller.setControl(intakeVelocityController.withVelocity(state.roller_velocity*-1));
    m_IntakeRoller_2.setControl(intakeVelocityController2.withVelocity(state.roller_velocity*-1*2));
  }

  public void positionIntake(Pivot pivot) {
    m_IntakePivot.setControl(PivotPositionControl.withPosition(pivot.position).withSlot(0)); //
  }

  public Command zeroEncoder() {
    return run(()->{m_IntakePivot.setPosition(0);});
}

public Command doStow() {
  return this.run(() -> {
      m_IntakeRoller.stopMotor();
      m_IntakeRoller_2.stopMotor();
      m_IntakePivot.setControl(pivotPosition.withPosition(Pivot.STOW.position).withSlot(1));     
    });
  }


  public Command doIntake() {
    return this.runOnce(() -> {
        this.positionIntake(Pivot.DEPLOY); 

    }).andThen(new WaitCommand(0.5)).andThen(this.run(()->{
              this.runIntake(State.INTAKE); 
    }));
  }


  public Command doOuttake(){
    return this.run(()->{
      this.runIntake(State.OUTTAKE);
      this.positionIntake(Pivot.DEPLOY);
      Hopper.get().reverseHopper();
    });
  }

public Command doOscilateIntake() {
  return this.run(() -> {
      //m_IntakeRoller.stopMotor();
      //m_IntakeRoller_2.stopMotor();

      double time = Timer.getFPGATimestamp();
      double frequency = 5.0; // Adjust this to change speed (AST changed, was 10.0)
      double oscillator = (.2 * Math.sin(time * frequency)) - 0.04;

      m_IntakePivot.setControl(pivotPosition.withPosition(oscillator).withSlot(2)); 
    });
  }



//Sim stuff, need to set tis up for the rest of the sub system for testing at home and playing around with the values
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

    double motorPos = (m_IntakePivotSim.getAngleRads()) / (2 * Math.PI);
    m_IntakePivot.getSimState().setRawRotorPosition(motorPos);
    
    
    m_IntakePivot.getSimState().setSupplyVoltage(12.0);
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Intake pose", m_IntakePivot.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("Roller speed", m_IntakeRoller.getVelocity().getValueAsDouble());
}
}