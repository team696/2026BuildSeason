// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;



public class Hopper extends SubsystemBase {

  
  public static Hopper hopper = null;


  public static synchronized Hopper get(){
    if(hopper == null){
      hopper = new Hopper();
    }
    return hopper;
  }

  //Motor controller
  private static DutyCycleOut HopperController = new DutyCycleOut(0.6);
    private static DutyCycleOut reverseCylce = new DutyCycleOut(-0.6);

  //Motor initalized
  private final TalonFX m_Hopper = new TalonFX(BotConstants.Hopper.HopperID);
  


  /** Creates a new Hopper. */
  public Hopper() {
    //Motor config
    m_Hopper.getConfigurator().apply(BotConstants.Hopper.cfg_Hopper);

    SmartDashboard.putNumber("Hopper speed", 0);

  }

  //Moves the belt that pushes all the balls towrads the shooter
  public void run_Hopper(){
    m_Hopper.setControl(HopperController);
  }

  public void reverseHopper(){
    m_Hopper.setControl(reverseCylce);
  }

  public Command run_Hopper_Command(){
    return runEnd(
      ()->{m_Hopper.setControl(HopperController);},
      ()->{m_Hopper.stopMotor();});
  }

 public Command run_Hopper_Reverse(){
    return runEnd(
      ()->{m_Hopper.setControl(reverseCylce);},
      ()->{m_Hopper.stopMotor();});
  }

  public Command oscilateHopper(){
    return runEnd(
      ()->{m_Hopper.setControl(new DutyCycleOut((0.3*Math.sin(Timer.getFPGATimestamp()/4)+0.4)));},
      ()->{m_Hopper.stopMotor();});
  }


  //Stops
  public Command Stop(){
    return run(()->{m_Hopper.stopMotor();});
  }



  @Override
  public void periodic() {
  }
}
