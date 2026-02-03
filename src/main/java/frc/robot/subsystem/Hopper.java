// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private static MotionMagicVelocityVoltage HopperController = new MotionMagicVelocityVoltage(0);
  //Motor initalized
  private final TalonFX m_Hopper = new TalonFX(BotConstants.Hopper.HopperID);
  /** Creates a new Hopper. */
  public Hopper() {
    //Motor config
    m_Hopper.getConfigurator().apply(BotConstants.cfg_Roller);

  }

  //Moves the belt that pushes all the balls towrads the shooter
  public Command run_Hopper(){
    return run(()->{m_Hopper.setControl(HopperController.withVelocity(1));});
  }

  //Stops
  public Command Stop(){
    return run(()->{m_Hopper.stopMotor();});
  }



  @Override
  public void periodic() {
  }
}
