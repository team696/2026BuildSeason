// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;



public class Hopper extends SubsystemBase {

    private static MotionMagicVelocityVoltage HopperController = new MotionMagicVelocityVoltage(0);

    private static Hopper hopper = null;


  private static synchronized Hopper get(){
    if(hopper == null){
      hopper = new Hopper();
    }
    return hopper;
  }




  private final TalonFX m_Hopper = new TalonFX(BotConstants.Hopper.HopperID);
  private final DigitalInput m_hopperbeambreak = new DigitalInput(BotConstants.Hopper.HopperBeamBreakID);
  private final CANrange testCanRange = new CANrange(0);
  /** Creates a new Hopper. */
  public Hopper() {
    m_Hopper.getConfigurator().apply(BotConstants.cfg_Roller);

  }

  public Command run_Hopper(){
    return run(()->{m_Hopper.setControl(HopperController.withVelocity(1));});
  }

  @Override
  public void periodic() {
    testCanRange.getDistance().getValue().in(Units.Inches);
    // This method will be called once per scheduler run
  }
}
