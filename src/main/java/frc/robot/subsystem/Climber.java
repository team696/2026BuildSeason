// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

/*This is for the maybe temperary L1 climb that will be going on the A bot for now
 * 
 */
public class Climber extends SubsystemBase {
 private static Climber Climber = null;

  //Prevents the need of duplicate objects
  public static synchronized Climber get(){
    if(Climber == null){
      Climber = new Climber();
    } 
    return Climber;
  }

  public final TalonFX mClimb = new TalonFX(BotConstants.Climber.Climber_ID); //Public so it can be accessed by the ZeroClimber Command
  private final MotionMagicVoltage climberPosition = new MotionMagicVoltage(0);
  //private final VelocityVoltage climberVelocityVoltage = new VelocityVoltage(0);
  //private final MotionMagicVelocityVoltage climberVelocity = new MotionMagicVelocityVoltage(0);

  // boolean isZeroed;

  private StatusSignal<Current> ClimberAmps;

  

  public Climber(){
    ClimberAmps = mClimb.getStatorCurrent();
    mClimb.getConfigurator().apply(BotConstants.Climber.cfg_Climber);
  // if(!isZeroed){this.zeroEncoder();}
  }

 public Command doRetract() {
  return run(()->{
    // mClimb.setControl(climberVelocity.withVelocity(-20));
    mClimb.setControl(climberPosition.withPosition(75));


  });
}
public Command doExtend(){
  return run(()->{
  mClimb.setControl(climberPosition.withPosition(0));  });
  }

//   public void setVelocity(double rps) {
//     mClimb.setControl(climberVelocity.withVelocity(rps));
// }

public void stop() {
    mClimb.stopMotor();
}

public void zeroEncoder() {
    mClimb.setPosition(0);
    // this.setIsZeroed(true);
}

// public void setIsZeroed(boolean zeroed) {
//     this.isZeroed = zeroed;
// }

public double getCurrentClimber(){
  return ClimberAmps.refresh().getValueAsDouble();
}

  @Override
  public void periodic() {
    double test = getCurrentClimber();
    //System.out.print(test);
    // Monitor climber position and current
   
  }
}
