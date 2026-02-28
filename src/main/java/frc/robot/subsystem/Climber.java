// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

  private final TalonFX mClimb = new TalonFX(BotConstants.Climber.Climber_1_ID);
  private final MotionMagicVoltage climberPosition = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage climberVelocity = new MotionMagicVelocityVoltage(0);

  boolean isZeroed;

  public Climber(){
  mClimb.getConfigurator().apply(BotConstants.Climber.cfg_Climber1);
  this.setDefaultCommand(doRetract());
  }

 public Command doRetract() {
  if (isZeroed) {
    return run(() -> {
      mClimb.setControl(climberPosition.withPosition(0));
    });
  } else {
    return run(() -> {
      // Do nothing if not zeroed
    });
  }
}
  public Command doExtend(){
  return run(()->{
  mClimb.setControl(climberPosition.withPosition(.2)); // .2 chosen randomly
  });
  }


  public void setVelocity(double rps) {
    mClimb.setControl(climberVelocity.withVelocity(rps));
}

public void stop() {
    mClimb.setControl(climberVelocity.withVelocity(0));
}

public double getStatorCurrent() {
    return mClimb.getStatorCurrent().getValueAsDouble();
}

public void zeroEncoder() {
    mClimb.setPosition(0);
}

public void setIsZeroed(boolean zeroed) {
    this.isZeroed = zeroed;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
