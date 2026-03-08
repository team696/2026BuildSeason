// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Climber;
import frc.robot.util.BotConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroClimber extends Command {
  /** Creates a new ZeroClimber. */

  private final DutyCycleOut ZeroingControl = new DutyCycleOut(0.35);
  public double current;

  public ZeroClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //BotConstants.Climber.cfg_Climber.CurrentLimits.StatorCurrentLimit = 60;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     current = Climber.get().mClimb.getStatorCurrent().getValueAsDouble();
    SmartDashboard.putNumber("Climber Current", current);

    Climber.get().mClimb.setControl(ZeroingControl);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climber.get().mClimb.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(current >= 4){
      Climber.get().zeroEncoder();
      return true;
    }
    return false;
  }
}
