// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Hopper;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  Translation2d desiredPose;
  boolean isAtspeed;
  /** Creates a new ShootCommand. */
  public ShootCommand(Translation2d desired_pose) {

    desiredPose = desired_pose;


    this.addRequirements(Shooter.get(),Hopper.get());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      isAtspeed = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double fudgeFactor = SmartDashboard.getNumber("Fudge Factor for Shooter", 0);
      double distMeters=Swerve.get().distTo(desiredPose);
      double velocity = BotConstants.Shooter.ShooterTable.get(distMeters) - fudgeFactor;
      double intakespeed = BotConstants.Shooter.backSpinTable.get(distMeters);

    Shooter.get().set_velocity(velocity);

    if((Math.abs(Shooter.get().getRollerVelocity()-velocity))<1){
        isAtspeed = true;
    }

    if(isAtspeed){
      Shooter.get().intake_shooter(intakespeed);
      Hopper.get().run_Hopper();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Hopper.get().Stop();
    Shooter.get().Stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
