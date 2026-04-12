// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HumanControls;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private final static SwerveRequest.FieldCentricFacingAngle FCFARequest = 
					new SwerveRequest.FieldCentricFacingAngle()
					.withDeadband(BotConstants.DriveConstants.MaxSpeed* 0.1)
					.withRotationalDeadband(BotConstants.DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withHeadingPID(8, 0, 0); 

  private Supplier<Translation2d> targetPosition;


  public AutoAlign(Supplier<Translation2d> targetPosition) {
      this.targetPosition = targetPosition;
      addRequirements(Swerve.get());

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        //Drives the swerve using the FCFA request
   Swerve.get().setControl(
        FCFARequest
        .withVelocityX((HumanControls.DriverPanel.leftJoyY.getAsDouble()/2)*BotConstants.DriveConstants.MaxSpeed)
        .withVelocityY((HumanControls.DriverPanel.leftJoyX.getAsDouble()/2)*BotConstants.DriveConstants.MaxSpeed)
        .withTargetDirection(Swerve.get().target_theta(targetPosition.get()))
        .withMaxAbsRotationalRate(DegreesPerSecond.of(360))
        .withRotationalDeadband(DegreesPerSecond.of(1)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
