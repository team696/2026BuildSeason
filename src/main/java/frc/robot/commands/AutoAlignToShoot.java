// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToShoot extends Command {
  /** Creates a new AutoAlign. */
  private final static SwerveRequest.FieldCentricFacingAngle FCFARequest = 
					new SwerveRequest.FieldCentricFacingAngle()
					.withDeadband(BotConstants.DriveConstants.MaxSpeed* 0.1)
					.withRotationalDeadband(BotConstants.DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withHeadingPID(3, 0, 0); 

  private Translation2d targetPosition;

  private double semiCircleSetDistance = Units.inchesToMeters(91.055); //in meters
	private PIDController moveToController = new PIDController(1.5, 0, 0);

  public AutoAlignToShoot(Translation2d targetPosition) {
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
    double distance = Swerve.get().distTo(targetPosition);
    Translation2d dir = new Translation2d(-1, Swerve.get().target_theta(targetPosition))
                        .times(moveToController.calculate(distance, this.semiCircleSetDistance));

   Swerve.get().setControl(
        FCFARequest
        .withVelocityX(dir.getX())
        .withVelocityY(dir.getY())
        .withTargetDirection(Swerve.get().target_theta(targetPosition).
        minus(new Rotation2d(Units.degreesToRadians(.87)))) // need an offset of .87 degrees
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
     if (Math.abs(Swerve.get().distTo(targetPosition) - this.semiCircleSetDistance) <.01)
      return true;
    return false;
  }
}
//up date