package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.subsystem.Intake.Pivot;
import frc.robot.util.*;

public class Binds {

	public static Optional<Alliance> alliance = DriverStation.getAlliance();
	public static Translation2d hub = Field.Alliance_Find.hub;
	public static Translation2d Pass_1 = Field.Alliance_Find.Pass_1;
	public static Translation2d Pass_2 = Field.Alliance_Find.Pass_2;
			//Standard driving
			private static final SwerveRequest.FieldCentric swerveFCDriveRequest = 
			new SwerveRequest.FieldCentric()
			.withDeadband(TunerConstants.MaxSpeed * 0.05)
			.withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	
			
			
			   
			
				public static final class DriverStation2026 {
				static {
					DriverStation.silenceJoystickConnectionWarning(true);
				}
			
			
				public static final void bind() {
					// Map Joysticks
					Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
							() -> swerveFCDriveRequest.withVelocityX(Math.pow(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 2))
								.withVelocityY(Math.pow(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 2))
								.withRotationalRate(Math.pow(HumanControls.DriverPanel.rightJoyX.getAsDouble(), 2))));
			
						// Reset Gyro
					HumanControls.DriverPanel.resetGyro.onTrue(new InstantCommand(() -> Swerve.get().seedFieldCentric()));
				}
			}
			
			
			
			public static final class Controller {
				static {
					DriverStation.silenceJoystickConnectionWarning(true);
				}
	
				//Xbox controller methods, simplifies and cleans up the bind() method a lot
				private static double getDriveForward() {
					return Math.pow(HumanControls.SingleXboxController.leftJoyY.getAsDouble(), 2) * Math.signum(HumanControls.SingleXboxController.leftJoyY.getAsDouble());
				}
	
				private static double getDriveRight() {
					return Math.pow(HumanControls.SingleXboxController.leftJoyX.getAsDouble(), 2) * Math.signum(HumanControls.SingleXboxController.leftJoyX.getAsDouble());
				}
				private static double getRotationClockwise() {
					return Math.pow(HumanControls.SingleXboxController.rightJoyX.getAsDouble(), 2) * Math.signum(HumanControls.SingleXboxController.rightJoyX.getAsDouble());
				}	
			
				public static final void bind() {
					Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
							() -> swerveFCDriveRequest
								.withVelocityX(getDriveForward())
								.withVelocityY(getDriveRight())
								.withRotationalRate(getRotationClockwise()))); //Standard driving

				HumanControls.SingleXboxController.X.whileTrue(new AutoAlign(hub)); //Auto align and rev
				HumanControls.SingleXboxController.Y.and(HumanControls.SingleXboxController.LB).whileTrue(new AutoAlign(Pass_1)); //Auto Align to conrer
				HumanControls.SingleXboxController.Y.and(HumanControls.SingleXboxController.RB).whileTrue(new AutoAlign(Pass_2));//Auto Align to the corner again
				HumanControls.SingleXboxController.LT.whileTrue(Intake.get().setState(Intake.State.INTAKE, Intake.Pivot.DEPLOY)); //Intake
				HumanControls.SingleXboxController.RT.whileTrue(Shooter.get().intake_shooter()); //Feed the balls from the hopper into the intake
				
		}

	}
}
