package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;

public class Binds {
		private static final SwerveRequest.FieldCentric swerveFCDriveRequest = new SwerveRequest.FieldCentric()
			.withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
		
		public final static SwerveRequest.FieldCentricFacingAngle FCFARequest = 
					new SwerveRequest.FieldCentricFacingAngle()
					.withDeadband(BotConstants.DriveConstants.MaxSpeed* 0.1)
					.withRotationalDeadband(BotConstants.DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withHeadingPID(3, 0, 0); 
		
		   
		
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
									.withRotationalRate(getRotationClockwise())));
					
					HumanControls.SingleXboxController.B.whileTrue(Swerve.get().applyRequest(() ->
						FCFARequest
							.withVelocityX(getDriveForward()) // Drive forward with negative Y (forward)
                    		.withVelocityY(getDriveRight()) // Drive left with negative X (left)
                    		.withTargetDirection(Swerve.get().target_theta())
            )
        );
		}

	}
}