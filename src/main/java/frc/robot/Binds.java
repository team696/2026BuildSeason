package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Swerve;

public class Binds {
   

	public static final class DriverStation2026 {
		static {
			DriverStation.silenceJoystickConnectionWarning(true);
		}

		private static final SwerveRequest.FieldCentric swerveFCDriveRequest = new SwerveRequest.FieldCentric()
				.withDeadband(TunerConstants.MaxSpeed * 0.05).withRotationalDeadband(TunerConstants.MaxAngularRate * 0.05)
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

		public static final void bind() {

		}
	}
}