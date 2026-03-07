package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Hopper;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.*;

public class Binds {

	public static Optional<Alliance> alliance = DriverStation.getAlliance();
	public static Translation2d hub = Field.Alliance_Find.hub;
	public static Translation2d Pass_1 = Field.Alliance_Find.Pass_1;
	public static Translation2d Pass_2 = Field.Alliance_Find.Pass_2;
			//Standard driving
	private static final SwerveRequest.FieldCentric swerveFCDriveRequest = 
		new SwerveRequest.FieldCentric()
		.withDeadband(BotConstants.DriveConstants.MaxSpeed * 0.05)
		.withRotationalDeadband(BotConstants.DriveConstants.MaxAngularRate * 0.05)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	
			
			
			   
			
public static final class DriverStation2026 {
	static {
		DriverStation.silenceJoystickConnectionWarning(true);
		}
			
			
	public static final void bind() {
					// Map Joysticks
		Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
			()-> swerveFCDriveRequest
				.withVelocityX(HumanControls.DriverPanel.leftJoyY.getAsDouble()*BotConstants.DriveConstants.MaxSpeed)
				.withVelocityY(HumanControls.DriverPanel.leftJoyX.getAsDouble()*BotConstants.DriveConstants.MaxSpeed)
				.withRotationalRate(HumanControls.DriverPanel.rightJoyX.getAsDouble()*BotConstants.DriveConstants.MaxAngularRate))); // Standard driving
			
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
		return 0.0;//HumanControls.SingleXboxController.leftJoyY.getAsDouble() * BotConstants.DriveConstants.MaxSpeed;
	}
	
	private static double getDriveRight() {
		return 0.0;//HumanControls.SingleXboxController.leftJoyX.getAsDouble() * BotConstants.DriveConstants.MaxSpeed;
	}
	private static double getRotationClockwise() {
		return 0.0;//HumanControls.SingleXboxController.rightJoyX.getAsDouble() * BotConstants.DriveConstants.MaxAngularRate;
	}	
			
	public static final void bind() {

		Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
			() -> swerveFCDriveRequest
			.withVelocityX(getDriveForward())
			.withVelocityY(getDriveRight())
			.withRotationalRate(getRotationClockwise()))); //Standard driving
		
		
		//Climber.get().setDefaultCommand(Climber.get().doExtend()); //Default to go up
		//Shooter.get().setDefaultCommand(Shooter.get().idle()); //Shooter rollers idle
		// Intake.get().setDefaultCommand(Intake.get().doStow());
		//Hopper.get().setDefaultCommand(Hopper.get().Stop());

		

	//HumanControls.SingleXboxController.X.whileTrue(new AutoAlign(hub)); //Auto align
	//HumanControls.SingleXboxController.A.whileTrue(Climber.get().doRetract()); //Hold A to go down
	//HumanControls.SingleXboxController.B.whileTrue(Climber.get().doExtend()); //Hold B to go up

	//HumanControls.SingleXboxController.LB.whileTrue(new AutoAlign(Pass_1)); //Auto Align to conrer
	//HumanControls.SingleXboxController.RB.whileTrue(new AutoAlign(Pass_2));//Auto Align to the corner again
	//HumanControls.SingleXboxController.A.whileTrue(Intake.get().doIntake()); //Intake .alongWith(Hopper.get().run_Hopper())
	HumanControls.SingleXboxController.RT.whileTrue(Shooter.get().ShootDash()); //.alongWith(Hopper.get().run_Hopper())
	//HumanControls.SingleXboxController.B.whileTrue(Swerve.get().alignToClimb());
	HumanControls.SingleXboxController.Y.whileTrue(Intake.get().doIntake().alongWith(Hopper.get().run_Hopper())); // on controller the button is X
	HumanControls.SingleXboxController.X.whileTrue(Intake.get().doStow().alongWith(Hopper.get().Stop()));




				
		}
	}
}
