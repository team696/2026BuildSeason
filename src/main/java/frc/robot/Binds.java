package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.GyroReset;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystem.Hopper;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;

public class Binds {
			//Standard driving
			
	private static final SwerveRequest.FieldCentric swerveFCDriveRequest = 
		new SwerveRequest.FieldCentric()
		.withDeadband(.1)  // was maxspeed *.1
		.withRotationalDeadband(.05)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
	
			
			
			   
			
public static final class DriverStation2026 {
	static {
		DriverStation.silenceJoystickConnectionWarning(true);
		}
	
	static double square(double input) {
		return input * input * Math.signum(input);
	}
			
	public static final void bind() {
					// Map Joysticks
		Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
			()-> swerveFCDriveRequest
				.withVelocityX(square(HumanControls.DriverPanel.leftJoyY.getAsDouble())*BotConstants.DriveConstants.MaxSpeed)
				.withVelocityY(square(HumanControls.DriverPanel.leftJoyX.getAsDouble())*BotConstants.DriveConstants.MaxSpeed)
				.withRotationalRate(square(-HumanControls.DriverPanel.rightJoyX.getAsDouble())*BotConstants.DriveConstants.MaxAngularRate))) ; // Standard driving
			
		HumanControls.DriverPanel.OtherButton.whileTrue(new AutoAlign(()->Field.Alliance_Find.hub).alongWith(Shooter.get().spinUpCommand()));
			
		}
	}

public static final class OperatorPanel{
	static{
		DriverStation.silenceJoystickConnectionWarning(true);
	}
	public static final void bind(){
		Hopper.get().setDefaultCommand(Hopper.get().Stop());
		Shooter.get().setDefaultCommand(Shooter.get().idle()); //Shooter rollers idle
		Intake.get().setDefaultCommand(Intake.get().doStow());

		HumanControls.OperatorPanel.SouceCoral.whileTrue(Intake.get().doIntake());
		HumanControls.OperatorPanel.GroundCoral.whileTrue(new ShootCommand(()->Field.Alliance_Find.hub).alongWith(Intake.get().doOscilateIntake()));
		HumanControls.OperatorPanel.gyro.onTrue(new GyroReset(Swerve.get()));
		HumanControls.OperatorPanel.releaseCoral.whileTrue(Shooter.get().ShootPass().alongWith(Intake.get().doOscilateIntake()));
		HumanControls.OperatorPanel.pickupAlgae.whileTrue(Intake.get().doOuttake());
		HumanControls.OperatorPanel.L3.whileTrue(Swerve.get().xMode());
		//HumanControls.OperatorPanel.deepOrSwitch.onTrue(new InstantCommand(()->Intake.get().setDefaultCommand(Intake.get().doDefense())));
		//HumanControls.OperatorPanel.deepOrSwitch.onFalse(new InstantCommand(()->Intake.get().setDefaultCommand(Intake.get().doStow())));

		//HumanControls.OperatorPanel.Climb1.whileTrue(new AutoAlignToShoot(hub));
		//HumanControls.OperatorPanel.Climb1.onTrue(new PathFindToClimb(Field.before_Tower_Blue).andThen(new PathFindToClimb()));


		
	}

}
			
			
			
public static final class Controller {
	static {
		DriverStation.silenceJoystickConnectionWarning(true);
		}
	
	//Xbox controller methods, simplifies and cleans up the bind() method a lot
	private static double getDriveForward() {
		return HumanControls.SingleXboxController.leftJoyY.getAsDouble() * BotConstants.DriveConstants.MaxSpeed;
	}
	
	private static double getDriveRight() {
		return HumanControls.SingleXboxController.leftJoyX.getAsDouble() * BotConstants.DriveConstants.MaxSpeed;
	}
	private static double getRotationClockwise() {
		return HumanControls.SingleXboxController.rightJoyX.getAsDouble() * BotConstants.DriveConstants.MaxAngularRate;
	}	
			
	public static final void bind() {

		Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
			() -> swerveFCDriveRequest
			.withVelocityX(getDriveForward())
			.withVelocityY(getDriveRight())
			.withRotationalRate(getRotationClockwise()))); //Standard driving
		
		
		//Climber.get().setDefaultCommand(Climber.get().doExtend()); //Default to go up
		//Shooter.get().setDefaultCommand(Shooter.get().idle()); //Shooter rollers idle
		//Intake.get().setDefaultCommand(Intake.get().doStow());
		//Hopper.get().setDefaultCommand(Hopper.get().Stop());

		

	HumanControls.SingleXboxController.A.whileTrue(Shooter.get().spinUpCommand());
	//HumanControls.SingleXboxController.B.whileTrue(new ShootCommand(()->Field.Alliance_Find.hub));
	HumanControls.SingleXboxController.Y.whileTrue(Swerve.get().xMode());
	 



				
		}
	}
}