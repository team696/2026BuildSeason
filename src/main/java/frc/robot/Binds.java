package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.AutoAlign;
//import frc.robot.commands.DefenseBoost;
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
			
		HumanControls.DriverPanel.OtherButton.whileTrue(new AutoAlign(()->Field.Alliance_Find.hub));
			
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
		HumanControls.OperatorPanel.L1.whileTrue(Intake.get().doOscilateIntake());
		HumanControls.OperatorPanel.L2.whileTrue(Swerve.get().xMode());
		HumanControls.OperatorPanel.L3.whileTrue(new ShootCommand(()->Field.Alliance_Find.hub));
		//HumanControls.OperatorPanel.L4.whileTrue(new DefenseBoost(120, 50));
		HumanControls.OperatorPanel.Processor.whileTrue(Shooter.get().spinUpCommand()); 


		
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

		Hopper.get().setDefaultCommand(Hopper.get().Stop());
		Shooter.get().setDefaultCommand(Shooter.get().idle()); //Shooter rollers idle
		Intake.get().setDefaultCommand(Intake.get().doStow());
		
		
	
		

		HumanControls.SingleXboxController.LT.whileTrue(Intake.get().doIntake());
		HumanControls.SingleXboxController.RT.whileTrue(new ShootCommand(()->Field.Alliance_Find.hub).alongWith(Intake.get().doOscilateIntake()));
		//HumanControls.OperatorPanel.gyro.onTrue(new GyroReset(Swerve.get()));
		HumanControls.SingleXboxController.Y.whileTrue(Shooter.get().ShootPass().alongWith(Intake.get().doOscilateIntake()));
		HumanControls.SingleXboxController.X.whileTrue(Intake.get().doOuttake());
		HumanControls.SingleXboxController.A.whileTrue(Intake.get().doOscilateIntake());
		HumanControls.SingleXboxController.B.whileTrue(Swerve.get().xMode());
		//HumanControls.SingleXboxController.RT.whileTrue(new ShootCommand(()->Field.Alliance_Find.hub));
		HumanControls.SingleXboxController.RB.whileTrue(new AutoAlign(()->Field.Alliance_Find.hub));
		HumanControls.SingleXboxController.LB.whileTrue(Shooter.get().spinUpCommand()); 




				
		}
	}
}