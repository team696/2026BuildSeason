package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlignToClimb;
import frc.robot.commands.AutoAlignToShoot;
import frc.robot.commands.GyroReset;
import frc.robot.commands.ZeroClimber;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Hopper;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;

public class Binds {

	public static Optional<Alliance> alliance = DriverStation.getAlliance();
	public static Translation2d hub = Field.Alliance_Find.hub;
	public static Translation2d Pass_1 = Field.Alliance_Find.Pass_1;
	public static Translation2d Pass_2 = Field.Alliance_Find.Pass_2;
	public static Pose2d climb = Field.Alliance_Find.climb_tower;
			//Standard driving
			
	private static final SwerveRequest.FieldCentric swerveFCDriveRequest = 
		new SwerveRequest.FieldCentric()
		.withDeadband(BotConstants.DriveConstants.MaxSpeed * 0.08)  // was maxspeed *.1
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
				.withRotationalRate(-HumanControls.DriverPanel.rightJoyX.getAsDouble()*BotConstants.DriveConstants.MaxAngularRate))) ; // Standard driving
			
		HumanControls.DriverPanel.OtherButton.whileTrue(new AutoAlign(hub));
			
		}
	}

public static final class OperatorPanel{
	static{
		DriverStation.silenceJoystickConnectionWarning(true);
	}
	public static final void bind(){
		Shooter.get().setDefaultCommand(Shooter.get().idle()); //Shooter rollers idle
		Intake.get().setDefaultCommand(Intake.get().doStow());

		HumanControls.OperatorPanel.SouceCoral.whileTrue(Intake.get().doIntake().alongWith(Hopper.get().testHopper()));
		//HumanControls.OperatorPanel.Barge.whileTrue(Intake.get().doStow());
		HumanControls.OperatorPanel.GroundCoral.whileTrue(Shooter.get().Shoot(hub));
		HumanControls.OperatorPanel.gyro.onTrue(new GyroReset(Swerve.get()));
		HumanControls.OperatorPanel.releaseCoral.whileTrue(new AutoAlign(Pass_1));
		HumanControls.OperatorPanel.pickupAlgae.whileTrue(new AutoAlign(Pass_2));
		HumanControls.OperatorPanel.L3.whileTrue(new ZeroClimber());
		HumanControls.OperatorPanel.L1.whileTrue(Climber.get().doExtend());
		HumanControls.OperatorPanel.L2.whileTrue(Climber.get().doRetract());
		// HumanControls.OperatorPanel.Climb1.whileTrue(new AutoAlignToShoot(hub).andThen(Shooter.get().Shoot(hub)));
		// HumanControls.OperatorPanel.Climb1.whileTrue(new AutoAlignToClimb(climb));
		HumanControls.OperatorPanel.Climb1.whileTrue(Swerve.get().alignToClimb());


		// L4 button seems to be flakey, changing to processor button
		HumanControls.OperatorPanel.Processor.and(HumanControls.OperatorPanel.deepOrSwitch).whileTrue(Swerve.get().alignToClimb());
		HumanControls.OperatorPanel.Processor.whileTrue(new ConditionalCommand(Swerve.get().alignToClimb(), Climber.get().gotToZero(), HumanControls.OperatorPanel.deepOrSwitch));
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
		Shooter.get().setDefaultCommand(Shooter.get().idle()); //Shooter rollers idle
		Intake.get().setDefaultCommand(Intake.get().doStow());
		//Hopper.get().setDefaultCommand(Hopper.get().Stop());

		

	HumanControls.SingleXboxController.X.whileTrue(new AutoAlign(hub)); //Auto align
	//HumanControls.SingleXboxController.A.whileTrue(Climber.get().doRetract()); //Hold A to go down
	//HumanControls.SingleXboxController.B.whileTrue(Climber.get().doExtend()); //Hold B to go up
	//HumanControls.SingleXboxController.A.onTrue(new ZeroClimber());

	//HumanControls.SingleXboxController.LB.whileTrue(new AutoAlign(Pass_1)); //Auto Align to conrer
	//HumanControls.SingleXboxController.RB.whileTrue(new AutoAlign(Pass_2));//Auto Align to the corner again
	HumanControls.SingleXboxController.LB.whileTrue(Intake.get().doIntake().alongWith(Hopper.get().run_Hopper(50))); //Intake 
	HumanControls.SingleXboxController.RB.whileTrue(Shooter.get().Shoot(hub)); 
	//HumanControls.SingleXboxController.B.whileTrue(Swerve.get().alignToClimb());
	//HumanControls.SingleXboxController.X.whileTrue(Intake.get().doStow());
	//HumanControls.SingleXboxController..whileTrue(Intake.get().zeroEncoder());
	//HumanControls.SingleXboxController.B.whileTrue(Hopper.get().run_Hopper());

	



				
		}
	}
}
