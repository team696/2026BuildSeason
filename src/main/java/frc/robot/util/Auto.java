package frc.robot.util;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Swerve;

public class Auto {
	public static boolean initialized = false;

	public static SendableChooser<Command> autoChooser;

	public static void initialize(NamedCommand... Commands) {
		if (initialized == true) {
			return;
		}

		for (NamedCommand command : Commands) 
		  command.register();

		if (configureAutoBuilder()) {
			autoChooser = AutoBuilder.buildAutoChooser();
			SmartDashboard.putData("Auto Chooser", autoChooser);
		}

		initialized = true;
	}

	public static Command getSelectedAuto() {
		if (!initialized)
			return null;

		return autoChooser.getSelected();
	}

	public static final class NamedCommand {
		public String name;
		public Command command;

		/**
		 * @param name    to be used inside of the pathplanner GUI
		 * @param command to be run when called inside pathplanner
		 */
		public NamedCommand(String name, Command command) {
			this.name = name;
			this.command = command;
		}

		public void register() {
			NamedCommands.registerCommand(name, command);
		}
	}

	private static boolean configureAutoBuilder() {
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
					() -> Swerve.get().getState().Pose, // Supplier of current robot pose
					Swerve.get()::resetPose, // Consumer for seeding pose against auto
					() -> Swerve.get().getState().Speeds, // Supplier of current robot speeds
					// Consumer of ChassisSpeeds and feedforwards to drive the robot
					(speeds, feedforwards) -> Swerve.get().setControl(
							m_pathApplyRobotSpeeds.withSpeeds(speeds)
									.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
									.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
					new PPHolonomicDriveController(
							new PIDConstants(10, 0, 0),
							new PIDConstants(7, 0, 0)),
					config,
					// Assume the path needs to be flipped for Red vs Blue, this is normally the
					// case
					() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
					Swerve.get() // Subsystem for requirements
			);
		} catch (Exception ex) {
			DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
			return false;
		}

		return true;
	}

	private static final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
}