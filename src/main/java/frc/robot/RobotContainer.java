// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.HumanControls.DriverPannel;
import frc.robot.HumanControls.SingleXboxController;
import frc.robot.subsystem.Swerve;

public class RobotContainer {

    //Robot = good;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final HumanControls.DriverPannel joystick = new HumanControls.DriverPannel();


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private boolean isDriverPanelConnected() {
    return DriverPannel.DriverPanel.isConnected();
}

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> {
        if (isDriverPanelConnected()) {
            // Driver Panel controls
            return drive.withVelocityX(-joystick.leftJoyY.getAsDouble() * MaxSpeed)
                       .withVelocityY(-joystick.leftJoyX.getAsDouble() * MaxSpeed)
                       .(joystick.rightJoyX.getAsDouble() * MaxAngularRate);
        } else {
            // Xbox controls (backup)
            return drive.withVelocityX(SingleXboxController.leftJoyY.getAsDouble() * MaxSpeed)
                       .withVelocityY(SingleXboxController.leftJoyX.getAsDouble() * MaxSpeed)
                       .withRotationalRate(SingleXboxController.rightJoyX.getAsDouble() * MaxAngularRate);
        }
    })
    );

         
    SingleXboxController.A.whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-SingleXboxController.leftJoyY.getAsDouble() * MaxSpeed)
             .withVelocityY(SingleXboxController.leftJoyX.getAsDouble() * MaxSpeed) // I think you negate this one
             .withRotationalRate(SingleXboxController.leftJoyY.getAsDouble() * MaxAngularRate))
    );
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
