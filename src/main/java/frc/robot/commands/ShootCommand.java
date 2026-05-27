// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Hopper;
import frc.robot.subsystem.LED;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;
import frc.robot.util.ShooterControlPolicy;

public class ShootCommand extends Command {

  private final Supplier<Translation2d> desiredPose;
  /**
   * Hysteresis latch state. Recomputed per execute() loop by ShooterControlPolicy.
   * Reset to false in initialize() so each command instance starts from cold.
   */
  private boolean isFiring;

  public ShootCommand(Supplier<Translation2d> desired_pose) {
    this.desiredPose = desired_pose;
    addRequirements(Shooter.get(), Hopper.get(), LED.get());
  }

  @Override
  public void initialize() {
    isFiring = false;
  }

  @Override
  public void execute() {
    double distMeters = Swerve.get().distTo(desiredPose.get());
    ShooterControlPolicy.Decision decision = ShooterControlPolicy.compute(
        distMeters,
        Shooter.get().getRollerVelocity(),
        BotConstants.Shooter.ShooterTable,
        BotConstants.Shooter.backSpinTable,
        ShooterControlPolicy.DEFAULT_ENTER_TOL,
        ShooterControlPolicy.DEFAULT_EXIT_TOL,
        isFiring);

    Shooter.get().set_velocity(decision.flywheelTarget());
    LED.get().LEDyellowBlink();

    isFiring = decision.fireIndexer();
    if (isFiring) {
      Shooter.get().intake_shooter(decision.indexerSpeed());
      Hopper.get().run_Hopper();
    }
  }

  @Override
  public void end(boolean interrupted) {
    Hopper.get().stop();
    Shooter.get().Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
