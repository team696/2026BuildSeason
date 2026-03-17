// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Swerve;
import frc.robot.util.Field;

public class PathFindToClimb extends Command {

    // --- Tunable constants ---
    private static final double MAX_VELOCITY_MPS        = 0.5;
    private static final double MAX_ACCEL_MPS2          = 0.7;
    private static final double MAX_ANGULAR_VEL_RAD     = Units.degreesToRadians(5);
    private static final double MAX_ANGULAR_ACCEL_RAD   = Units.degreesToRadians(10);

    private final PathConstraints constraints = new PathConstraints(
            MAX_VELOCITY_MPS,
            MAX_ACCEL_MPS2,
            MAX_ANGULAR_VEL_RAD,
            MAX_ANGULAR_ACCEL_RAD);

    private final Pose2d targetPose;

    // The inner PathPlanner command we delegate to
    private Command pathfindCommand;

    /**
     * Pathfinds to the climb tower using PathPlanner. Ends early (via
     * isFinished) once PathPlanner's onTarget() signal fires, at which
     * point you should chain AutoAlignToGoalPose for final alignment.
     *
     * @param targetPose The goal pose to pathfind toward.
     */
    public PathFindToClimb(Pose2d targetPose) {
        this.targetPose = targetPose;
        addRequirements(Swerve.get());
    }

    /** Convenience constructor — uses the default climb tower pose from Field. */
    public PathFindToClimb() {
        this(Field.Alliance_Find.climb_tower);
    }

    @Override
    public void initialize() {
        // Build the pathfind command fresh each schedule so the path
        // is generated from the robot's current position, not a stale one.
        pathfindCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);

        pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        pathfindCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // Always cancel the inner command so PathPlanner cleans up its
        // trajectory follower regardless of why we stopped.
        pathfindCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // Let PathPlanner decide when we're on target, OR stop if the
        // inner command finished on its own (e.g. reached the full pose).
        return pathfindCommand.isFinished();
    }
}