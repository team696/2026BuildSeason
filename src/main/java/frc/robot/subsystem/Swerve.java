//Generously given to us by Oscar, eliminates lots of the BS CTRE added into their swerve code. 

package frc.robot.subsystem;

import java.io.Console;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.TunerConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.BaseCam.AprilTagResult;
import frc.robot.util.Field;
import frc.robot.util.LimeLightCam;

public final class Swerve extends TunerSwerveDrivetrain implements Subsystem, Sendable {
	private static Swerve m_Swerve;
	private static LimeLightCam leftCam = new LimeLightCam("limelight-left");
    private static LimeLightCam rightCam = new LimeLightCam("limelight-right");
	private static LimeLightCam frontCam = new LimeLightCam("limelight-front");

	Rotation2d yawoffset = new Rotation2d(0);

	public static synchronized Swerve get() {
		if (m_Swerve == null)
			m_Swerve = TunerConstants.createDrivetrain();
		return m_Swerve;
	}

	public Swerve(
		SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
		super(drivetrainConstants, 0, modules);
		if (Utils.isSimulation()) {
			simulationInit();
		}
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()){
			updateYawoffset();
		}

		  // This runs 50 times per second
        leftCam.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);    
        rightCam.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);
		frontCam.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);
		leftCam.SetRobotOrientation(yawoffset);
		rightCam.SetRobotOrientation(yawoffset);
		frontCam.SetRobotOrientation(yawoffset);
	}

	
    boolean acceptEstimate(AprilTagResult latestResult) {
      if(latestResult.distToTag > 3){
        return false;
      }
	  //System.out.println(latestResult.pose.getX()+","+latestResult.pose.getY());
	  setVisionMeasurementStdDevs(VecBuilder.fill(0.001, 0.001, 0.001)); // trust the tag a lot, change and scale this in the future 
      return true;
    }

	public void updateYawoffset(){
		yawoffset = getPose().getRotation().minus(getPigeon2().getRotation2d());
	}


	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Value", () -> 1, null);
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}


	@Override
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
	}

	@Override
	public void addVisionMeasurement(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
				visionMeasurementStdDevs);
	}

	public Rotation2d target_theta_hub(){
      return new Rotation2d(Math.atan2(
        Field.hub_position_blue.getY() - Swerve.get().getPose().getY(),
        Field.hub_position_blue.getX() - Swerve.get().getPose().getX()
        )).minus(Swerve.get().getPose().getRotation());
    }

	public Rotation2d target_theta(Translation2d desiredpose){
		return new Rotation2d(Math.atan2(
			desiredpose.getY() - Swerve.get().getPose().getY(),
			desiredpose.getX() - Swerve.get().getPose().getX()
		)).minus(Swerve.get().getPose().getRotation());
	}

	

	public Pose2d getPose(){
		return this.getState().Pose;
	}

	

	double m_lastSimTime;
	Notifier simUpdate;

	private void simulationInit() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();
		simUpdate = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		simUpdate.setName("Swerve Simulation Update");
		simUpdate.startPeriodic(0.005);
	}



		// Since we are using a holonomic drivetrain, the rotation component of this pose
	// represents the goal holonomic rotation
	Pose2d targetPose = Field.Alliance_Find.climb_tower;

	// Create the constraints to use while pathfinding
	PathConstraints constraints = new PathConstraints(
        2.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

	public Command alignToClimb(){
	// Since AutoBuilder is configured, we can use it to build pathfinding commands
	return AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0); // Goal end velocity in meters/sec
	}

}