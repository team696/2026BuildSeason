//Generously given to us by Oscar, eliminates lots of the BS CTRE added into their swerve code. 

package frc.robot.subsystem;

import java.util.function.Supplier;

import javax.sound.midi.SysexMessage;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.TunerConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.BaseCam.AprilTagResult;
import frc.robot.util.Field;
import frc.robot.util.LimeLightCam;
import frc.robot.util.LimelightHelpers;

public final class Swerve extends TunerSwerveDrivetrain implements Subsystem, Sendable {
	private static Swerve m_Swerve;
	private LimeLightCam frontCamera=new LimeLightCam("limelight-front");
	//private LimeLightCam driverCamera = new LimeLightCam("limelight-divePOV", new int[]{31, 32},true);

	//private LimeLightCam leftCamera=new LimeLightCam("limelight-left");
	private LimeLightCam backCamera=new LimeLightCam("limelight-back");


	double semiCircleSetDistance = 1.0; //in meters
	PIDController moveToController = new PIDController(0, 0, 0);

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
		SmartDashboard.putBoolean("Accepted", false);
	}

	@Override
	public void periodic() {
		  // This runs 50 times per second
		  SmartDashboard.putNumber("Distance to hub", this.distTo(Field.Alliance_Find.hub));
        frontCamera.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate); 
		//this.cameraLocalization();
		//leftCamera.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);    
		backCamera.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);    
		frontCamera.SetRobotOrientation(getPose().getRotation());
	}


	void cameraLocalization(){
		frontCamera.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate); 
		if (DriverStation.isTeleop() && CommandScheduler.getInstance().isScheduled(Swerve.get().alignToClimb())) {
    		//driverCamera.addVisionEstimate(this::addVisionMeasurement, this::acceptEstimate);
 			} 
	}
	
    boolean acceptEstimate(AprilTagResult latestResult) {
        if (latestResult.distToTag > 3.5)
        return false;
		SmartDashboard.putBoolean("Accepted", false);
      if (latestResult.ambiguity > 0.6)
        return false; // Too Ambiguous, Ignore
		SmartDashboard.putBoolean("Accepted", false);
      if (getState().Speeds.omegaRadiansPerSecond > 2.5)
        return false; // Rotating too fast, ignore
		SmartDashboard.putBoolean("Accepted", false);
      if (latestResult.distToTag < 1) {
        setVisionMeasurementStdDevs(VecBuilder.fill(2.0, 2.0, 50.0));
		SmartDashboard.putBoolean("Accepted", true);
      } else {
        setVisionMeasurementStdDevs(
            VecBuilder.fill(latestResult.ambiguity * Math.pow(latestResult.distToTag, 2)*3.0,
                latestResult.ambiguity * Math.pow(latestResult.distToTag, 2)*3.0,
                latestResult.ambiguity * Math.pow(latestResult.distToTag, 2)*3.0));
		SmartDashboard.putBoolean("Accepted", true);
      }
      return true;
    }


	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Value", () -> 1, null);
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

void pidToDistance(){
	moveToController.calculate(kNumConfigAttempts);
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
		super.addVisionMeasurement(visionRobotPoseMeters,timestampSeconds,
				visionMeasurementStdDevs);
	}

	public Rotation2d target_theta_hub(){
      return new Rotation2d(Math.atan2(
        Field.hub_position_blue.getY() - Swerve.get().getPose().getY(),
        Field.hub_position_blue.getX() - Swerve.get().getPose().getX()
	  	));
    }

	public Rotation2d target_theta(Translation2d desiredpose){
		return new Rotation2d(Math.atan2(
			desiredpose.getY() - Swerve.get().getPose().getY(),
			desiredpose.getX() - Swerve.get().getPose().getX()
		));
	}

	

	public Pose2d getPose(){
		return this.getState().Pose;
	}

	  /**
   * 
   * @param position Position for distance from
   * @return distance from position argument
   */
  public double distTo(Translation2d position) {
    return getPose().getTranslation().getDistance(position);
  }

  /**
   * 
   * @param position Position for distance from
   * @return distance from position argument
   */
  public double distTo(Pose2d position) {
    return distTo(position.getTranslation());
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

	// Only for AutoAlign to climb command: Create the constraints to use while pathfinding
	PathConstraints constraints = new PathConstraints(
        0.5, 0.7,
        Units.degreesToRadians(5), Units.degreesToRadians(10));
	public Translation2d getVecToPose(Pose2d target){
		Translation2d dist = Swerve.get().getPose().minus(targetPose).getTranslation(); 
		return dist;
	}
	public Command alignToClimb(){
	// Since AutoBuilder is configured, we can use it to build pathfinding commands
	//SwerveRequest.FieldCentric fs = new SwerveRequest.FieldCentric();
	//final double p = 3;
	return AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
         0.0);
		//.andThen(Swerve.get().applyRequest(()->fs.withVelocityX(p*getVecToPose(targetPose).getX()).withVelocityY(p*getVecToPose(targetPose).getY()))); // Goal end velocity in meters/sec
}

}