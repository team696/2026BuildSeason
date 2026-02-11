// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HumanControls;
import frc.robot.TunerConstants;
import frc.robot.subsystem.Hopper;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;

public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */

    //FCFA reqest, Field Centric Drive Facing Angle request, allows driving while letting the angle be hijacked
  	private final static SwerveRequest.FieldCentricFacingAngle FCFARequest = 
					new SwerveRequest.FieldCentricFacingAngle()
					.withDeadband(BotConstants.DriveConstants.MaxSpeed* 0.1)
					.withRotationalDeadband(BotConstants.DriveConstants.MaxAngularRate * 0.15) // Add a  deadband
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
					.withHeadingPID(3, 0, 0); 

      HumanControls joystick = new HumanControls();

      //Tunner Constant stuff for driving 
      private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
      public final  double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
      //Target pose
      private Translation2d targetPosition;
		

  
  public AutoAlign(Translation2d targetPosition) {

    targetPosition = this.targetPosition;
    addRequirements(Swerve.get(),Shooter.get(),Hopper.get());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Calculates distance
    double distance = Swerve.get().getPose().getTranslation().getDistance(targetPosition);
    //Calculates shooter angle using interpolation table in BotConstants
    double shooterangle = BotConstants.Hood.shooterTable.get(distance);
    //Calculate velocity using interpolation table in BotConstants
    double shootervelocity = BotConstants.Shooter.velocityTable.get(distance);

    //Drives the swerve using the FCFA request
    Swerve.get().applyRequest(()->
        FCFARequest
        .withVelocityX((HumanControls.DriverPanel.leftJoyY.getAsDouble()/2)*MaxSpeed)
        .withVelocityY((HumanControls.DriverPanel.leftJoyX.getAsDouble()/2)*MaxSpeed)
        .withTargetDirection(Swerve.get().target_theta(targetPosition))
        .withHeadingPID(5,0,0)
        .withMaxAbsRotationalRate(DegreesPerSecond.of(360))
        .withRotationalDeadband(DegreesPerSecond.of(1)));

    

    //Sets the hood angle and velocity
    Shooter.get().setHoodAngle(shooterangle);
    Shooter.get().set_velocity(shootervelocity);

    //Checks if everything is in place before the hopper runs
    if(Math.abs((shooterangle/360)-Shooter.get().getHoodPosition()) >= 0.1 &&
      Math.abs(shootervelocity-Shooter.get().getRollerVelocity()) >= 0.2){
        Hopper.get().run_Hopper();
    }


  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().Stop();
    Hopper.get().Stop();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){


    Shooter.get().setHoodAngle(0);
    Shooter.get().Stop();
    Hopper.get().Stop();

    return false;
  }



}
