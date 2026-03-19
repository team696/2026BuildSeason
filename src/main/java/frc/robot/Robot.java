// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;
import frc.robot.util.Field.Alliance_Find;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Telemetry logger;
  
  public Robot() {
    this.logger = new Telemetry(BotConstants.DriveConstants.MaxSpeed);
  
    Auto.initialize(
    new Auto.NamedCommand("Shoot", Shooter.get().Shoot(Field.Alliance_Find.hub).withTimeout(4)),
    
    new Auto.NamedCommand("Intake_", Intake.get().doIntake().withTimeout(2.0)),
    
    new Auto.NamedCommand("Do stow", Intake.get().doStow().withTimeout(0.5)),

    new Auto.NamedCommand("Extend Climber", Climber.get().doExtend().withTimeout(2.0)),

    new Auto.NamedCommand("Retract Climber", Climber.get().doRetract().withTimeout(2.0))
    );
    
    // Binds.DriverStation2026.bind(); // Joysticks
    // Binds.OperatorPanel.bind();
    Binds.Controller.bind();  // X Box controller
    
    Climber.get().zeroEncoder();
    Shooter.get().zero_shooter();
    Intake.get().zeroEncoder();


    
    
    
  }

public double DistanceFinder(Translation2d targetPosition){
    return Swerve.get().getPose().getTranslation().getDistance(targetPosition);
  }

  @Override
  public void robotPeriodic() {
    logger.telemeterize(Swerve.get().getState());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

@Override
	public void autonomousInit() {
		m_autonomousCommand = Auto.getSelectedAuto();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}

    Climber.get().gotToZero();
	}



  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
