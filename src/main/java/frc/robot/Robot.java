// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;
import frc.robot.util.Field.Alliance_Find;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Telemetry logger;
  
  public Robot() {
    Binds.DriverStation2026.bind();
    Binds.Controller.bind();
    this.logger = new Telemetry(BotConstants.DriveConstants.MaxSpeed);    

    
  
    
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
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

@Override
	public void autonomousInit() {
  
		m_autonomousCommand = Auto.getSelectedAuto();

    new Field.Alliance_Find();

    Auto.initialize(
    new Auto.NamedCommand("Shoot", Shooter.get().Shoot(
        BotConstants.Shooter.velocityTable.get(DistanceFinder(Field.Alliance_Find.hub)),
        BotConstants.Hood.shooterTable.get(DistanceFinder(Field.Alliance_Find.hub)))),
    
    new Auto.NamedCommand("Pass_1", Shooter.get().Shoot(
        BotConstants.Shooter.velocityTable.get(DistanceFinder(Field.Alliance_Find.Pass_1)),
        BotConstants.Hood.shooterTable.get(DistanceFinder(Field.Alliance_Find.Pass_1)))),
    
    new Auto.NamedCommand("Pass_2", Shooter.get().Shoot(
        BotConstants.Shooter.velocityTable.get(DistanceFinder(Field.Alliance_Find.Pass_2)),
        BotConstants.Hood.shooterTable.get(DistanceFinder(Field.Alliance_Find.Pass_2)))),

    new Auto.NamedCommand("Intake", Intake.get().doIntake()),
    
    new Auto.NamedCommand("Reset Intake", Intake.get().doStow())

    //new Auto.NamedCommand("Climb L1", Climber.get().climbL1())
  );

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
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
