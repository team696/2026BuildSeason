// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShootCommand;
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
  
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);


  public Robot() {
    this.logger = new Telemetry(BotConstants.DriveConstants.MaxSpeed);
  
    Auto.initialize(
    new Auto.NamedCommand("Shoot", new ShootCommand(()->Field.Alliance_Find.hub).withTimeout(3.5)),

    new Auto.NamedCommand("ShootForever", new ShootCommand(()->Field.Alliance_Find.hub)),


    new Auto.NamedCommand("Shorter Shoot", new ShootCommand(()->Field.Alliance_Find.hub).withTimeout(2.5)),
    
    new Auto.NamedCommand("Intake_", Intake.get().doIntake().withTimeout(2)),
    
    new Auto.NamedCommand("Do stow", Intake.get().doStow().withTimeout(0.5)),

    new Auto.NamedCommand("Extend Climber", Climber.get().doExtend().withTimeout(3.0)),

    new Auto.NamedCommand("Oscilate", Intake.get().doOscilateIntake().withTimeout(4.0)),

    new Auto.NamedCommand("OscilateForever", Intake.get().doOscilateIntake()),

    new Auto.NamedCommand("Retract Climber", Climber.get().doRetract().withTimeout(2.0))
    );
    
    Binds.DriverStation2026.bind();
    Binds.OperatorPanel.bind();
    //Binds.Controller.bind();
    
    Climber.get().zeroEncoder();
    Intake.get().zeroEncoder();
    //SignalLogger.start();
    /*UsbCamera cam = CameraServer.startAutomaticCapture(0);
    cam.setFPS(24);
    cam.setResolution(120, 120);*/
    
  
  }

public double DistanceFinder(Translation2d targetPosition){
    return Swerve.get().getPose().getTranslation().getDistance(targetPosition);
  }

  

  @Override
  public void robotPeriodic() {
    Field.Alliance_Find.setAlliance();
    // Publish total telemetry
    SmartDashboard.putNumber("PDH Total Current", m_pdh.getTotalCurrent());
    SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("PDH Temperature", m_pdh.getTemperature());
    
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

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}

    //Climber.get().gotToZero();
	}



  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

  // Intake.get().m_IntakePivot.getConfigurator().apply(BotConstants.Intake.cfg_Pivot_Deploy);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    /*Climber.get().doExtend();*/

     Climber.get().gotToZero().schedule();
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
