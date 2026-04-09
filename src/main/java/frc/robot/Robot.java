// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.BotConstants;
import frc.robot.util.Field;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final Telemetry logger;
  
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  private final CANdle m_candle = new CANdle(0, "rio");


  public Robot() {
    this.logger = new Telemetry(BotConstants.DriveConstants.MaxSpeed);
    CANdleConfiguration configAll = new CANdleConfiguration();

    configAll.LED.StripType = StripTypeValue.GRB;
    configAll.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
    m_candle.getConfigurator().apply(configAll);

    Auto.initialize(
    new Auto.NamedCommand("Shoot", new ShootCommand(()->Field.Alliance_Find.hub).withTimeout(5.5)),

    new Auto.NamedCommand("ShootForever", new ShootCommand(()->Field.Alliance_Find.hub)),


    new Auto.NamedCommand("Shorter Shoot", new ShootCommand(()->Field.Alliance_Find.hub).withTimeout(2.5)),
    
    new Auto.NamedCommand("Intake_", Intake.get().doIntake().withTimeout(2)),

    new Auto.NamedCommand("Intake Forever", Intake.get().doIntake()),
    
    new Auto.NamedCommand("Do stow", Intake.get().doStow().withTimeout(0.5)),

    new Auto.NamedCommand("Rev up", Shooter.get().spinUpCommand()),

    
    new Auto.NamedCommand("Oscilate", Intake.get().doOscilateIntake().withTimeout(4.0)),

    new Auto.NamedCommand("OscilateForever", Intake.get().doOscilateIntake()),


    new Auto.NamedCommand("Outtake", Intake.get().doOuttake().withTimeout(3.5))

    );
    
    Binds.DriverStation2026.bind();
    Binds.OperatorPanel.bind();
    //Binds.Controller.bind();
    
    Intake.get().zeroEncoder();
    Intake.get().SlotZeroConfigIntake();
    //SignalLogger.start();
    //DataLogManager.start();
  
  }

public double DistanceFinder(Translation2d targetPosition){
    return Swerve.get().getPose().getTranslation().getDistance(targetPosition);
  }

  

  @Override
  public void robotPeriodic() {
    Field.Alliance_Find.setAlliance();
    // Publish total telemetry
    // SmartDashboard.putNumber("PDH Total Current", m_pdh.getTotalCurrent());
    // SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    // SmartDashboard.putNumber("PDH Temperature", m_pdh.getTemperature());
    // SmartDashboard.putNumber("Distance to hub", Swerve.get().distTo(Field.Alliance_Find.hub));
    
    logger.telemeterize(Swerve.get().getState());
    CommandScheduler.getInstance().run();

    m_candle.setControl(new SingleFadeAnimation(0, 100).withColor(new RGBWColor(Color.kRed)));

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
