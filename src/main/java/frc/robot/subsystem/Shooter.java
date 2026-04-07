// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;


import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.BotConstants;


public class Shooter extends SubsystemBase {

  private static Shooter shooter = null;

  public static synchronized Shooter get(){
    if (shooter == null){
      shooter = new Shooter();
    }
    return shooter;
  }
  

  /** Creates a new Shooter. */
      //Motors
      private final TalonFX m_Shooter = new TalonFX(BotConstants.Shooter.shooterflywheel_ID);
      private final TalonFX m_Shooter_2 = new TalonFX(BotConstants.Shooter.shooterflywheel2_ID);
      private final TalonFX m_ShooterIntake = new TalonFX(BotConstants.Shooter.shooterIntake_ID);
      //Controllers
      private final MotionMagicVelocityVoltage shooterVelocityController = new MotionMagicVelocityVoltage(0);
      private final VelocityVoltage intakeRollerController = new VelocityVoltage(0);




      //Simulation
      private final FlywheelSim m_FlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.008, 1.0), DCMotor.getKrakenX60(2), 0.008);
            private final FlywheelSim m_FLywheelSim2 = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.008, 1.0), DCMotor.getKrakenX60(2), 0.008);

      //Data
      private StatusSignal<AngularVelocity> velocity_roller;
      private StatusSignal<AngularVelocity> velocity_roller_2;
      private StatusSignal<AngularVelocity> indexer_velocity;

  public Shooter() {
      //Config set up
      m_Shooter.getConfigurator().apply(BotConstants.Shooter.cfg_shooter);
      m_ShooterIntake.getConfigurator().apply(BotConstants.Shooter.cfg_shooter_intake);

        SmartDashboard.putNumber("Desired velocity", 0);
        SmartDashboard.putNumber("Desired Backspin", 0);
        SmartDashboard.putNumber("Indexer Velocity", 0);

        SmartDashboard.putNumber("Fudge Factor for Shooter", 0);

        

      SmartDashboard.putNumber("Launch Speed", 0);
      SmartDashboard.putNumber("Front roller", 0);
      SmartDashboard.putNumber("Indexer speed", 0);
      

      
      velocity_roller = m_Shooter.getVelocity();
      velocity_roller_2 = m_Shooter_2.getVelocity();
      indexer_velocity = m_ShooterIntake.getVelocity();
  }

  //Sets the velocity
  public void set_velocity(double velocity){
    m_Shooter.setControl(shooterVelocityController.withVelocity(velocity));
    m_Shooter_2.setControl(new Follower(BotConstants.Shooter.shooterflywheel_ID,MotorAlignmentValue.Opposed)); 
  }


  //Intakes the ball for the shooter
  public void intake_shooter(double speed){
    m_ShooterIntake.setControl(intakeRollerController.withVelocity(speed));
  }



  public Command idle(){
    return run(()->{this.Stop();; Hopper.get().Stop();});
  }

  //Commented out, runtime overrun error

  // public Command Shoot(Translation2d desired_pose){
  //     return runEnd(()->{
  //       double fudgefactor = SmartDashboard.getNumber("Fudge Factor for Shooter", 0);
  //       double distMeters=Swerve.get().distTo(desired_pose);
  //       double velocity = BotConstants.Shooter.ShooterTable.get(distMeters) - fudgefactor;
  //       double intakespeed = BotConstants.Shooter.backSpinTable.get(distMeters);
  //       DriverStation.reportWarning("Fudge Factor recieved:"+fudgefactor, false);
  //       SmartDashboard.putNumber("Desired velocity", velocity);
  //       SmartDashboard.putNumber("Desired Backspin", intakespeed);

  //        this.set_velocity(velocity);

  //       if((Math.abs(getRollerVelocity()-velocity))<1.1){
  //             this.intake_shooter(intakespeed);
  //             Hopper.get().run_Hopper();
  //           }
  //       else{
  //         m_ShooterIntake.stopMotor();
  //         Hopper.get().Stop();
  //       }

  //     },
  //     ()->{
  //         this.Stop();
  //         Hopper.get().Stop();
  //     });
  //   }

  public Command ShootPass(){
      return runEnd(()->{
        // double distMeters=Swerve.get().distTo(desired_pose);
        double velocity = -45.0;
        double intakespeed = -40;

         this.set_velocity(velocity);

        if((Math.abs(getRollerVelocity()-velocity))<2.5){
              this.intake_shooter(intakespeed);
              Hopper.get().run_Hopper();
            }
        else{
          m_ShooterIntake.stopMotor();
          Hopper.get().Stop();
        }

      },
      ()->{
          m_Shooter.stopMotor();
          m_Shooter_2.stopMotor();
          m_ShooterIntake.stopMotor();
          Hopper.get().Stop();
      });
    }

    public Command ShootDash(){
      return runEnd(()->{
        double velocity = SmartDashboard.getNumber("Launch Speed", 0.0);
        double intakespeed = SmartDashboard.getNumber("Indexer speed", 0.0);

        this.set_velocity(velocity);

        Hopper.get().run_Hopper();

        if((Math.abs(getRollerVelocity()-velocity))<3){
              this.intake_shooter(intakespeed);
            }
        else{
          m_ShooterIntake.stopMotor();
        }

      },
      ()->{
          m_Shooter.stopMotor();
          m_Shooter_2.stopMotor();
          m_ShooterIntake.stopMotor();
          Hopper.get().Stop();
      });
    }


    // AST added command to spin up shooter roller during auto while driving
    public Command spinUpCommand(){
      return run(()->{
        this.set_velocity(-15);
      });
  }
    
  public Command reverse(){
      return runEnd(()->{
        this.set_velocity(30);
      },
      ()->{
        m_Shooter.stopMotor();
      });
  }

  //Stops everything
  public void Stop(){
      m_Shooter.stopMotor();
      m_Shooter_2.stopMotor();
      m_ShooterIntake.stopMotor();
      Hopper.get().Stop();

  }

  //Data stuff
 
  public double getRollerVelocity(){
    return (velocity_roller.refresh().getValueAsDouble() + velocity_roller_2.refresh().getValueAsDouble()) / 2;
  }

  public double getIndexerVelocity(){
    return indexer_velocity.refresh().getValueAsDouble();
  }


  @Override
  public void simulationPeriodic(){
      // 1. Get the voltage from the TalonFX SimState
  // This is the voltage the PID controller is CURRENTLY sending to the motor
  double motorVoltage = m_Shooter.getSimState().getMotorVoltage();
  double indexerVoltage = m_ShooterIntake.getSimState().getMotorVoltage();

  // 2. Update the physics model
  m_FlywheelSim.setInputVoltage(motorVoltage);
  m_FlywheelSim.update(0.020); // standard 20ms loop

  m_FLywheelSim2.setInputVoltage(indexerVoltage);
  

  // 3. Update the TalonFX sensor simulation
  // This makes m_Shooter.getVelocity() actually return the simulated speed
    m_Shooter.getSimState().setRotorVelocity(
    m_FlywheelSim.getAngularVelocityRadPerSec() / (2 * Math.PI) // convert rad/s to Rotations/s
  );
  
  // Update supply voltage so the motor thinks it has power
    m_Shooter.getSimState().setSupplyVoltage(12.0);
  }
  
  @Override
  public void periodic() {
    //Data stuff used in Autoalign
    getRollerVelocity();
    getIndexerVelocity();

    SmartDashboard.putNumber("Velocity", getRollerVelocity());
    SmartDashboard.putNumber("Indexer Velocity", getIndexerVelocity());


  }
}
