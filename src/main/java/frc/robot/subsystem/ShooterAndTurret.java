// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

public class ShooterAndTurret extends SubsystemBase {

  private static ShooterAndTurret shooter_and_turret = null;

  public static synchronized ShooterAndTurret get(){
    if (shooter_and_turret == null){
      shooter_and_turret = new ShooterAndTurret();
    }
    return shooter_and_turret;
  }
  

  /** Creates a new ShooterAndTurret. */
      private final TalonFX m_Shooter1 = new TalonFX(BotConstants.Shooter.shooterflywheel_1_ID);
      private final TalonFX m_Shooter2 = new TalonFX(BotConstants.Shooter.shooterflywheel_2_ID);
      private final TalonFX m_Hood = new TalonFX(BotConstants.Hood.Hood_ID);
      private final TalonFX m_Turret = new TalonFX(BotConstants.Turret.Turret_ID);

  public ShooterAndTurret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
