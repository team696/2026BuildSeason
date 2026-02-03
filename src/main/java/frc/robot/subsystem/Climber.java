// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
This is for Akivas clibmer


*/

package frc.robot.subsystem;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BotConstants;

public class Climber extends SubsystemBase {

  private static Climber climber = null;

  public static synchronized Climber get(){
    if(climber == null){
      climber = new Climber();
    }
    return climber;
  }

private enum HookPositions {
    IDLE(0.0),
    HOOK(90.0);

    private final double angle;

    HookPositions(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}

// Example usage:
private HookPositions mHook = HookPositions.HOOK;
private HookPositions mIdle = HookPositions.IDLE;

    private final TalonFX m_Climber1 = new TalonFX(BotConstants.Climber.Climber_1_ID);
    private final TalonFX m_Climber2 = new TalonFX(BotConstants.Climber.Climber_2_ID);
    private final MotionMagicVelocityVoltage climberVelocityController = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage climberPivotController = new MotionMagicVoltage(0);
    private StatusSignal<Angle> position_Climber;

  public Climber(){
    m_Climber1.getConfigurator().apply(BotConstants.Climber.cfg_Climber1);
    m_Climber2.getConfigurator().apply(BotConstants.Climber.cfg_Climber2);
    position_Climber = m_Climber1.getPosition();
  }


public Command pivotClimb(){
    return run(()->{m_Climber2.setControl(climberPivotController.withPosition(mHook.getAngle()));     
    })
  ;
}



public Command climbL1(){
    return run(()->{m_Climber1.setControl(climberVelocityController.withVelocity(1));     
    }).until(()->atPosition(1.0));
}

public Command climbL2(){
    return run(()->{m_Climber1.setControl(climberVelocityController.withVelocity(1));     
    }).until(()->atPosition(2.0));
}

public Command climbL3(){
    return run(()->{m_Climber1.setControl(climberVelocityController.withVelocity(1));     
    }).until(()->atPosition(3.0));
}


private Boolean atPosition(double Chainrotation){
  double numSproketRotations = BotConstants.Climber.chainLength / BotConstants.Climber.sproketDiameter;
  double chainRotations = (1/BotConstants.Climber.gearRatio)*(1/numSproketRotations);
  return Chainrotation>=(position_Climber.getValueAsDouble()/chainRotations);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
