// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Field.Alliance_Find;

public class LED extends SubsystemBase {

  private static LED led = null;
  //Prevents the need of duplicate objects
  public static synchronized LED get(){
    if(led == null){
      led = new LED();
    } 
    return led;
  }
  /** Creates a new LED. */

  private final CANdle m_candle = new CANdle(0, "rio");


  public LED() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.LED.StripType = StripTypeValue.GRB;
        configAll.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        m_candle.getConfigurator().apply(configAll);
        setDefaultCommand(AlianceColorLED());
  }


  public void LEDblue(){
    m_candle.setControl(new SolidColor(0, 100).withColor(new RGBWColor(Color.kBlue)));
    
  }

  public void LEDred(){
    m_candle.setControl(new SolidColor(0, 100).withColor(new RGBWColor(Color.kRed)));
    
  }

  public void LEDyellowBlink(){
    m_candle.setControl(new StrobeAnimation(0, 100).withColor(new RGBWColor(Color.kYellow)).withFrameRate(4));
  }

  public void LEDgreenBlink(){
    m_candle.setControl(new StrobeAnimation(0, 100).withColor(new RGBWColor(Color.kGreen)).withFrameRate(4));
  }

  public void tuffAnimation(){
    m_candle.setControl(new LarsonAnimation(0, 100).withBounceMode(LarsonBounceValue.Back).withColor(new RGBWColor(Color.kHotPink)).withFrameRate(25).withSize(3));
  }

  public void applyAllianceColor() {
    if (Alliance_Find.alliance == Alliance.Blue) {
      LEDblue();
    } else if (Alliance_Find.alliance == Alliance.Red) {
      LEDred();
    } else {
      // default/fallback
      LEDyellowBlink();
    }
  }

  public Command AlianceColorLED(){
    return run(() -> applyAllianceColor());
  }

  public Command CodeInitialize(){
    return run(()->tuffAnimation());
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
