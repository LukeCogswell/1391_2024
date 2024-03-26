// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CANConstants.CANivoreID;

import java.util.Map;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  AddressableLED leds = new AddressableLED(9);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(54); 
  SuppliedValueWidget<Boolean> topThirdColorWidget = Shuffleboard.getTab("Matches").addBoolean("TopThird", () -> false)
    .withPosition(12, 0)
    .withSize(1, 2)
    // .withProperties(Map.of("colorWhenFalse", Color.kBlack.toHexString()))
    ;
  SuppliedValueWidget<Boolean> midThirdColorWidget = Shuffleboard.getTab("Matches").addBoolean("Middle Third", () -> false)
    .withPosition(12, 2)
    .withSize(1, 2)
    // .withProperties(Map.of("colorWhenFalse", Color.kBlack.toString()))
    ;
  SuppliedValueWidget<Boolean> botThirdColorWidget = Shuffleboard.getTab("Matches").addBoolean("Bottom Third", () -> false)
    .withPosition(12, 4)
    .withSize(1, 2)
    // .withProperties(Map.of("colorWhenFalse", Color.kBlack.toString()))
    ;
  SuppliedValueWidget<Boolean> topHalfColorWidget = Shuffleboard.getTab("Matches").addBoolean("Top Half", () -> false)
    .withPosition(11, 0)
    .withSize(1, 3)
    // .withProperties(Map.of("colorWhenFalse", Color.kBlack.toString()))
    ;
  SuppliedValueWidget<Boolean> botHalfColorWidget = Shuffleboard.getTab("Matches").addBoolean("Bottom Half", () -> false)
    .withPosition(11, 3)
    .withSize(1, 3)
    // .withProperties(Map.of("colorWhenFalse", Color.kBlack.toString()))
    ;
  /** Creates a new LEDs. */
  public LEDs() {
    // topThirdColorWidget.withProperties(Map.of("colorWhenFalse", Color.kBlack.toHexString()));
    // midThirdColorWidget.withProperties(Map.of("colorWhenFalse", Color.kBlack.toHexString()));
    // botThirdColorWidget.withProperties(Map.of("colorWhenFalse", Color.kBlack.toHexString()));
    // topHalfColorWidget.withProperties(Map.of("colorWhenFalse", Color.kBlack.toHexString()));
    // botHalfColorWidget.withProperties(Map.of("colorWhenFalse", Color.kBlack.toHexString()));
    leds.setLength(54);
    // Shuffleboard.getTab("Testing");
    // setTopThird(Color.kWhite);
    // setBottomThird(Color.kPurple);
    // setMiddleThird(Color.kBrown);
    // start();
    // setBottomHalf(Color.kRed);
    // setTopHalf(Color.kAqua);

  }

  @Override
  public void periodic() {
      // SmartDashboard.putNumber("BUS OFF COUNT: ", CANBus.getStatus(CANivoreID).BusOffCount);
      // SmartDashboard.putNumber("TRANSMIT BUFFER FULL COUNT: ", CANBus.getStatus(CANivoreID).TxFullCount);
    // setPercent(Timer.getFPGATimestamp() % 1 / 1);
    // setBottomHalf(new Color(255, 0, 0));
    // This method will be called once per scheduler run
  }

  public void start() {
    leds.setData(ledBuffer);
    topThirdColorWidget.withProperties(Map.of("colorWhenFalse", ledBuffer.getLED(25).toHexString()));
    midThirdColorWidget.withProperties(Map.of("colorWhenFalse", ledBuffer.getLED(10).toHexString()));
    botThirdColorWidget.withProperties(Map.of("colorWhenFalse", ledBuffer.getLED(0).toHexString()));
    topHalfColorWidget.withProperties(Map.of("colorWhenFalse", ledBuffer.getLED(25).toHexString()));
    botHalfColorWidget.withProperties(Map.of("colorWhenFalse", ledBuffer.getLED(0).toHexString()));
    leds.start();
  }

  public void set(Color color) {
    for (int i=0; i<= 53; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  public void setPercent(Double percent, Color color) {
    set(Color.kBlack);
    for (int j=0; j<=1; j++){
      for (int i = 0; i <= Math.round(percent*26); i++) {
        ledBuffer.setLED(i + (j*27), color);
      }
    }
    start();
  }

  public void setBottomHalf(Color color) {
    for (int j = 0; j<=1; j++) {
      for (int i = 0; i <= 13; i++) {
        ledBuffer.setLED(i + (j*27), color);
      }
    }
    // leds.setData(ledBuffer);
    // leds.start();
  }
  
  public void setTopHalf(Color color) {
    for (int j = 0; j<=1; j++) {
      for (int i = 14; i <= 26; i++) {
        ledBuffer.setLED(i + (j*27), color);
      }
    }
    // leds.setData(ledBuffer);
    // leds.start();
  }

  public void setBottomThird(Color color) {
    for (int j = 0; j<=1; j++) {
      for (int i = 0; i <= 8; i++) {
        ledBuffer.setLED(i + (j*27), color);
      }
    }
    botThirdColorWidget.withProperties(Map.of("colorWhenTrue", color));
    // leds.setData(ledBuffer);
    // leds.start();
  }

  public void setMiddleThird(Color color) {
    for (int j = 0; j<=1; j++) {
      for (int i = 9; i <= 17; i++) {
        ledBuffer.setLED(i + (j*27), color);
      }
    }
    // leds.setData(ledBuffer);
    // leds.start();
  }

  public void setTopThird(Color color) {
    for (int j = 0; j<=1; j++) {
      for (int i = 18; i <= 26; i++) {
        ledBuffer.setLED(i + (j*27), color);
      }
    }
    // leds.setData(ledBuffer);
    // leds.start();
  }
    
}
