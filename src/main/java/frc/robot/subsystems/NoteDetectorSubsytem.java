// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetectorSubsytem extends SubsystemBase {
  /** Creates a new NoteDetector. */

  private static NoteDetectorSubsytem instance;

  private final int SensorPort = 0;
  private final double DistThreshold = 3;

  private AnalogInput ultrasonicSensor;

  private double voltageScaleFactor = 1;
  private double dist = DistThreshold + 1;

  public NoteDetectorSubsytem() {
    ultrasonicSensor = new AnalogInput(SensorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    voltageScaleFactor = 5 / RobotController.getVoltage5V();

    // Formula to calculate range in Centimeters:
    dist = ultrasonicSensor.getValue() * voltageScaleFactor * 0.125;
  }

  /*
   * Returns true if ring is seen
   * false if ring is not seen
   */

  public boolean getRingStatus() {
    if (dist <= DistThreshold) {
      return true;
    }
    return false;
  }

  public static NoteDetectorSubsytem getInstance() {
    if (instance == null) {
      instance = new NoteDetectorSubsytem();
      return instance;
    }
    return instance;
  }
}
