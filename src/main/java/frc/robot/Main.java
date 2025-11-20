// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // Default robot code
    RobotBase.startRobot(RobotWithExamples::new);
    
    // To test AdvantageKit example subsystems with simulated controller:
    // Uncomment the line below and comment out the line above
    // RobotBase.startRobot(RobotWithExamples::new);
    
    // See EXAMPLE_SUBSYSTEMS_USAGE.md for complete instructions
  }


  // with omer adam everything works :)
}
