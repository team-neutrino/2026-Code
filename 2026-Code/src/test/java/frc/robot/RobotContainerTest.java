// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotContainerTest {
  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
  }

  @Test
  void testRobotContainerConstruction() {
    assertDoesNotThrow(() -> new RobotContainer());
  }

  @Test
  void testGetAutonomousCommand() {
    RobotContainer robotContainer = new RobotContainer();
    Command autoCommand = robotContainer.getAutonomousCommand();
    assertNotNull(autoCommand, "Autonomous command should not be null");
  }
}
