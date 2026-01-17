package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RobotContainerTest {
  @BeforeEach
  void setup() {
    assertTrue(HAL.initialize(500, 0));
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
