
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.NetworkTables;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelNT extends SubsystemBase {
  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private DoubleTopic distance = m_nt.getDoubleTopic("distance");
  private DoubleSubscriber getDistance = distance.subscribe(0, null);

  public FuelNT() {

  }

  @Override
  public void periodic() {
    System.out.println(getDistance.get());
  }
}
