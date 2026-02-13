// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.jetson;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class ReceiveData extends SubsystemBase {
  private DatagramSocket m_socket;

  public ReceiveData() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
