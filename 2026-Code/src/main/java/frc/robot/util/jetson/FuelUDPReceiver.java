package frc.robot.util.jetson;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import static frc.robot.util.Constants.VisionConstants.*;

public class FuelUDPReceiver extends SubsystemBase {
    // The "Lock" object prevents the Main Thread and UDP Thread from
    // colliding while reading/writing these variables.
    private final Object lock = new Object();
    private double x, y, theta;
    private double lastTimestamp;

    public FuelUDPReceiver() {
        // Create the background thread so we don't block the main robot loop
        Thread udpThread = new Thread(this::runSocket);

        // Naming the thread makes it show up clearly in the WPILib Profiler
        udpThread.setName("UDP-Vision-Thread");

        // Daemon ensures the thread closes when the robot code stops/restarts
        udpThread.setDaemon(true);
        udpThread.start();
    }

    private void runSocket() { // try-with-resources ensures the socket closes if the thread crashes
        try (DatagramSocket socket = new DatagramSocket(PORT_ID)) {
            byte[] buffer = new byte[24]; // 3 doubles * 8 bytes = 24 bytes

            while (!Thread.currentThread().isInterrupted()) {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

                // This line PAUSES the thread until a packet arrives.
                // Because this is a separate thread, the robot keeps driving.
                socket.receive(packet);

                ByteBuffer bb = ByteBuffer.wrap(buffer);
                bb.order(ByteOrder.BIG_ENDIAN); // Not sure if BIG or Littel ENDIAN matters

                double newX = bb.getDouble();
                double newY = bb.getDouble();
                double newTheta = bb.getDouble();

                // Move the data into the shared variables safely
                synchronized (lock) {
                    x = newX;
                    y = newY;
                    theta = newTheta;
                    lastTimestamp = Timer.getFPGATimestamp();
                }
            }
        } catch (Exception e) {
            DriverStation.reportError("Vision UDP Error: " + e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * * Returns true if we have received a packet in the last 0.5 seconds.
     */
    public boolean isVisionFresh() {
        synchronized (lock) {
            return (Timer.getFPGATimestamp() - lastTimestamp) < 0.5;
        }
    }

    // Getters to safely access data from other parts of the robot
    public double getX() {
        synchronized (lock) {
            return x;
        }
    }

    public double getY() {
        synchronized (lock) {
            return y;
        }
    }

    public double getTheta() {
        synchronized (lock) {
            return theta;
        }
    }

    @Override
    public void periodic() {
        // System.out.println("X" + getX());
        // System.out.println("Y" + getY());
        // System.out.println("Theta" + getTheta());
    }
}