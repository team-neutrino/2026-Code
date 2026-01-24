package frc.robot.command_factories;

import static frc.robot.util.Constants.IndexerConstants.*;
import static frc.robot.util.Subsystems2026.index;

import edu.wpi.first.wpilibj2.command.Command;

public class IndexFactory {
    public static Command runSpindexer() {
        return index.runSpindexer(INDEXING_VOLTAGE);
    }
}
