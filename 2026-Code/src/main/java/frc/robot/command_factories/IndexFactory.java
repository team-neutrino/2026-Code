package frc.robot.command_factories;

import static frc.robot.util.Constants.IndexerConstants.*;
import static frc.robot.util.Subsystem.index;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IndexFactory {
    public static Command runIndexer(){
        return new SequentialCommandGroup(
            index.runSpindexer(INDEXING_VOLTAGE),
            index.runOutputWheel(OUTPUT_WHEEL_VOLTAGE));
    }
    
}