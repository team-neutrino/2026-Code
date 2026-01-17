package frc.robot.command_factories;

import static frc.robot.util.Constants.IndexerConstants.*;
import static frc.robot.util.Subsystem.indexer;


import edu.wpi.first.wpilibj2.command.Command;

public class IndexerFactory {
    public static Command runIndexer(){
        return indexer.runIndexer(INDEXING_VOLTAGE);
    }
}
