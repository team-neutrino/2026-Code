# 2026-Code

## Rebuilt documentation

- [Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)
- [Full drawing package](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf)
- [April Tags](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-apriltag-images-user-guide.pdf)

## Useful Java Docs

- [WPILIB](https://github.wpilib.org/allwpilib/docs/release/java/index.html)
- [Phoenix](https://api.ctr-electronics.com/phoenix6/release/java/)
- [Rev](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
- [LimeLight](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib)

## Drivetrain CANivore Bus CAN IDs
| Subsystem | Description | CAN ID  | Device      |
| --------- | ----------- | ------- | ----------- |
| Swerve    | IMU         | 0       | Pigeon v2   |
| Swerve    | FL Encoder  | 1       | CANCoder    |
| Swerve    | FR Encoder  | 2       | CANCoder    |
| Swerve    | BR Encoder  | 3       | CANCoder    |
| Swerve    | BL Encoder  | 4       | CANCoder    |
| Swerve    | FL Angle    | 5       | Kraken x44  |
| Swerve    | FL Speed    | 6       | Kraken x60  |
| Swerve    | FR Angle    | 7       | Kraken x44  |
| Swerve    | FR Speed    | 8       | Kraken x60  |
| Swerve    | BR Angle    | 9       | Kraken x44  |
| Swerve    | BR Speed    | 10      | Kraken x60  |
| Swerve    | BL Angle    | 11      | Kraken x44  |
| Swerve    | BL Speed    | 12      | Kraken x60  |

## Rio Bus CAN IDs
| Subsystem | Description       | CAN ID  | Device      |
| --------- | ----------------- | ------- | ----------- |
| Intake    | Roller            | 12      | Kraken x60  |
| Intake    | Deploy            | 13      | Kraken x44  |
| Indexer   | Spindexer         | 14      | Kraken x44  |
| Shooter   | Hood              | 15      | Kraken x44  |
| Shooter   | Flywheel Main     | 16      | Kraken x60  |
| Shooter   | Flywheel Follower | 17      | Kraken x60  |
| Kicker    | Kicker            | 18      | Kraken x60  |
| Climb     |                   | 19      | Kraken x60  |
| Climb     |                   | 20      | CANrange    |
| Indexer   | Left Top Hopper   | 25      | CANrange    |
| Indexer   | Right Top Hopper  | 26      | CANrange    |
| Indexer   |                   | 28      | CanandColor |
| Turret    |                   | 29      | Kraken x44  |

## DIO
| Subsystem | Description          | Port   |
| --------- | -------------------- | ------ |

## PWM
| Subsystem | Description          | Port   |
| --------- | -------------------- | ------ |

## PDH
| Port | Destination          | Breaker (A) | Wire Gauge |
| ---- | -------------------- | ----------- | ---------- |
| 0    | Left Front Speed     | 40          | 10         |
| 1    | Right Front Speed    | 40          | 10         |
| 2    | Left Back Angle      | 40          | 12         |
| 3    | Left Front Angle     | 40          | 12         |
| 4    | Right Front Angle    | 40          | 12         |
| 5    | Spindexer            | 40          | 10         |
| 6    | Deploy Intake        | 40          | 10         |
| 7    | —                    | —           | —          |
| 8    | —                    | —           | —          |
| 9    | —                    | —           | —          |
| 10   | —                    | —           | —          |
| 11   | —                    | —           | —          |
| 12   | —                    | —           | —          |
| 13   | —                    | —           | —          |
| 14   | Tower                | 40          | 10         |
| 15   | Spinning Intake      | 40          | 10         |
| 16   | Right Back Angle     | 40          | 12         |
| 17   | Mini Power Module    | 40          | 12         |
| 18   | Right Back Speed     | 40          | 10         |
| 19   | Left Back Speed      | 40          | 10         |
| 20   | RoboRIO              | 10          | 18         |
| 21   | —                    | —           | —          |
| 22   | —                    | —           | —          |
| 23   | —                    | —           | —          |

## MPM
| Port | Destination | Breaker (A) |
| ---- | ----------- | ----------- |
| 01   | Brainbox    | 10          |
| 02   | Radio       | 10          |
| 03   | 4 CANCoders | 10          |
| 04   | CANivore    | 10          |
| 05   | Pigeon 2    | 10          |

## Buttons Controller
| Button        | Function        |
| ------------- | --------------- |
| Left Trigger  | Intake          |
| Right Trigger | Outtake         |
| Y             | Gentle shot     |
| Back Button   | Engage Climb    |
| Start Button  | Disengage Climb |

## Driver Controller
| Button        | Function        |
| ------------- | --------------- |
| Left Stick    | Manual Driving  |
| Right Stick   | Manual Rotation |
| Left Bumper   | Drive + Climb   |
| Y             | Drive + Shuttle |
| X             | Drive + Score   |
| B             | Drive + Intake  |
| Back          | Reset Yaw       |
| D-Pad         | Change target   |


## Auton Paths
Neutral Zone Paths:
- Depot-side start + end
- Depot-side start, outpost-side end

## Raspberry Pi
