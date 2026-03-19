package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.conveyorConstant;

public class Conveyor extends SubsystemBase{
    private final TalonFX conveyorIndexer = new TalonFX(conveyorConstant.indexer, "CANivore");
    private final TalonFX conveyorRoller = new TalonFX(conveyorConstant.roller, "CANivore");

    public void conveyorTransmiss() {
        conveyorIndexer.set(0.8);
        conveyorRoller.set(0.8);
    }

    public void converyStop() {
        conveyorIndexer.set(0);
        conveyorRoller.set(0);
    }
}
