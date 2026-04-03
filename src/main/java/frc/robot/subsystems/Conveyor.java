package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.canBUS;
import frc.robot.Constant.conveyorConstant;

public class Conveyor extends SubsystemBase{
    private final TalonFX conveyorIndexer = new TalonFX(conveyorConstant.indexer, canBUS.canivore);
    private final TalonFX conveyorRoller = new TalonFX(conveyorConstant.roller, canBUS.canivore);

    public void conveyorTransmiss() {
        conveyorIndexer.set(conveyorConstant.transmiss);
        conveyorRoller.set(conveyorConstant.transmiss);
    }

    public void converyorReverse() {
        conveyorIndexer.set(conveyorConstant.reverse);
        conveyorRoller.set(conveyorConstant.reverse);
    }

    public void converyStop() {
        conveyorIndexer.set(conveyorConstant.stop);
        conveyorRoller.set(conveyorConstant.stop);
    }
}
