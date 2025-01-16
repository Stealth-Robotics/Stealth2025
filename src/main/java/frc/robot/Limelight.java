package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Limelight {
    NetworkTable table;

    public Limelight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }
}
