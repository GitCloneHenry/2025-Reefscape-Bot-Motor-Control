package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorDrive = new TalonFX(CANConstants.kElevatorDriveID);

    public ElevatorSubsystem() {}

    @Override
    public void periodic() {}
}
