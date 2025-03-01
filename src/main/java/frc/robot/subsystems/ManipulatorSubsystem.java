package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.EncoderConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    // Defines a TalonFXS motor which is connected to a Minion.
    private final TalonFXS m_manipulatorDriveMotor = new TalonFXS(CANConstants.kManipulatorDriveMotorID);
    // Defines a TalonFX motor which is connected to a Falcon 500.
    private final TalonFX  m_manipulatorAngleMotor = new TalonFX(CANConstants.kManipulatorAngleMotorID);

    // Defines a configuration for the TalonFXS motor.
    private final TalonFXSConfiguration m_manipulatorDriveConfiguration = new TalonFXSConfiguration();
    // Defines a configuration for the TalonFX motor.
    private final TalonFXConfiguration m_manipulatorAngleConfiguration = new TalonFXConfiguration();

    // Defines a MotionMagicVoltage to ellagantly control the TalonFX.
    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    // Defines a DutyCycleEncoder which is a REV Through Bore Encoder.
    private final DutyCycleEncoder m_manipulatorAngleEncoder = new DutyCycleEncoder(DIOConstants.kManipulatorAngleEncoderID);

    /**Creates a Manipulator Subsystem
     * This is used to control the coral manipulator.
     */
    public ManipulatorSubsystem() {
        // Applies motor configurations.
        applyMotorConfigurations();
    }

    /**Creates and Applies Motor Configurations
     * This creates and applies the motor configurations for the 
     * manipulator's drive motor and its turn motor. Along with 
     * this, it creates and configures parameters to be used with
     * motion magic.
     */
    public void applyMotorConfigurations() {
        // Defines a configuration slot for the TalonFX and TalonFXS
        Slot0Configs driveSlot0 = m_manipulatorDriveConfiguration.Slot0;
        Slot0Configs angleSlot0 = m_manipulatorAngleConfiguration.Slot0;

        // Defines a configuration for the MotionMagic
        MotionMagicConfigs angleMotionMagic = m_manipulatorAngleConfiguration.MotionMagic;

        // Tells the TalonFXS that it's connected to a Minion motor.
        m_manipulatorDriveConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        driveSlot0.kS = 0.24;  // Static Feedforward value for TalonFXS
        driveSlot0.kV = 0.12;  // Velocity Feedforward value for TalonFXS
        driveSlot0.kP = 0.11;  // Proportion value for TalonFXS
        driveSlot0.kI = 0.5;   // Integral value for TalonFXS
        driveSlot0.kD = 0.001; // Derivative value for TalonFXS

        angleSlot0.kS = 0.24;  // Static Feedforward value for TalonFX
        angleSlot0.kV = 0.12;  // Velocity Feedforward value for TalonFX
        angleSlot0.kP = 0.11;  // Proportion value for TalonFX
        angleSlot0.kI = 0.5;   // Integral value for TalonFX
        angleSlot0.kD = 0.001; // Derivative value for TalonFX

        angleMotionMagic.MotionMagicCruiseVelocity = 80;   // Maximum MotionMagic Velocity
        angleMotionMagic.MotionMagicAcceleration   = 160;  // Maximum MotionMagic Acceleration
        angleMotionMagic.MotionMagicJerk           = 1600; // Maximum MotionMagic Jerk

        // Apply configurations for the TalonFXS and TalonFX respectively 
        m_manipulatorDriveMotor.getConfigurator().apply(
            m_manipulatorDriveConfiguration, 0.050);
        m_manipulatorAngleMotor.getConfigurator().apply(
                m_manipulatorAngleConfiguration, 0.050);
        
        // Configure the TalonFXS to coast when no output is specified
        m_manipulatorDriveMotor.setNeutralMode(NeutralModeValue.Coast);

        // Configure the TalonFX's encoder to the position read by the absolute encoder.
        m_manipulatorAngleMotor.setPosition(m_manipulatorAngleEncoder.get());
        // Configure the TalonFX to brake when no output is specified 
        m_manipulatorAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**Get Manipulator Command
     * @param speed percentage value [-1, 1] used to control the manipulator's
     * drive motor
     * @return Returns a new RunCommand used to set the speed percentage 
     * for the manipulator's drive motor.
     */
    public Command getManipulatorDriveCommand(double speed) {
        // Define a DutyCycleOut that will be used to move the TalonFXS at the specified speed
        DutyCycleOut manipulatorSpeed = new DutyCycleOut(speed);
        
        // Define a RunCommand that is used to set the speed of the motor
        return new RunCommand(
            () -> m_manipulatorDriveMotor.setControl(manipulatorSpeed), this);
    }

    /**Extends the Coral Manipulator
     * @return Returns a Command that triggers the manipulator to extend.
     */
    public Command extendCoralManipulator() {
        // Define a new command that moves the TalonFX to the desired extension
        return Commands.runOnce(() -> 
            m_manipulatorAngleMotor.setControl(
                m_motionMagicVoltage.withPosition(
                    EncoderConstants.kDesiredManipulatorPositionExtended)
            ), this
        );
    }

    /**Retracts the Coral Manipulator
     * @return Returns a Command that triggers the manipulator to retract.
     */
    public Command retractCoralManipulator() {
        // Define a new command that moves the TalonFX to the desired retraction
        return Commands.runOnce(() -> 
            m_manipulatorAngleMotor.setControl(
                m_motionMagicVoltage.withPosition(
                    EncoderConstants.kDesiredManipulatorPositionRetracted)
            ), this
        );
    }

    @Override
    public void periodic() {}
}
