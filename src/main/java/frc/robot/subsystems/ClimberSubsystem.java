package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.EncoderConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX m_climberAngleMotor = new TalonFX(CANConstants.kClimberAngleMotorID);

    private final TalonFXConfiguration m_climberAngleConfiguration = new TalonFXConfiguration();

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final DutyCycleEncoder m_climberAngleEncoder = new DutyCycleEncoder(DIOConstants.kClimberAngleEncoderID);

    private double m_targetClimberPosition;

    public ClimberSubsystem(CommandXboxController driverController) {
        applyMotorConfigurations(); 
    }

    public void applyMotorConfigurations() {
        Slot0Configs angleSlot0 = m_climberAngleConfiguration.Slot0;

        MotionMagicConfigs angleMotionMagic = m_climberAngleConfiguration.MotionMagic;

        angleSlot0.kS = 0.24;
        angleSlot0.kV = 0.12;
        angleSlot0.kP = 0.11;
        angleSlot0.kI = 0.5;
        angleSlot0.kD = 0.001;

        angleMotionMagic.MotionMagicCruiseVelocity = 80; 
        angleMotionMagic.MotionMagicAcceleration   = 160;
        angleMotionMagic.MotionMagicJerk           = 1600;

        m_climberAngleMotor.getConfigurator().apply(
                m_climberAngleConfiguration, 0.050);
        
        m_climberAngleMotor.setPosition(m_climberAngleEncoder.get());
        m_climberAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void incrementClimberPosition(double value) {
        double encoderPosition = m_climberAngleMotor.getPosition().getValueAsDouble();

        if (encoderPosition + value < EncoderConstants.kMinimumAcceptableClimberPosition || 
            encoderPosition + value > EncoderConstants.kMaximumAcceptableClimberPosition) {
            return;
        }

        m_targetClimberPosition += value;

        m_climberAngleMotor.setControl(m_motionMagicVoltage.withPosition(m_targetClimberPosition));
    }
}
