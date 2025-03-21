package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.EncoderConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_climberAngleMotor = new TalonFX(CANConstants.kClimberAngleMotorID);

  private final TalonFXConfiguration m_climberAngleConfiguration = new TalonFXConfiguration();

  private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  private final DutyCycleEncoder m_climberAngleEncoder =
      new DutyCycleEncoder(DIOConstants.kClimberAngleEncoderID);

  private double m_targetClimberPosition = 10.0;

  public void applyMotorConfigurations() {
    Slot0Configs angleSlot0 = m_climberAngleConfiguration.Slot0;

    MotionMagicConfigs angleMotionMagic = m_climberAngleConfiguration.MotionMagic;

    AudioConfigs audioConfigs = m_climberAngleConfiguration.Audio;

    audioConfigs.AllowMusicDurDisable = true;

    angleSlot0.kS = 0.0;
    angleSlot0.kV = 0.0;
    angleSlot0.kP = 2.0;
    angleSlot0.kI = 0.0;
    angleSlot0.kD = 0.0;

    angleMotionMagic.MotionMagicCruiseVelocity = 4500;
    angleMotionMagic.MotionMagicAcceleration = 7000;
    angleMotionMagic.MotionMagicJerk = 21000;

    m_climberAngleMotor.getConfigurator().apply(m_climberAngleConfiguration, 0.050);

    m_climberAngleMotor.setPosition((0.597 - m_climberAngleEncoder.get()) * 100.0 * 48.0 / 15.0);
    m_climberAngleMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public TalonFX getMotor() {
    return m_climberAngleMotor;
  }

  public void incrementClimberPosition(double value) {
    m_targetClimberPosition =
        Math.min(
            Math.max(
                m_targetClimberPosition + value * 1.5,
                EncoderConstants.kMinimumAcceptableClimberPosition),
            EncoderConstants.kMaximumAcceptableClimberPosition);
  }

  public void setClimberPosition(double value) {
    m_targetClimberPosition =
        Math.min(
            Math.max(value, EncoderConstants.kMinimumAcceptableClimberPosition),
            EncoderConstants.kMaximumAcceptableClimberPosition);
  }

  public double getErrorFromTarget() {
    return Math.abs(m_climberAngleMotor.getClosedLoopError().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // System.err.println((0.447 - m_climberAngleEncoder.get()) * 100.0 * 48.0 / 15.0 + ", " + m_climberAngleMotor.getPosition().getValueAsDouble());

    // System.err.println(m_climberAngleEncoder.get());

    m_climberAngleMotor.setControl(m_motionMagicVoltage.withPosition(m_targetClimberPosition));
  }
}
