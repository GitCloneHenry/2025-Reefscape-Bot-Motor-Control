package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.ModuleConstants;

public class TiltRampSubsystem extends SubsystemBase {
    private final SparkMax m_tiltRampDriveMotor = new SparkMax(CANConstants.kTiltRampDriveMotorID, MotorType.kBrushless);;
    private final TalonFXS m_tiltRampAngleMotor = new TalonFXS(CANConstants.kTiltRampAngleMotorID);

    private final TalonFXSConfiguration m_tiltRampAngleConfiguration = new TalonFXSConfiguration();

    private final SparkClosedLoopController m_tiltRampDriveController = m_tiltRampDriveMotor.getClosedLoopController();

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final DutyCycleEncoder m_tiltRampAngleEncoder = new DutyCycleEncoder(DIOConstants.kTiltRampAngleEncoderID);

    public TiltRampSubsystem() {
        applyMotorConfigurations();
    }

    public void applyMotorConfigurations() {
        SparkMaxConfig floorIntakeDriveConfig = new SparkMaxConfig();
        double floorIntakeDriveFeedForward = 1 / ModuleConstants.kFloorIntakeDriveFreeSpeedRps;

        floorIntakeDriveConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(12);
        floorIntakeDriveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0.0, 0.0)
            .velocityFF(floorIntakeDriveFeedForward)
            .outputRange(-1, 1);

        m_tiltRampDriveMotor.configure(
            floorIntakeDriveConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

        Slot0Configs angleSlot0 = m_tiltRampAngleConfiguration.Slot0;

        MotionMagicConfigs angleMotionMagic = m_tiltRampAngleConfiguration.MotionMagic;

        m_tiltRampAngleConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        angleSlot0.kS = 0.24;
        angleSlot0.kV = 0.12;
        angleSlot0.kP = 0.11;
        angleSlot0.kI = 0.5;
        angleSlot0.kD = 0.001;

        angleMotionMagic.MotionMagicCruiseVelocity = 80; 
        angleMotionMagic.MotionMagicAcceleration   = 160;
        angleMotionMagic.MotionMagicJerk           = 1600;

        m_tiltRampAngleMotor.getConfigurator().apply(
            m_tiltRampAngleConfiguration, 0.050);
        
        m_tiltRampAngleMotor.setPosition(m_tiltRampAngleEncoder.get());
        m_tiltRampAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command startFloorIntake() {
        return Commands.runOnce(() -> {
            m_tiltRampAngleMotor.setControl(m_motionMagicVoltage.withPosition(100));
            m_tiltRampDriveController.setReference(1, ControlType.kVelocity);
        }, this);
    }

    public Command stopFloorIntake() {
        m_tiltRampAngleMotor.setControl(m_motionMagicVoltage.withPosition(100));
        return Commands.runOnce(() -> m_tiltRampDriveController.setReference(0, ControlType.kVelocity), this);
    }
}
