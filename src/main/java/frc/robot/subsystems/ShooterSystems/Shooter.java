package frc.robot.subsystems.ShooterSystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase 
{
    private static final int CONFIG_RETRIES = 5;

    private static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 24/23; // TODO: check ratio
    private static final double MM_CRUISE_RPS = 120.0;
    private static final double MM_ACCEL_RPS2 = 240.0;

    private static final double SLOT0_KS = 0.0; // TODO - tune
    private static final double SLOT0_KV = 0.0; // TODO - tune
    private static final double SLOT0_KP = 0.25; // TODO - tune
    private static final double SLOT0_KI = 0.0; // TODO - tune
    private static final double SLOT0_KD = 0.0; // TODO - tune

    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;
    private static final double SPEED_TOLERANCE_RPS = 0.5;

    private static final InvertedValue LEFT_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    private static final InvertedValue RIGHT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

    private final TalonFX shooterLeftMotor;
    private final TalonFX shooterRightMotor;
    private final MotionMagicVelocityVoltage motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0);

    private double currentSpeedSetpointRps = 0.0;

    public Shooter() {
        shooterLeftMotor = new TalonFX(Constants.CAN_ID.SHOOTER_LEFT, Constants.CAN_BUS.RIO);
        shooterRightMotor = new TalonFX(Constants.CAN_ID.SHOOTER_RIGHT, Constants.CAN_BUS.RIO);
        configureMotor(shooterLeftMotor, LEFT_MOTOR_INVERTED, "left shooter");
        configureMotor(shooterRightMotor, RIGHT_MOTOR_INVERTED, "right shooter");
    }

    public void setSpeed(double speedRps) {
        currentSpeedSetpointRps = Math.max(0.0, speedRps);
        shooterLeftMotor.setControl(motionMagicVelocityRequest.withVelocity(currentSpeedSetpointRps));
        shooterRightMotor.setControl(motionMagicVelocityRequest.withVelocity(currentSpeedSetpointRps));
        SmartDashboard.putNumber("Shooter Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    public void setTargetSpeedRps(double speedRps) {
        setSpeed(speedRps);
    }

    public double getSpeed() {
        return (shooterLeftMotor.getVelocity().getValueAsDouble() + shooterRightMotor.getVelocity().getValueAsDouble()) / 2.0;
    }

    public double getCurrentSpeedRps() {
        return getSpeed();
    }

    public double getSpeedSetpoint() {
        return currentSpeedSetpointRps;
    }

    public boolean isAtSpeed() {
        return isAtSpeed(SPEED_TOLERANCE_RPS);
    }

    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= toleranceRps;
    }

    public void stop() {
        currentSpeedSetpointRps = 0.0;
        shooterLeftMotor.stopMotor();
        shooterRightMotor.stopMotor();
        SmartDashboard.putNumber("Shooter Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current Speed RPS", getSpeed());
    }

    private void configureMotor(TalonFX motor, InvertedValue invertedValue, String motorName) {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2)))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT0_KS)
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));

        shooterConfigs.MotorOutput.Inverted = invertedValue;
        shooterConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = motor.getConfigurator().apply(shooterConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for " + motorName + ", error code: " + status);
        }
    }
}
