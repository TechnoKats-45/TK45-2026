package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindex extends SubsystemBase {
    private static final int CONFIG_RETRIES = 5;

    private static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 1.0; // TODO: check gear ratio, should be 1:1

    private static final double SLOT0_KS = 0.0; // TODO - tune
    private static final double SLOT0_KV = 0.0; // TODO - tune
    private static final double SLOT0_KP = 0.2; // TODO - tune
    private static final double SLOT0_KI = 0.0; // TODO - tune
    private static final double SLOT0_KD = 0.0; // TODO - tune

    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private static final double SPEED_TOLERANCE_RPS = 0.5;
    private static final double MAX_SPINDEX_SPEED_RPS = 40.0; // TODO - tune
    private static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

    private final TalonFX spindexMotor;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private double currentSpeedSetpointRps = 0.0;

    public Spindex() {
        spindexMotor = new TalonFX(Constants.CAN_ID.SPINDEXER);
        configureMotor();
    }

    public void setSpeed(double speedRps) {
        currentSpeedSetpointRps = speedRps;
        spindexMotor.setControl(velocityRequest.withVelocity(currentSpeedSetpointRps));
        SmartDashboard.putNumber("Spindex Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    // Compatibility with existing callsites that currently use percent output.
    public void runFeed(double percentOutput) {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        setSpeed(clamped * MAX_SPINDEX_SPEED_RPS);
    }

    public double getSpeed() {
        return spindexMotor.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed() {
        return isAtSpeed(SPEED_TOLERANCE_RPS);
    }

    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= toleranceRps;
    }

    public void stop() {
        currentSpeedSetpointRps = 0.0;
        spindexMotor.stopMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration spindexConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withSlot0(new Slot0Configs()
                        .withKS(SLOT0_KS)
                        .withKV(SLOT0_KV)
                        .withKP(SLOT0_KP)
                        .withKI(SLOT0_KI)
                        .withKD(SLOT0_KD));

        spindexConfigs.MotorOutput.Inverted = MOTOR_INVERTED;
        spindexConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = spindexMotor.getConfigurator().apply(spindexConfigs);
            if (status.isOK()) {
                break;
            }
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs for spindex, error code: " + status);
        }
    }
}
