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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase 
{
    private static final int CONFIG_RETRIES = 5;

    private static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 22/24;
    private static final double MM_CRUISE_RPS = 120;
    private static final double MM_ACCEL_RPS2 = 5;

    private static final double SLOT0_KS = 0; // TODO - tune
    private static final double SLOT0_KV = 0.0; // TODO - tune
    private static final double SLOT0_KP = .55; // TODO - tune
    private static final double SLOT0_KI = 0.0; // TODO - tune
    private static final double SLOT0_KD = 0.0; // TODO - tune

    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private static final InvertedValue LEFT_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
    private static final InvertedValue RIGHT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
    private static final MotorAlignmentValue RIGHT_FOLLOW_ALIGNMENT = MotorAlignmentValue.Opposed;

    private final TalonFX shooterLeftMotor;
    private final TalonFX shooterRightMotor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private double currentSpeedSetpointRps = 0.0;
    private double lastKs = SLOT0_KS;
    private double lastKv = SLOT0_KV;
    private double lastKp = SLOT0_KP;
    private double lastKi = SLOT0_KI;
    private double lastKd = SLOT0_KD;

    public Shooter() {
        shooterLeftMotor = new TalonFX(Constants.CAN_ID.SHOOTER_LEFT, Constants.CAN_BUS.CANIVORE);
        shooterRightMotor = new TalonFX(Constants.CAN_ID.SHOOTER_RIGHT, Constants.CAN_BUS.CANIVORE);
        configureMotor(shooterLeftMotor, LEFT_MOTOR_INVERTED, "left shooter");
        configureMotor(shooterRightMotor, RIGHT_MOTOR_INVERTED, "right shooter");
        shooterRightMotor.setControl(new Follower(Constants.CAN_ID.SHOOTER_LEFT, RIGHT_FOLLOW_ALIGNMENT));
        putPidDefaults();
    }

    public void setSpeed(double speedRps) {
        currentSpeedSetpointRps = Math.max(0.0, speedRps);
        shooterLeftMotor.setControl(velocityRequest.withVelocity(currentSpeedSetpointRps));
        shooterRightMotor.setControl(new Follower(Constants.CAN_ID.SHOOTER_LEFT, RIGHT_FOLLOW_ALIGNMENT));
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

    public boolean isAtSpeed() 
    {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= Constants.Shooter.SPEED_TOLERANCE_RPS;
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
        updatePidFromDashboard();
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
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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

    private void putPidDefaults() {
        SmartDashboard.putBoolean("Shooter/PID/LiveEnable", false);
        SmartDashboard.putNumber("Shooter/PID/kS", SLOT0_KS);
        SmartDashboard.putNumber("Shooter/PID/kV", SLOT0_KV);
        SmartDashboard.putNumber("Shooter/PID/kP", SLOT0_KP);
        SmartDashboard.putNumber("Shooter/PID/kI", SLOT0_KI);
        SmartDashboard.putNumber("Shooter/PID/kD", SLOT0_KD);
    }

    private void updatePidFromDashboard() {
        if (!SmartDashboard.getBoolean("Shooter/PID/LiveEnable", false)) {
            return;
        }
        double ks = SmartDashboard.getNumber("Shooter/PID/kS", SLOT0_KS);
        double kv = SmartDashboard.getNumber("Shooter/PID/kV", SLOT0_KV);
        double kp = SmartDashboard.getNumber("Shooter/PID/kP", SLOT0_KP);
        double ki = SmartDashboard.getNumber("Shooter/PID/kI", SLOT0_KI);
        double kd = SmartDashboard.getNumber("Shooter/PID/kD", SLOT0_KD);

        if (ks == lastKs && kv == lastKv && kp == lastKp && ki == lastKi && kd == lastKd) {
            return;
        }

        var slot0 = new Slot0Configs()
                .withKS(ks)
                .withKV(kv)
                .withKP(kp)
                .withKI(ki)
                .withKD(kd);
        StatusCode leftStatus = shooterLeftMotor.getConfigurator().apply(slot0);
        StatusCode rightStatus = shooterRightMotor.getConfigurator().apply(slot0);
        if (leftStatus.isOK() && rightStatus.isOK()) {
            lastKs = ks;
            lastKv = kv;
            lastKp = kp;
            lastKi = ki;
            lastKd = kd;
        }
    }
    public void printDiagnostics() {
        SmartDashboard.putNumber("Shooter Current Speed RPS", getSpeed());
        SmartDashboard.putNumber("Shooter Speed Setpoint RPS", currentSpeedSetpointRps);
        SmartDashboard.putBoolean("Shooter Is At Speed", isAtSpeed());
        SmartDashboard.putNumber("Shooter Current", shooterLeftMotor.getSupplyCurrent().getValueAsDouble());

    }
}
