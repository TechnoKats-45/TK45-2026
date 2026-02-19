package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase
{
    private static final int CONFIG_RETRIES = 5;
    private static final double STATOR_CURRENT_LIMIT_AMPS = 80.0;
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
    private static final double MM_CRUISE_RPS = 8.0;
    private static final double MM_ACCEL_RPS2 = 12.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 12; // TODO: Check this - Sport Gearbox = 12:1
    private static final double SLOT0_KS = 0.0;
    private static final double SLOT0_KV = 0.0;
    private static final double SLOT0_KP = 10.0;
    private static final double SLOT0_KI = 0.0;
    private static final double SLOT0_KD = 1.0;
    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private final TalonFX climberMotor;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private double currentHeightSetpointInches = 0.0;

    public Climber() 
    {
        climberMotor = new TalonFX(Constants.CAN_ID.CLIMBER);
        configureClimberMotor();
    }

    public void zeroEncoder() {
        climberMotor.setPosition(0.0);
        currentHeightSetpointInches = 0.0;
    }

    public void setHeightInches(double heightInches) 
    {
        double clampedHeight = MathUtil.clamp(
                heightInches,
                Constants.Climber.MIN_HEIGHT_INCHES,
                Constants.Climber.MAX_HEIGHT_INCHES);
        currentHeightSetpointInches = clampedHeight;
        setPositionRotations(heightInchesToRotations(clampedHeight));
    }

    public void stop() 
    {
        climberMotor.stopMotor();
    }

    public boolean isAtHeight() 
    {
        return Math.abs(getHeightInches() - currentHeightSetpointInches) <= Constants.Climber.HEIGHT_TOLERANCE_INCHES;
    }

    public double getHeightInches() 
    {
        return rotationsToHeightInches(climberMotor.getPosition().getValueAsDouble());
    }

    private void setPositionRotations(double rotations) {
        climberMotor.setControl(motionMagicVoltage.withPosition(rotations));
    }

    private static double heightInchesToRotations(double heightInches) {
        return heightInches * Constants.Climber.ROTATIONS_PER_INCH;
    }

    private static double rotationsToHeightInches(double rotations) {
        return rotations / Constants.Climber.ROTATIONS_PER_INCH;
    }

    private void configureClimberMotor() {
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2)));

        climberConfigs.Slot0.kS = SLOT0_KS;
        climberConfigs.Slot0.kV = SLOT0_KV;
        climberConfigs.Slot0.kP = SLOT0_KP;
        climberConfigs.Slot0.kI = SLOT0_KI;
        climberConfigs.Slot0.kD = SLOT0_KD;
        climberConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = climberMotor.getConfigurator().apply(climberConfigs);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply climber configs, error code: " + status);
        }
    }
}
