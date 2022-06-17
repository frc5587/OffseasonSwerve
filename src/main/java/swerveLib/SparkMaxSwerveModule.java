package swerveLib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerveLib.SwerveBase.SwerveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxSwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SwerveConstants swerveConstants;

    /**
     * Constructs a SwerveModule for Spark MAX motor controllers with a drive motor & turning motor.
     * 
     * @param driveMotorId - CAN ID for the drive motor.
     * @param turningMotorId - CAN ID for the turning motor.
     */
    public SparkMaxSwerveModule(int driveMotorId, int turningMotorId, SwerveConstants swerveConstants) {
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        this.swerveConstants = swerveConstants;

        swerveConstants.turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the curret state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
    }

    /**
     * Sets the desired state for the module.
     * 
     * @param desiredState - Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getPosition()));

        final double driveOutput = swerveConstants.drivePidController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = swerveConstants.driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turningOutput = swerveConstants.turningPidController.calculate(turningEncoder.getVelocity(), state.angle.getRadians());
        final double turningFeedforward = swerveConstants.turningFeedforward.calculate(swerveConstants.turningPidController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turningOutput + turningFeedforward);
    }

    /**
     * Set velocity of the drive & turning motors to 0.
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}