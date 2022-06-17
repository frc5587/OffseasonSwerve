package swerveLib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/** A swerve drive drivetrain. */
public class SwerveBase extends SubsystemBase {
    private final AHRS navX = new AHRS();

    /**
     * A constants object that provides everything needed by {@link SwerveBase}.
     */
    public static class SwerveConstants {
        public final int[][] motors;
        public final double modulePos, wheelRadius, maxAngularVelocity, maxAngularAcceleration;
        public final PIDController drivePidController;
        public final ProfiledPIDController turningPidController;
        public final SimpleMotorFeedforward driveFeedforward, turningFeedforward;

        /**
         * A constants object that provides everything needed by {@link SwerveBase}.
         * 
         * @param motors - A 2D array of motor IDs for the swerve modules. 
         * @param modulePos - The position of the swerve modules.
         * @param wheelRadius - The radius of the swerve wheel in meters.
         * @param maxAngularVelocity - Maximum angular velocity (should be {@see Math#PI}).
         * @param maxAngularAcceleration - Maximum angular acceleration (should be {@see Math#PI} * 2).
         * @param drivePidController - A PIDController for drive motors.
         * @param turningPidController - A ProfiledPIDController for turning motors.
         * @param driveFeedforward - A SimpleMotorFeedforward for drive motors.
         * @param turningFeedforward - A SimpleMotorFeedforward for turning motors.
         */
        public SwerveConstants(int[][] motors, double modulePos, double wheelRadius, double maxAngularVelocity, double maxAngularAcceleration, PIDController drivePidController, ProfiledPIDController turningPidController, SimpleMotorFeedforward driveFeedforward, SimpleMotorFeedforward turningFeedforward) {
            this.motors = motors;
            this.modulePos = modulePos;
            this.wheelRadius = wheelRadius;
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.drivePidController = drivePidController;
            this.turningPidController = turningPidController;
            this.driveFeedforward = driveFeedforward;
            this.turningFeedforward = turningFeedforward;
        }
    }

    protected SwerveConstants swerveConstants;

    protected final FalconSwerveModule frontLeft = new FalconSwerveModule(swerveConstants.motors[0][0], swerveConstants.motors[0][1], swerveConstants);
    protected final FalconSwerveModule frontRight = new FalconSwerveModule(swerveConstants.motors[1][0], swerveConstants.motors[1][1], swerveConstants);
    protected final FalconSwerveModule backLeft = new FalconSwerveModule(swerveConstants.motors[2][0], swerveConstants.motors[2][1], swerveConstants);
    protected final FalconSwerveModule backRight = new FalconSwerveModule(swerveConstants.motors[3][0], swerveConstants.motors[3][1], swerveConstants);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(swerveConstants.modulePos, swerveConstants.modulePos), new Translation2d(swerveConstants.modulePos, -swerveConstants.modulePos), new Translation2d(-swerveConstants.modulePos, swerveConstants.modulePos), new Translation2d(-swerveConstants.modulePos, -swerveConstants.modulePos));

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, navX.getRotation2d());

    public SwerveBase(SwerveConstants swerveConstants) {
        this.swerveConstants = swerveConstants;

        navX.reset();
    }

    /**
     * Stop the drivetrain motors.
     */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Field-relative optional joystick drive for a swerve drive platform.
     * 
     * @param xSpeed - The robot's speed along the X axis (forward) [-1.0..1.0].
     * @param ySpeed - The robot's speed along the Y axis (sideways) [-1.0..1.0].
     * @param omega - Angular rate of the robot.
     * @param fieldRelative - Whether the provided X and Y speeds are relative to the field.
     */
    public void swerveDrive(double xSpeed, double ySpeed, double omega, boolean fieldRelative) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, navX.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, omega));

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Field-relative optional joystick drive for a swerve drive platform with a maximum velocity.
     * 
     * @param xSpeed - The robot's speed along the X axis (forward) [-1.0..1.0].
     * @param ySpeed - The robot's speed along the Y axis (sideways) [-1.0..1.0].
     * @param omega - Angular rate of the robot.
     * @param fieldRelative - Whether the provided X and Y speeds are relative to the field.
     * @param maximumVelocity - Maximum velocity the module can reach.
     */
    public void swerveDrive(double xSpeed, double ySpeed, double omega, boolean fieldRelative, double maximumVelocity) {
        var swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, navX.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, omega));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maximumVelocity);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        odometry.update(
            navX.getRotation2d(),
            frontLeft.getState(),
            frontRight.getState(), 
            backLeft.getState(),
            backRight.getState()
        );
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
