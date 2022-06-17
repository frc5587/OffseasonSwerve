// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swerveLib.SwerveBase.SwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>All measurements should be in meters. Using {@link edu.wpi.first.math.util.Units#inchesToMeters(double)} is permitted.
 */
public final class Constants {
    private static int[][] motorIds = {
            { 10, 15 },
            { 11, 16 },
            { 12, 17 },
            { 13, 18 },
    };

    public static SwerveConstants SWERVE_CONSTANTS = new SwerveConstants(motorIds, 0, Units.inchesToMeters(1.5 / 2),
            Math.PI,
            Math.PI * 2,
            new PIDController(0, 0, 0), new ProfiledPIDController(0, 0, 0,
                    new TrapezoidProfile.Constraints(
                            Math.PI, Math.PI * 2)),
            new SimpleMotorFeedforward(0, 0), new SimpleMotorFeedforward(0, 0));
}
