// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;
import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Swerve.SwerveConstants.SDSModules;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Ports {
    public static final String CANIVORE_NAME = "Sussy Squad";
    public static final int PIGEON_ID = 13;
  }

  public static final class Swerve {
    public static final boolean GYRO_INVERSION = false;

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(23);
    public static final double WHEEL_BASE = Units.inchesToMeters(23);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)
    );

    // Updated PID values for NEO motors - these will need tuning
    public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
        20, // Current limit
        false,
        PIDConfig.getPid(0.3), // P value for position control
        MotorConfig.Mode.COAST
    );

    public static final MotorConfig ANGLE_FLIPPED_CONFIG = new MotorConfig(
        20,
        true,
        PIDConfig.getPid(0.3),
        MotorConfig.Mode.COAST
    );

    public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
        40,
        true,
        PIDConfig.getPid(0.1, 0.5), // P and F values for velocity control
        MotorConfig.Mode.BRAKE
    );

    public static final MotorConfig DRIVE_FLIPPED_CONFIG = new MotorConfig(
        40,
        false,
        PIDConfig.getPid(0.1, 0.5),
        MotorConfig.Mode.BRAKE
    );

    public static final PIDConfig autoRotate = PIDConfig.getPid(0.008, 0.00, 0.00);
    public static final SDSModules MODULE_TYPE = SDSModules.MK4i;
    public static final boolean SWERVE_TUNING_MODE = false;

    public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants[] {
      new SwerveModuleConstants(0, Rotation2d.fromDegrees(0.461182 * 360), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_CONFIG, ANGLE_CONFIG),
      new SwerveModuleConstants(1, Rotation2d.fromDegrees(0.561035 * 360), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
      new SwerveModuleConstants(2, Rotation2d.fromDegrees(0.570557 * 360), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_CONFIG, ANGLE_CONFIG),
      new SwerveModuleConstants(3, Rotation2d.fromDegrees(0.336426 * 360 + 180), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG)
    };
  }


}
