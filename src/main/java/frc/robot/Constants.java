// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;
import SushiFrcLib.Control.PIDConfig;
import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Swerve.SwerveConstants.SDSModules;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final boolean REDUCE_SPEED = true;
    public static final double LOW_SPEED = 2;
    public static final double LOW_ROT = Math.PI;
    public static final int CAMERA_RESOLUTIONX = 1280;
    public static final int CAMERA_RESOLUTIONY = 800;


    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(23);
    public static final double WHEEL_BASE = Units.inchesToMeters(23);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE) / 2;

    // these need to be in module order
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), //0
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), //1
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), //2
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0) //3
    );

    // Updated PID values for NEO motors - these will need tuning
    public static final MotorConfig ANGLE_CONFIG = new MotorConfig(
        20, // Current limit
        false,
        PIDConfig.getPid(0.03), // P value for position control
        MotorConfig.Mode.COAST
    );

    public static final MotorConfig ANGLE_FLIPPED_CONFIG = new MotorConfig(
        20,
        true,
        PIDConfig.getPid(0.03),
        MotorConfig.Mode.COAST
    );

    public static final MotorConfig DRIVE_CONFIG = new MotorConfig(
        40,
        true,
        PIDConfig.getPid(0.15, 0, 0., 0), // P and F values for velocity                                                                                             
        MotorConfig.Mode.BRAKE
    );

    public static final MotorConfig DRIVE_FLIPPED_CONFIG = new MotorConfig(
        40,
        false,
        PIDConfig.getPid(0.05, 0),
        MotorConfig.Mode.BRAKE
    );

    public static final PIDConfig autoRotate = PIDConfig.getPid(0.008, 0.00, 0.00);
    public static final SDSModules MODULE_TYPE = SDSModules.MK4i;
    public static final boolean SWERVE_TUNING_MODE = true;

    public static final SwerveModuleConstants[] SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants[] {
      new SwerveModuleConstants(0, Rotation2d.fromDegrees(310.078125), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
      new SwerveModuleConstants(1, Rotation2d.fromDegrees(240.820312), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
      new SwerveModuleConstants(2, Rotation2d.fromDegrees(298.388672), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG),
      new SwerveModuleConstants(3, Rotation2d.fromDegrees(162.421875), MODULE_TYPE,
          SWERVE_TUNING_MODE, DRIVE_FLIPPED_CONFIG, ANGLE_FLIPPED_CONFIG)
    };
  }


}
