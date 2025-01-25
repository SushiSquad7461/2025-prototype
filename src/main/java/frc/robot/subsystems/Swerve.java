// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import SushiFrcLib.Sensors.gyro.Pigeon;
import SushiFrcLib.SmartDashboard.TunableNumber;
import SushiFrcLib.Swerve.SwerveModules.SwerveModuleNeo;
import SushiFrcLib.Swerve.SwerveTemplates.VisionBaseSwerve;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController rotationLockPID;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator camFilter;
    private final TunableNumber maxDistanceCamToTarget;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

    private Swerve() {
        super(
          new SwerveModuleNeo[] { // pass these in same order as they are enumerated in the kinematics in the constants
            new SwerveModuleNeo(Constants.Swerve.SWERVE_MODULE_CONSTANTS[0]),
            new SwerveModuleNeo(Constants.Swerve.SWERVE_MODULE_CONSTANTS[1]),
            new SwerveModuleNeo(Constants.Swerve.SWERVE_MODULE_CONSTANTS[2]),
            new SwerveModuleNeo(Constants.Swerve.SWERVE_MODULE_CONSTANTS[3]),
          },
          new Pigeon(Constants.Ports.PIGEON_ID, Constants.Swerve.GYRO_INVERSION),
          Constants.Swerve.SWERVE_KINEMATICS);

          
        locationLock = false;
        rotationLockPID = Constants.Swerve.autoRotate.getPIDController();

        camera = new PhotonCamera("2025");
        camFilter = new PhotonPoseEstimator(
                AprilTagFields.k2025Reefscape.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());
        maxDistanceCamToTarget = new TunableNumber("Max Cam->Target (m)", 6, true);
    }

    public void enableRotationLock(double angle) {
        locationLock = true;

        rotationLockPID.setSetpoint(angle);
        rotationLockPID.calculate(getGyro().getAngle().getDegrees());
    }

    public Command rotateCommand(double angle) {
        return enableRotationLockCommand(angle).until(() -> {
            return Math.abs(getGyro().getAngle().getDegrees() - angle) < 2;
        }).andThen(disableRotationLockCommand());
    }

    public Command enableRotationLockCommand(double angle) {
        return runOnce(() -> enableRotationLock(angle));
    }

    public Command disableRotationLockCommand() {
        return runOnce(() -> disableRotationLock());
    }

    public void disableRotationLock() {
        locationLock = false;
    }

    @Override
    public void drive(Translation2d translation, double rotation) {
        if (locationLock) {
            rotation = rotationLockPID.calculate(getGyro().getAngle().getDegrees());
        }

        super.drive(translation, rotation);
        SmartDashboard.putNumber("rotation", rotation);
    }

    public Command resetHeading() {
        return runOnce(() -> setOdomPose(
            new Pose2d(
                getOdomPose().getTranslation(), 
                new Rotation2d()
            )
        ));
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!camera.isConnected())
            return;
        
        List<PhotonPipelineResult> camResults = camera.getAllUnreadResults();
        PhotonPipelineResult camPhotonPipelineResult = camResults.get(camResults.size()-1); //set idx to wanted target
        
        var pose = camFilter.update(camPhotonPipelineResult);

        if (!pose.isPresent() || pose.get().targetsUsed.size() < 2)
            return;

        var targetsCloseEnough = true;
        for (var target : pose.get().targetsUsed) {
            var transform = target.getBestCameraToTarget();
            double cameraToTagDistance = new Pose3d().transformBy(transform).getTranslation().getNorm();
            if (cameraToTagDistance > maxDistanceCamToTarget.get()) {
                targetsCloseEnough = false;
                break;
            }
        }

        if (targetsCloseEnough) {
            super.addVisionTargets(List.of(pose.get()));
        }

    }
}