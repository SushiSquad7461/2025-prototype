// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;

public class Swerve extends VisionBaseSwerve {
    private static Swerve instance;

    private boolean locationLock;
    private PIDController rotationLockPID;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator camFilter;
    private final TunableNumber maxDistanceCamToTarget;

    private final PIDController xController = new PIDController(0.1, 0, 0); //TODO: set these pid constants
    private final PIDController yController = new PIDController(0.1, 0, 0);
    private final PIDController rotationController = new PIDController(0.1, 0, 0);

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

        camera = new PhotonCamera("Arducam");
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
        if (locationLock && !isAligned()) { // Disable locationLock during autoAlign
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

    public Command getAutoAlignCommand() {
        return new RunCommand(
            () -> {
                PhotonPipelineResult result = camera.getLatestResult();
                
                if (result.hasTargets()) {
                    var bestTarget = result.getBestTarget();
                    
                    System.out.print("reached autoalign");
                    // target pose relative to the camera
                    double x = bestTarget.getBestCameraToTarget().getX();
                    double y = bestTarget.getBestCameraToTarget().getY();
                    double yaw = bestTarget.getYaw();
                    
                    double xSpeed = xController.calculate(x, 0);
                    double ySpeed = yController.calculate(y, 0);
                    double rotationSpeed = rotationController.calculate(yaw, 0);
                    
                    // drive with the calculated speeds
                    drive(
                        new Translation2d(xSpeed, ySpeed),
                        rotationSpeed
                    );
                }
            }
        ).until(this::isAligned).withTimeout(5); 
    }

    private boolean isAligned() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) return false;
        
        var target = result.getBestTarget();
        Transform3d targetTransform = target.getBestCameraToTarget();
        
        return Math.abs(targetTransform.getX()) < 0.1 && // TODO: set X and Y bounds
               Math.abs(targetTransform.getY()) < 0.1 &&
               Math.abs(target.getYaw()) < 5.0;
    }

    @Override
    public void periodic() {
        super.periodic();

    }
}