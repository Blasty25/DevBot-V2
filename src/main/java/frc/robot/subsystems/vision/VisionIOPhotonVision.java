// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Latest Result
    var txResult = camera.getLatestResult();
    inputs.connected = txResult.hasTargets();

    List<Integer> tagIdsList = new ArrayList<>();
    List<Rotation2d> tagTxsList = new ArrayList<>();

    for (var target : txResult.getTargets()) {
      int tagId = target.getFiducialId();
      double yawDegrees = target.getYaw();

      tagIdsList.add(tagId);
      tagTxsList.add(Rotation2d.fromDegrees(yawDegrees));
    }

    // Save tag IDs and txs from latest result
    inputs.tagIds = tagIdsList.stream().mapToInt(Integer::intValue).toArray();
    inputs.txs = tagTxsList.toArray(new Rotation2d[0]);

    // Also update latest target observation for simple servoing
    if (txResult.hasTargets()) {
      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromDegrees(txResult.getBestTarget().getYaw()),
              Rotation2d.fromDegrees(txResult.getBestTarget().getPitch()));
    } else {
      inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    }

    // Pose Estimation Results
    Set<Short> poseTagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {

      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        poseTagIds.addAll(multitagResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                multitagResult.fiducialIDsUsed.size(),
                totalTagDistance / result.targets.size(),
                PoseObservationType.PHOTONVISION));
      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          poseTagIds.add((short) target.fiducialId);

          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm(),
                  PoseObservationType.PHOTONVISION));
        }
      }
    }

    // Save pose observations
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
  }
}
