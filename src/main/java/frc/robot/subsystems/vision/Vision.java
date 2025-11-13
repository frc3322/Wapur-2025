package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final NetworkTable visionTable;

  public Vision() {
    visionTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kVisionTableName);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("EagleEye/HasTarget", getTargetPose().isPresent());
    if (getTargetPose().isPresent()) {
      Logger.recordOutput("EagleEye/TargetPose", getTargetPose().get());
    }
  }

  /**
   * Gets the target position from NetworkTables and converts it to a Pose3d.
   *
   * @return Optional containing the target Pose3d if data is available, empty otherwise
   */
  public Optional<Pose3d> getTargetPose() {
    var positionEntry = visionTable.getEntry(VisionConstants.kTargetPositionEntryName);
    double[] positionArray = positionEntry.getDoubleArray(new double[0]);

    if (positionArray.length >= 3) {
      // Convert Vector3 (x, y, z) to Pose3d with identity rotation
      Translation3d translation =
          new Translation3d(positionArray[0], positionArray[1], positionArray[2]);
      Rotation3d rotation = new Rotation3d(); // Identity rotation
      Pose3d targetPose = new Pose3d(translation, rotation);

      // Log vision data
      Logger.recordOutput("EagleEye/HasTarget", true);
      Logger.recordOutput("EagleEye/TargetPose", targetPose);
      Logger.recordOutput("EagleEye/TargetPosition", translation);

      return Optional.of(targetPose);
    } else {
      // No valid data available
      Logger.recordOutput("EagleEye/HasTarget", false);
      return Optional.empty();
    }
  }
}
