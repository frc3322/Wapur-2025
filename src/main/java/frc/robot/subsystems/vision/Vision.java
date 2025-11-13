package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FlatpackDecoder;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final NetworkTable visionTable;

  public Vision() {
    visionTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kVisionTableName);
  }

  @Override
  public void periodic() {
    Optional<Pose3d> targetPose = getTargetPose();
    targetPose.ifPresent(pose -> Logger.recordOutput("EagleEye/TargetPose", pose));
  }

  /**
   * Gets the target position from NetworkTables and converts it to a Pose3d.
   *
   * @return Optional containing the target Pose3d if data is available, empty otherwise
   */
  public Optional<Pose3d> getTargetPose() {
    var positionEntry = visionTable.getEntry(VisionConstants.kTargetPositionEntryName);
    byte[] rawPacket = positionEntry.getRaw(new byte[0]);

    if (rawPacket.length == 0) {
      Logger.recordOutput("EagleEye/HasTarget", false);
      return Optional.empty();
    }

    try {
      Object decodedPayload = FlatpackDecoder.decode(rawPacket);
      Logger.recordOutput("EagleEye/FlatpackSchema", determineSchemaLabel(decodedPayload));
      Optional<Translation3d> translation = extractTranslation(decodedPayload);

      if (translation.isPresent()) {
        Pose3d targetPose = new Pose3d(translation.get(), new Rotation3d());
        Logger.recordOutput("EagleEye/HasTarget", true);
        Logger.recordOutput("EagleEye/TargetPosition", translation.get());
        return Optional.of(targetPose);
      }

      Logger.recordOutput("EagleEye/HasTarget", false);
      Logger.recordOutput("EagleEye/FlatpackError", "Unsupported payload structure");
      return Optional.empty();
    } catch (IllegalArgumentException exception) {
      Logger.recordOutput("EagleEye/HasTarget", false);
      Logger.recordOutput("EagleEye/FlatpackError", exception.getMessage());
      return Optional.empty();
    }
  }

  private Optional<Translation3d> extractTranslation(Object decodedPayload) {
    if (decodedPayload instanceof FlatpackDecoder.Vector3[] vectors && vectors.length > 0) {
      FlatpackDecoder.Vector3 vector = vectors[0];
      return Optional.of(new Translation3d(vector.x, vector.y, vector.z));
    }

    if (decodedPayload instanceof FlatpackDecoder.Vector2[] vectors && vectors.length > 0) {
      FlatpackDecoder.Vector2 vector = vectors[0];
      return Optional.of(new Translation3d(vector.x, vector.y, 0.0));
    }

    if (decodedPayload instanceof float[] values && values.length >= 3) {
      return Optional.of(new Translation3d(values[0], values[1], values[2]));
    }

    return Optional.empty();
  }

  private String determineSchemaLabel(Object decodedPayload) {
    if (decodedPayload instanceof FlatpackDecoder.Vector3[]) {
      return "vector3_array";
    }

    if (decodedPayload instanceof FlatpackDecoder.Vector2[]) {
      return "vector2_array";
    }

    if (decodedPayload instanceof float[]) {
      return "float_array";
    }

    return decodedPayload.getClass().getSimpleName();
  }
}
