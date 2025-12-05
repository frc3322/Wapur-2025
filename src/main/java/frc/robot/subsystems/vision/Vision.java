package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FlatpackDecoder;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem that processes target poses from NetworkTables using Flatpack decoding. */
public class Vision extends SubsystemBase {
  private static final String HAS_TARGET_KEY = "EagleEye/HasTarget";
  private final NetworkTable visionTable;

  /** Initializes vision subsystem with NetworkTables connection. */
  public Vision() {
    visionTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kVisionTableName);
  }

  /** Periodically processes target pose data and logs it with 90-degree rotation. */
  @Override
  public void periodic() {
    Optional<Pose2d> targetPose = getTargetPose();
    targetPose.ifPresent(
        pose ->
            Logger.recordOutput(
                "EagleEye/TargetPose",
                pose.rotateAround(pose.getTranslation(), new Rotation2d(Math.PI / 2))));
  }

  /**
   * Gets the target pose from NetworkTables and converts it to a Pose2d.
   *
   * @return Optional containing the target Pose2d if data is available, empty otherwise
   */
  public Optional<Pose2d> getTargetPose() {
    var positionEntry = visionTable.getEntry(VisionConstants.kTargetPositionEntryName);
    byte[] rawPacket = positionEntry.getRaw(new byte[0]);

    if (rawPacket.length == 0) {
      Logger.recordOutput(HAS_TARGET_KEY, false);
      return Optional.empty();
    }

    try {
      Object decodedPayload = FlatpackDecoder.decode(rawPacket);
      Logger.recordOutput("EagleEye/FlatpackSchema", determineSchemaLabel(decodedPayload));
      Optional<Pose2d> pose = extractPose(decodedPayload);

      if (pose.isPresent()) {
        Pose2d targetPose = pose.get();
        Logger.recordOutput(HAS_TARGET_KEY, true);
        return Optional.of(targetPose);
      }

      Logger.recordOutput(HAS_TARGET_KEY, false);
      Logger.recordOutput("EagleEye/FlatpackError", "Unsupported payload structure");
      return Optional.empty();
    } catch (IllegalArgumentException exception) {
      Logger.recordOutput(HAS_TARGET_KEY, false);
      Logger.recordOutput("EagleEye/FlatpackError", exception.getMessage());
      return Optional.empty();
    }
  }

  /** Attempts to extract Pose2d from decoded payload using multiple parsing strategies. */
  private Optional<Pose2d> extractPose(Object decodedPayload) {
    Optional<Pose2d> pose = poseFromConcretePayload(decodedPayload);
    if (pose.isPresent()) {
      return pose;
    }
    pose = poseFromGenericPayload(decodedPayload);
    if (pose.isPresent()) {
      return pose;
    }
    return poseFromFloatArray(decodedPayload);
  }

  /** Determines the schema type of the decoded payload for logging purposes. */
  private String determineSchemaLabel(Object decodedPayload) {
    if (decodedPayload instanceof FlatpackDecoder.Vector3[]) {
      return "vector3_array";
    }

    if (decodedPayload instanceof FlatpackDecoder.Vector2[]) {
      return "vector2_array";
    }

    if (decodedPayload instanceof FlatpackDecoder.Pose3D[]) {
      return "pose3d_array";
    }

    if (decodedPayload instanceof FlatpackDecoder.Pose2D[]) {
      return "pose2d_array";
    }

    if (decodedPayload instanceof float[]) {
      return "float_array";
    }

    if (decodedPayload instanceof FlatpackDecoder.GenericObjectArray genericArray) {
      return genericArray.descriptor.name + "_array";
    }

    if (decodedPayload instanceof FlatpackDecoder.GenericObject genericObject) {
      return genericObject.descriptor.name;
    }

    return decodedPayload.getClass().getSimpleName();
  }

  /** Extracts Pose2d from generic descriptor and float values by matching component names. */
  private Optional<Pose2d> poseFromDescriptor(
      FlatpackDecoder.SchemaDescriptor descriptor, float[] values) {
    Double xComponent = null;
    Double yComponent = null;
    Double rotationComponent = null;
    for (int index = 0; index < Math.min(descriptor.components.size(), values.length); index++) {
      String componentName = descriptor.components.get(index);
      double componentValue = values[index];
      if (componentName.equalsIgnoreCase("x")) {
        xComponent = componentValue;
      } else if (componentName.equalsIgnoreCase("y")) {
        yComponent = componentValue;
      } else if (componentName.equalsIgnoreCase("rotation")
          || componentName.equalsIgnoreCase("theta")
          || componentName.equalsIgnoreCase("yaw")) {
        rotationComponent = componentValue;
      }
    }
    if (xComponent == null || yComponent == null) {
      return Optional.empty();
    }
    double heading = rotationComponent == null ? 0.0 : rotationComponent;
    return Optional.of(new Pose2d(xComponent, yComponent, new Rotation2d(heading)));
  }

  /** Attempts to extract Pose2d from concrete FlatpackDecoder types (Pose2D, Pose3D, Vector2, Vector3). */
  private Optional<Pose2d> poseFromConcretePayload(Object payload) {
    if (payload instanceof FlatpackDecoder.Pose3D[] poses3d && poses3d.length > 0) {
      return Optional.of(poseFromPose3d(poses3d[0]));
    }
    if (payload instanceof FlatpackDecoder.Pose2D[] poses2d && poses2d.length > 0) {
      return Optional.of(poseFromPose2d(poses2d[0]));
    }
    if (payload instanceof FlatpackDecoder.Pose3D pose3d) {
      return Optional.of(poseFromPose3d(pose3d));
    }
    if (payload instanceof FlatpackDecoder.Pose2D pose2d) {
      return Optional.of(poseFromPose2d(pose2d));
    }
    if (payload instanceof FlatpackDecoder.Vector3[] vectors3 && vectors3.length > 0) {
      return Optional.of(poseFromVector(vectors3[0]));
    }
    if (payload instanceof FlatpackDecoder.Vector2[] vectors2 && vectors2.length > 0) {
      return Optional.of(poseFromVector(vectors2[0]));
    }
    if (payload instanceof FlatpackDecoder.Vector3 vector3) {
      return Optional.of(poseFromVector(vector3));
    }
    if (payload instanceof FlatpackDecoder.Vector2 vector2) {
      return Optional.of(poseFromVector(vector2));
    }
    return Optional.empty();
  }

  /** Converts Vector3 to Pose2d (x,y components only, no rotation). */
  private Pose2d poseFromVector(FlatpackDecoder.Vector3 vector) {
    return new Pose2d(vector.x, vector.y, new Rotation2d());
  }

  /** Converts Vector2 to Pose2d (x,y components only, no rotation). */
  private Pose2d poseFromVector(FlatpackDecoder.Vector2 vector) {
    return new Pose2d(vector.x, vector.y, new Rotation2d());
  }

  /** Converts Pose3D to Pose2d using x,y,yaw components. */
  private Pose2d poseFromPose3d(FlatpackDecoder.Pose3D pose) {
    return new Pose2d(pose.x, pose.y, new Rotation2d(pose.yaw));
  }

  /** Converts Pose2D to Pose2d directly. */
  private Pose2d poseFromPose2d(FlatpackDecoder.Pose2D pose) {
    return new Pose2d(pose.x, pose.y, new Rotation2d(pose.rotation));
  }

  /** Attempts to extract Pose2d from generic FlatpackDecoder objects using schema descriptors. */
  private Optional<Pose2d> poseFromGenericPayload(Object payload) {
    if (payload instanceof FlatpackDecoder.GenericObjectArray array && array.values.length > 0) {
      return poseFromDescriptor(array.descriptor, array.values[0]);
    }
    if (payload instanceof FlatpackDecoder.GenericObject object) {
      return poseFromDescriptor(object.descriptor, object.values);
    }
    return Optional.empty();
  }

  /** Attempts to extract Pose2d from raw float array [x, y, rotation]. */
  private Optional<Pose2d> poseFromFloatArray(Object payload) {
    if (payload instanceof float[] values && values.length >= 2) {
      double heading = values.length >= 3 ? values[2] : 0.0;
      return Optional.of(new Pose2d(values[0], values[1], new Rotation2d(heading)));
    }
    return Optional.empty();
  }
}
