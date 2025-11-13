package frc.robot.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

/**
 * Decodes Flatpack binary serialization format used by EagleEye vision system. Supports
 * float_array, vector2_array, and vector3_array schemas.
 */
public class FlatpackDecoder {
  private static final byte[] MAGIC = {0x46, 0x50, 0x4B, 0x31}; // "FPK1"
  private static final int MAX_ARRAY_SIZE = 10000;

  /** 2D vector representation for vision data. */
  public static class Vector2 {
    public final float x;
    public final float y;

    public Vector2(float x, float y) {
      this.x = x;
      this.y = y;
    }

    @Override
    public String toString() {
      return String.format("Vector2{x=%.3f, y=%.3f}", x, y);
    }
  }

  /** 3D vector representation for vision data. */
  public static class Vector3 {
    public final float x;
    public final float y;
    public final float z;

    public Vector3(float x, float y, float z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }

    @Override
    public String toString() {
      return String.format("Vector3{x=%.3f, y=%.3f, z=%.3f}", x, y, z);
    }
  }

  /** Decoded packet containing schema name and payload buffer. */
  public static class DecodedPacket {
    public final String schemaName;
    public final ByteBuffer payload;

    public DecodedPacket(String schemaName, ByteBuffer payload) {
      this.schemaName = schemaName;
      this.payload = payload;
    }
  }

  /**
   * Decodes the header of a Flatpack packet and returns the schema name and payload buffer.
   *
   * @param rawData Raw binary data from NetworkTables
   * @return DecodedPacket containing schema name and payload buffer
   * @throws IllegalArgumentException if magic number is invalid or data is malformed
   */
  public static DecodedPacket decodeHeader(byte[] rawData) throws IllegalArgumentException {
    if (rawData.length < 5) {
      throw new IllegalArgumentException("Packet too short for Flatpack header");
    }

    ByteBuffer buffer = ByteBuffer.wrap(rawData);

    // Validate magic number
    for (int i = 0; i < 4; i++) {
      if (buffer.get() != MAGIC[i]) {
        throw new IllegalArgumentException("Invalid FPK1 magic number");
      }
    }

    // Read schema name length
    int schemaNameLength = buffer.get() & 0xFF; // Unsigned byte

    if (buffer.remaining() < schemaNameLength) {
      throw new IllegalArgumentException("Insufficient data for schema name");
    }

    // Read schema name
    byte[] schemaNameBytes = new byte[schemaNameLength];
    buffer.get(schemaNameBytes);
    String schemaName = new String(schemaNameBytes, StandardCharsets.UTF_8);

    // Remaining bytes are payload
    byte[] payloadBytes = new byte[buffer.remaining()];
    buffer.get(payloadBytes);
    ByteBuffer payload = ByteBuffer.wrap(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);

    return new DecodedPacket(schemaName, payload);
  }

  /**
   * Decodes a complete Flatpack packet into the appropriate Java object.
   *
   * @param rawData Raw binary data from NetworkTables
   * @return Decoded object (float[], Vector2[], or Vector3[])
   * @throws IllegalArgumentException if packet is malformed or schema is unknown
   */
  public static Object decode(byte[] rawData) throws IllegalArgumentException {
    DecodedPacket packet = decodeHeader(rawData);

    switch (packet.schemaName) {
      case "float_array":
        return decodeFloatArray(packet.payload);
      case "vector2_array":
        return decodeVector2Array(packet.payload);
      case "vector3_array":
        return decodeVector3Array(packet.payload);
      default:
        throw new IllegalArgumentException("Unknown schema: " + packet.schemaName);
    }
  }

  /** Decodes a float array from the payload buffer. */
  private static float[] decodeFloatArray(ByteBuffer buffer) throws IllegalArgumentException {
    if (buffer.remaining() < 4) {
      throw new IllegalArgumentException("Insufficient data for float array count");
    }

    int count = buffer.order(ByteOrder.LITTLE_ENDIAN).getInt();

    if (count < 0 || count > MAX_ARRAY_SIZE) {
      throw new IllegalArgumentException("Invalid array count: " + count);
    }

    if (buffer.remaining() < count * 4) {
      throw new IllegalArgumentException("Insufficient data for float array elements");
    }

    float[] result = new float[count];
    for (int i = 0; i < count; i++) {
      result[i] = buffer.getFloat();
    }
    return result;
  }

  /** Decodes a Vector2 array from the payload buffer. */
  private static Vector2[] decodeVector2Array(ByteBuffer buffer) throws IllegalArgumentException {
    if (buffer.remaining() < 4) {
      throw new IllegalArgumentException("Insufficient data for vector2 array count");
    }

    int count = buffer.order(ByteOrder.LITTLE_ENDIAN).getInt();

    if (count < 0 || count > MAX_ARRAY_SIZE) {
      throw new IllegalArgumentException("Invalid array count: " + count);
    }

    if (buffer.remaining() < count * 8) {
      throw new IllegalArgumentException("Insufficient data for vector2 array elements");
    }

    Vector2[] result = new Vector2[count];
    for (int i = 0; i < count; i++) {
      float x = buffer.getFloat();
      float y = buffer.getFloat();
      result[i] = new Vector2(x, y);
    }
    return result;
  }

  /** Decodes a Vector3 array from the payload buffer. */
  private static Vector3[] decodeVector3Array(ByteBuffer buffer) throws IllegalArgumentException {
    if (buffer.remaining() < 4) {
      throw new IllegalArgumentException("Insufficient data for vector3 array count");
    }

    int count = buffer.order(ByteOrder.LITTLE_ENDIAN).getInt();

    if (count < 0 || count > MAX_ARRAY_SIZE) {
      throw new IllegalArgumentException("Invalid array count: " + count);
    }

    if (buffer.remaining() < count * 12) {
      throw new IllegalArgumentException("Insufficient data for vector3 array elements");
    }

    Vector3[] result = new Vector3[count];
    for (int i = 0; i < count; i++) {
      float x = buffer.getFloat();
      float y = buffer.getFloat();
      float z = buffer.getFloat();
      result[i] = new Vector3(x, y, z);
    }
    return result;
  }
}
