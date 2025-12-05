package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;

/**
 * Decodes Flatpack binary serialization format used by EagleEye vision system.
 * Supports
 * float_array, vector2, vector3, pose2d, pose3d, and their array variants.
 */
public class FlatpackDecoder {
    private static final byte[] MAGIC = { 0x46, 0x50, 0x4B, 0x31 }; // "FPK1"
    private static final byte[] MANIFEST_MAGIC = { 0x46, 0x50, 0x4B, 0x4D }; // "FPKM"
    private static final byte SUPPORTED_MANIFEST_VERSION = 1;
    private static final String VISION_TABLE_NAME = "EagleEye";
    private static final String MANIFEST_ENTRY_NAME = "schema_manifest";
    private static final int MAX_ARRAY_SIZE = 10000;
    private static final int VECTOR2_COMPONENTS = 2;
    private static final int VECTOR3_COMPONENTS = 3;
    private static final int POSE2D_COMPONENTS = 3;
    private static final int POSE3D_COMPONENTS = 6;
    private static final Map<String, SchemaDescriptor> schemaRegistry = new HashMap<>();
    private static volatile boolean manifestLoaded = false;
    private static final Object manifestLock = new Object();
    private static final Logger logger = Logger.getLogger(FlatpackDecoder.class.getName());
    private static final Set<String> loggedWarnings = Collections.synchronizedSet(new HashSet<>());

    public enum SchemaKind {
        OBJECT(0),
        OBJECT_ARRAY(1),
        FLOAT_ARRAY(2);

        private final int encodedValue;

        SchemaKind(int encodedValue) {
            this.encodedValue = encodedValue;
        }

        public static SchemaKind fromByte(int rawValue) {
            for (SchemaKind kind : values()) {
                if (kind.encodedValue == rawValue) {
                    return kind;
                }
            }
            logWarningOnce("Unsupported schema kind byte: " + rawValue);
            return null;
        }
    }

    public static class SchemaDescriptor {
        public final String name;
        public final SchemaKind kind;
        public final List<String> components;

        public SchemaDescriptor(String name, SchemaKind kind, List<String> components) {
            this.name = name;
            this.kind = kind;
            this.components = Collections.unmodifiableList(new ArrayList<>(components));
        }

        public int componentCount() {
            return components.size();
        }
    }

    public static class GenericObject {
        public final SchemaDescriptor descriptor;
        public final float[] values;

        public GenericObject(SchemaDescriptor descriptor, float[] values) {
            this.descriptor = descriptor;
            this.values = values;
        }
    }

    public static class GenericObjectArray {
        public final SchemaDescriptor descriptor;
        public final float[][] values;

        public GenericObjectArray(SchemaDescriptor descriptor, float[][] values) {
            this.descriptor = descriptor;
            this.values = values;
        }
    }

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

    /** 2D pose representation containing translation and rotation. */
    public static class Pose2D {
        public final float x;
        public final float y;
        public final float rotation;

        public Pose2D(float x, float y, float rotation) {
            this.x = x;
            this.y = y;
            this.rotation = rotation;
        }

        @Override
        public String toString() {
            return String.format("Pose2D{x=%.3f, y=%.3f, rotation=%.3f}", x, y, rotation);
        }
    }

    /**
     * 3D pose representation containing translation and rotation as Euler angles.
     */
    public static class Pose3D {
        public final float x;
        public final float y;
        public final float z;
        public final float roll;
        public final float pitch;
        public final float yaw;

        public Pose3D(float x, float y, float z, float roll, float pitch, float yaw) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }

        @Override
        public String toString() {
            return String.format(
                    "Pose3D{x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f}",
                    x, y, z, roll, pitch, yaw);
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
     * Logs a warning message only once to prevent log spam.
     *
     * @param message The warning message to log
     */
    private static void logWarningOnce(String message) {
        if (loggedWarnings.add(message)) {
            logger.warning(message);
        }
    }

    /**
     * Decodes the header of a Flatpack packet and returns the schema name and
     * payload buffer.
     *
     * @param rawData Raw binary data from NetworkTables
     * @return DecodedPacket containing schema name and payload buffer, or null if
     *         decoding fails
     */
    public static DecodedPacket decodeHeader(byte[] rawData) {
        if (rawData.length < 5) {
            logWarningOnce("Packet too short for Flatpack header");
            return null;
        }

        ByteBuffer buffer = ByteBuffer.wrap(rawData);

        // Validate magic number
        for (int i = 0; i < 4; i++) {
            if (buffer.get() != MAGIC[i]) {
                logWarningOnce("Invalid FPK1 magic number");
                return null;
            }
        }

        // Read schema name length
        int schemaNameLength = buffer.get() & 0xFF; // Unsigned byte

        if (buffer.remaining() < schemaNameLength) {
            logWarningOnce("Insufficient data for schema name");
            return null;
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
     * @return Decoded object (float[], Vector2[], or Vector3[]), or null if
     *         decoding fails
     */
    public static Object decode(byte[] rawData) {
        ensureManifestLoaded();
        DecodedPacket packet = decodeHeader(rawData);
        if (packet == null) {
            return null;
        }
        SchemaDescriptor descriptor = schemaRegistry.get(packet.schemaName);
        if (descriptor == null) {
            logWarningOnce("Schema not present in manifest: " + packet.schemaName);
            return null;
        }

        switch (descriptor.kind) {
            case FLOAT_ARRAY:
                return decodeFloatArray(packet.payload);
            case OBJECT:
                return decodeObject(packet.payload, descriptor);
            case OBJECT_ARRAY:
                return decodeObjectArray(packet.payload, descriptor);
            default:
                logWarningOnce("Unsupported schema kind for " + descriptor.name);
                return null;
        }
    }

    private static boolean validateRemaining(ByteBuffer buffer, int requiredBytes, String context) {
        if (buffer.remaining() < requiredBytes) {
            logWarningOnce("Insufficient data for " + context);
            return false;
        }
        return true;
    }

    private static int readArrayCount(
            ByteBuffer buffer, int elementComponentCount, String schemaName) {
        if (!validateRemaining(buffer, Integer.BYTES, schemaName + " count")) {
            return -1;
        }
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        int elementCount = buffer.getInt();
        if (elementCount < 0 || elementCount > MAX_ARRAY_SIZE) {
            logWarningOnce("Invalid array count: " + elementCount + " for " + schemaName);
            return -1;
        }
        int requiredElementBytes = elementCount * elementComponentCount * Float.BYTES;
        if (!validateRemaining(buffer, requiredElementBytes, schemaName + " elements")) {
            return -1;
        }
        return elementCount;
    }

    /** Decodes a float array from the payload buffer. */
    private static float[] decodeFloatArray(ByteBuffer buffer) {
        int elementCount = readArrayCount(buffer, 1, "float_array");
        if (elementCount < 0) {
            return null;
        }
        float[] result = new float[elementCount];
        for (int index = 0; index < elementCount; index++) {
            result[index] = buffer.getFloat();
        }
        return result;
    }

    private static Object decodeObject(ByteBuffer buffer, SchemaDescriptor descriptor) {
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        int componentBytes = descriptor.componentCount() * Float.BYTES;
        if (!validateRemaining(buffer, componentBytes, descriptor.name + " components")) {
            return null;
        }
        float[] values = new float[descriptor.componentCount()];
        for (int index = 0; index < values.length; index++) {
            values[index] = buffer.getFloat();
        }
        return interpretObject(values, descriptor);
    }

    private static Object decodeObjectArray(ByteBuffer buffer, SchemaDescriptor descriptor) {
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        int elementCount = readArrayCount(buffer, descriptor.componentCount(), descriptor.name + "_array");
        if (elementCount < 0) {
            return null;
        }
        if (descriptor.componentCount() == 0) {
            return new GenericObjectArray(descriptor, new float[elementCount][0]);
        }
        if (matchesVector2(descriptor)) {
            return decodeVector2ArrayValues(buffer, elementCount);
        }
        if (matchesVector3(descriptor)) {
            return decodeVector3ArrayValues(buffer, elementCount);
        }
        if (matchesPose2D(descriptor)) {
            return decodePose2DArrayValues(buffer, elementCount);
        }
        if (matchesPose3D(descriptor)) {
            return decodePose3DArrayValues(buffer, elementCount);
        }
        return decodeGenericObjectArray(buffer, descriptor, elementCount);
    }

    private static Object interpretObject(float[] values, SchemaDescriptor descriptor) {
        if (matchesVector2(descriptor)) {
            return new Vector2(values[0], values[1]);
        }
        if (matchesVector3(descriptor)) {
            return new Vector3(values[0], values[1], values[2]);
        }
        if (matchesPose2D(descriptor)) {
            return new Pose2D(values[0], values[1], values[2]);
        }
        if (matchesPose3D(descriptor)) {
            return new Pose3D(values[0], values[1], values[2], values[3], values[4], values[5]);
        }
        return new GenericObject(descriptor, values);
    }

    private static Vector2[] decodeVector2ArrayValues(ByteBuffer buffer, int elementCount) {
        Vector2[] vectors = new Vector2[elementCount];
        for (int index = 0; index < elementCount; index++) {
            float x = buffer.getFloat();
            float y = buffer.getFloat();
            vectors[index] = new Vector2(x, y);
        }
        return vectors;
    }

    private static Vector3[] decodeVector3ArrayValues(ByteBuffer buffer, int elementCount) {
        Vector3[] vectors = new Vector3[elementCount];
        for (int index = 0; index < elementCount; index++) {
            float x = buffer.getFloat();
            float y = buffer.getFloat();
            float z = buffer.getFloat();
            vectors[index] = new Vector3(x, y, z);
        }
        return vectors;
    }

    private static Pose2D[] decodePose2DArrayValues(ByteBuffer buffer, int elementCount) {
        Pose2D[] poses = new Pose2D[elementCount];
        for (int index = 0; index < elementCount; index++) {
            float x = buffer.getFloat();
            float y = buffer.getFloat();
            float rotation = buffer.getFloat();
            poses[index] = new Pose2D(x, y, rotation);
        }
        return poses;
    }

    private static Pose3D[] decodePose3DArrayValues(ByteBuffer buffer, int elementCount) {
        Pose3D[] poses = new Pose3D[elementCount];
        for (int index = 0; index < elementCount; index++) {
            float x = buffer.getFloat();
            float y = buffer.getFloat();
            float z = buffer.getFloat();
            float roll = buffer.getFloat();
            float pitch = buffer.getFloat();
            float yaw = buffer.getFloat();
            poses[index] = new Pose3D(x, y, z, roll, pitch, yaw);
        }
        return poses;
    }

    private static GenericObjectArray decodeGenericObjectArray(
            ByteBuffer buffer, SchemaDescriptor descriptor, int elementCount) {
        float[][] objects = new float[elementCount][descriptor.componentCount()];
        for (int element = 0; element < elementCount; element++) {
            for (int component = 0; component < descriptor.componentCount(); component++) {
                objects[element][component] = buffer.getFloat();
            }
        }
        return new GenericObjectArray(descriptor, objects);
    }

    private static boolean matchesVector2(SchemaDescriptor descriptor) {
        return descriptor.componentCount() == VECTOR2_COMPONENTS
                && (descriptor.name.equalsIgnoreCase("vector2")
                        || componentsMatch(descriptor.components, "x", "y"));
    }

    private static boolean matchesVector3(SchemaDescriptor descriptor) {
        return descriptor.componentCount() == VECTOR3_COMPONENTS
                && (descriptor.name.equalsIgnoreCase("vector3")
                        || componentsMatch(descriptor.components, "x", "y", "z"));
    }

    private static boolean matchesPose2D(SchemaDescriptor descriptor) {
        return descriptor.componentCount() == POSE2D_COMPONENTS
                && (descriptor.name.equalsIgnoreCase("pose2d")
                        || componentsMatch(descriptor.components, "x", "y", "rotation")
                        || componentsMatch(descriptor.components, "x", "y", "theta"));
    }

    private static boolean matchesPose3D(SchemaDescriptor descriptor) {
        return descriptor.componentCount() == POSE3D_COMPONENTS
                && (descriptor.name.equalsIgnoreCase("pose3d")
                        || componentsMatch(descriptor.components, "x", "y", "z", "roll", "pitch", "yaw"));
    }

    private static boolean componentsMatch(List<String> components, String... expected) {
        if (components.size() != expected.length) {
            return false;
        }
        for (int index = 0; index < expected.length; index++) {
            if (!components
                    .get(index)
                    .toLowerCase(Locale.ROOT)
                    .equals(expected[index].toLowerCase(Locale.ROOT))) {
                return false;
            }
        }
        return true;
    }

    private static void ensureManifestLoaded() {
        if (manifestLoaded) {
            return;
        }
        synchronized (manifestLock) {
            if (manifestLoaded) {
                return;
            }
            loadManifest();
            manifestLoaded = true;
        }
    }

    private static void loadManifest() {
        byte[] manifestBytes = fetchManifestBytes();
        if (manifestBytes == null || manifestBytes.length == 0) {
            logWarningOnce("Flatpack schema manifest is unavailable");
            return;
        }
        if (!parseManifest(manifestBytes)) {
            logWarningOnce("Failed to parse Flatpack schema manifest");
        }
    }

    private static byte[] fetchManifestBytes() {
        NetworkTableEntry manifestEntry = NetworkTableInstance.getDefault().getTable(VISION_TABLE_NAME)
                .getEntry(MANIFEST_ENTRY_NAME);
        return manifestEntry.getRaw(new byte[0]);
    }

    private static boolean parseManifest(byte[] manifestBytes) {
        ByteBuffer buffer = ByteBuffer.wrap(manifestBytes).order(ByteOrder.LITTLE_ENDIAN);
        if (buffer.remaining() < MANIFEST_MAGIC.length + 1 + 4 + 2) {
            logWarningOnce("Manifest buffer too small for header");
            return false;
        }
        for (byte magicByte : MANIFEST_MAGIC) {
            if (buffer.get() != magicByte) {
                logWarningOnce("Invalid FPKM manifest header");
                return false;
            }
        }
        int manifestVersion = buffer.get() & 0xFF;
        if (manifestVersion != SUPPORTED_MANIFEST_VERSION) {
            logWarningOnce("Unsupported manifest version: " + manifestVersion);
            return false;
        }
        int payloadLength = buffer.getInt();
        if (payloadLength < 2 || payloadLength > buffer.remaining()) {
            logWarningOnce("Invalid payload length: " + payloadLength);
            return false;
        }
        int schemaCount = buffer.getShort() & 0xFFFF;
        Map<String, SchemaDescriptor> parsedDescriptors = new HashMap<>();
        for (int schemaIndex = 0; schemaIndex < schemaCount; schemaIndex++) {
            String schemaName = readLengthPrefixedString(buffer, "schema name");
            if (schemaName == null) {
                logWarningOnce("Failed to read schema name");
                return false;
            }
            if (buffer.remaining() < 1) {
                logWarningOnce("Insufficient data for schema kind for " + schemaName);
                return false;
            }
            SchemaKind schemaKind = SchemaKind.fromByte(buffer.get() & 0xFF);
            if (schemaKind == null) {
                logWarningOnce("Failed to parse schema kind for " + schemaName);
                return false;
            }
            if (buffer.remaining() < 1) {
                logWarningOnce("Insufficient data for component count for " + schemaName);
                return false;
            }
            int componentCount = buffer.get() & 0xFF;
            List<String> components = new ArrayList<>(componentCount);
            for (int componentIndex = 0; componentIndex < componentCount; componentIndex++) {
                String component = readLengthPrefixedString(buffer, schemaName + " component");
                if (component == null) {
                    logWarningOnce("Failed to read component for " + schemaName);
                    return false;
                }
                components.add(component);
            }
            parsedDescriptors.put(schemaName, new SchemaDescriptor(schemaName, schemaKind, components));
        }
        schemaRegistry.clear();
        schemaRegistry.putAll(parsedDescriptors);
        return true;
    }

    private static String readLengthPrefixedString(ByteBuffer buffer, String context) {
        if (buffer.remaining() < 1) {
            logWarningOnce("Insufficient data for " + context + " length");
            return null;
        }
        int length = buffer.get() & 0xFF;
        if (buffer.remaining() < length) {
            logWarningOnce("Insufficient data for " + context);
            return null;
        }
        byte[] stringBytes = new byte[length];
        buffer.get(stringBytes);
        return new String(stringBytes, StandardCharsets.UTF_8);
    }
}
