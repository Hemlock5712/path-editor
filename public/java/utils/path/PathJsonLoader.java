package frc.robot.utils.path;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads and saves path data from/to JSON format.
 *
 * <p>Compatible with the React path editor's JSON export format. Uses Jackson (bundled with
 * WPILib).
 */
public final class PathJsonLoader {

  private static final ObjectMapper MAPPER = new ObjectMapper();

  private PathJsonLoader() {}

  /**
   * Loads a path from the deploy directory.
   *
   * @param filename Filename relative to deploy/paths/ (e.g., "myPath.json")
   * @return Parsed path data
   * @throws IOException if the file cannot be read
   */
  public static PathData fromFile(String filename) throws IOException {
    Path filePath = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(filename);
    String json = Files.readString(filePath);
    return fromJson(json);
  }

  /**
   * Parses path data from a JSON string.
   *
   * @param json JSON string in the path editor format
   * @return Parsed path data
   * @throws RuntimeException if the JSON is invalid
   */
  public static PathData fromJson(String json) {
    try {
      JsonNode root = MAPPER.readTree(json);

      // Control points (required)
      List<Translation2d> controlPoints = new ArrayList<>();
      for (JsonNode pt : root.get("controlPoints")) {
        controlPoints.add(new Translation2d(pt.get("x").asDouble(), pt.get("y").asDouble()));
      }

      // Heading waypoints (optional)
      List<PathData.HeadingWaypoint> headingWaypoints = new ArrayList<>();
      if (root.has("headingWaypoints")) {
        for (JsonNode hw : root.get("headingWaypoints")) {
          headingWaypoints.add(
              new PathData.HeadingWaypoint(
                  hw.get("waypointIndex").asDouble(),
                  Rotation2d.fromDegrees(hw.get("degrees").asDouble())));
        }
      }

      // Global constraints (optional, use defaults if missing)
      VelocityConstraints constraints = VelocityConstraints.defaults();
      if (root.has("constraints")) {
        JsonNode c = root.get("constraints");
        if (c.has("maxVelocity")) {
          constraints = constraints.withMaxVelocity(c.get("maxVelocity").asDouble());
        }
        if (c.has("maxAcceleration")) {
          constraints = constraints.withMaxAcceleration(c.get("maxAcceleration").asDouble());
        }
        if (c.has("startVelocity")) {
          constraints = constraints.withStartVelocity(c.get("startVelocity").asDouble());
        }
        if (c.has("endVelocity")) {
          constraints = constraints.withEndVelocity(c.get("endVelocity").asDouble());
        }
      }

      return new PathData(controlPoints, headingWaypoints, constraints);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse path JSON", e);
    }
  }

  /**
   * Serializes path data to a JSON string.
   *
   * @param data Path data to serialize
   * @return JSON string
   */
  public static String toJson(PathData data) {
    ObjectNode root = MAPPER.createObjectNode();
    root.put("version", "1.0");

    // Control points
    ArrayNode points = root.putArray("controlPoints");
    for (Translation2d pt : data.controlPoints()) {
      ObjectNode p = points.addObject();
      p.put("x", pt.getX());
      p.put("y", pt.getY());
    }

    // Heading waypoints
    if (!data.headingWaypoints().isEmpty()) {
      ArrayNode headings = root.putArray("headingWaypoints");
      for (PathData.HeadingWaypoint hw : data.headingWaypoints()) {
        ObjectNode h = headings.addObject();
        h.put("waypointIndex", hw.waypointIndex());
        h.put("degrees", hw.heading().getDegrees());
      }
    }

    // Constraints
    ObjectNode constraints = root.putObject("constraints");
    constraints.put("maxVelocity", data.globalConstraints().getMaxVelocity());
    constraints.put("maxAcceleration", data.globalConstraints().getMaxAcceleration());
    constraints.put("startVelocity", data.globalConstraints().getStartVelocity());
    constraints.put("endVelocity", data.globalConstraints().getEndVelocity());

    return root.toString();
  }
}
