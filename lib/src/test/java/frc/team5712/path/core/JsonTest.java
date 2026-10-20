package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class JsonTest {
  @Test
  void fixed5RoundsAndTrims() {
    assertEquals("0", Json.fixed5(0.0));
    assertEquals("0", Json.fixed5(-0.0));
    assertEquals("0", Json.fixed5(4e-6));
    assertEquals("0.00001", Json.fixed5(6e-6));
    assertEquals("-0.00001", Json.fixed5(-1e-5));
    assertEquals("1.5", Json.fixed5(1.5));
    assertEquals("-2", Json.fixed5(-2.000001));
    assertEquals("3.14159", Json.fixed5(Math.PI));
    assertEquals("123456.78901", Json.fixed5(123456.789012));
    assertEquals("0.1", Json.fixed5(0.1 + 0.2 - 0.2));
    assertThrows(IllegalArgumentException.class, () -> Json.fixed5(Double.NaN));
  }

  @Test
  void exactRoundTripsEveryDouble() {
    double[] values = {0.1, 1.0 / 3, -2.5e-12, 1e300, 6.02214076e23, 42, -0.0, Math.PI};
    for (double v : values) {
      assertEquals(v, Double.parseDouble(Json.exact(v)), 0.0);
    }
    assertEquals("42", Json.exact(42.0));
    assertThrows(IllegalArgumentException.class, () -> Json.exact(Double.POSITIVE_INFINITY));
  }

  @Test
  void parsesTheUsualShapes() {
    Object v = Json.parse(" { \"a\": [1, 2.5e1, -3], \"b\": { \"c\": \"x\\\"y\\u0041\" }, \"d\": true, \"e\": null } ");
    Map<?, ?> m = (Map<?, ?>) v;
    assertEquals(List.of(1.0, 25.0, -3.0), m.get("a"));
    assertEquals("x\"yA", ((Map<?, ?>) m.get("b")).get("c"));
    assertEquals(Boolean.TRUE, m.get("d"));
    assertEquals(null, m.get("e"));
    assertEquals(List.of("a", "b", "d", "e"), List.copyOf(m.keySet()));
  }

  @Test
  void rejectsBrokenText() {
    for (String bad : new String[] {"", "{", "[1,]", "{\"a\" 1}", "tru", "\"abc", "1 2", "{\"a\":-}"}) {
      assertThrows(IllegalArgumentException.class, () -> Json.parse(bad), bad);
    }
  }

  @Test
  void writesPlainContainersInlineAndTheRestSpread() {
    Map<String, Object> inner = new LinkedHashMap<>();
    inner.put("x", 1.25);
    inner.put("name", "a\nb");
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("n", 3);
    root.put("list", List.of(inner, List.of()));
    root.put("nums", List.of(1.0, 2.0));
    root.put("empty", Map.of());
    String expected =
        """
        {
          "n": 3,
          "list": [
            { "x": 1.25, "name": "a\\nb" },
            []
          ],
          "nums": [1, 2],
          "empty": {}
        }
        """;
    assertEquals(expected, Json.write(root, Json::fixed5));
  }
}
