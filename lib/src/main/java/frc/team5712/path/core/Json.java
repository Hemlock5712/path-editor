package frc.team5712.path.core;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleFunction;

/**
 * A small JSON reader and writer, so the core needs no libraries. Objects read as
 * {@code LinkedHashMap<String, Object>} (key order kept), arrays as {@code List<Object>}, numbers as
 * {@code Double}.
 *
 * <p>The writer is deterministic: 2-space indents, an object or array that holds only plain values
 * goes on one line ({@code { "x": 1.5, "y": 0 }}), anything else is spread one entry per line. Keys
 * come out in insertion order, so callers fix the order.
 */
public final class Json {
  private Json() {}

  // ------------------------------------------------------------------------------ numbers

  /**
   * Rounds to 5 decimals and drops trailing zeros: {@code 1.5}, {@code -0.00001}, {@code 3}. Integer
   * arithmetic after one rounding, so the TypeScript writer prints the same characters.
   */
  public static String fixed5(double x) {
    if (!Double.isFinite(x)) {
      throw new IllegalArgumentException("JSON can't hold " + x);
    }
    long r = Math.round(x * 100000.0);
    if (r == 0) {
      return "0";
    }
    String sign = r < 0 ? "-" : "";
    long abs = Math.abs(r);
    long whole = abs / 100000;
    long frac = abs % 100000;
    if (frac == 0) {
      return sign + whole;
    }
    StringBuilder digits = new StringBuilder(Long.toString(frac));
    while (digits.length() < 5) {
      digits.insert(0, '0');
    }
    int end = digits.length();
    while (digits.charAt(end - 1) == '0') {
      end--;
    }
    return sign + whole + "." + digits.substring(0, end);
  }

  /** Shortest text that reads back to the same double (for test vectors). */
  public static String exact(double x) {
    if (!Double.isFinite(x)) {
      throw new IllegalArgumentException("JSON can't hold " + x);
    }
    if (x == Math.rint(x) && Math.abs(x) < 1e15) {
      return Long.toString((long) x);
    }
    return Double.toString(x);
  }

  // ------------------------------------------------------------------------------ writing

  /** Writes a value with the given number format, ending in a newline. */
  public static String write(Object value, DoubleFunction<String> numbers) {
    StringBuilder out = new StringBuilder();
    write(out, value, numbers, "");
    out.append('\n');
    return out.toString();
  }

  private static boolean isPlain(Object v) {
    return !(v instanceof Map) && !(v instanceof List);
  }

  private static void write(StringBuilder out, Object v, DoubleFunction<String> numbers, String indent) {
    if (v == null) {
      out.append("null");
    } else if (v instanceof Number n) {
      if (v instanceof Integer || v instanceof Long) {
        out.append(n.longValue());
      } else {
        out.append(numbers.apply(n.doubleValue()));
      }
    } else if (v instanceof Boolean b) {
      out.append(b);
    } else if (v instanceof String s) {
      writeString(out, s);
    } else if (v instanceof Map<?, ?> map) {
      if (map.isEmpty()) {
        out.append("{}");
        return;
      }
      boolean inline = map.values().stream().allMatch(Json::isPlain);
      String inner = indent + "  ";
      out.append(inline ? "{ " : "{\n");
      boolean first = true;
      for (Map.Entry<?, ?> e : map.entrySet()) {
        if (!first) {
          out.append(inline ? ", " : ",\n");
        }
        first = false;
        if (!inline) {
          out.append(inner);
        }
        writeString(out, (String) e.getKey());
        out.append(": ");
        write(out, e.getValue(), numbers, inner);
      }
      out.append(inline ? " }" : "\n" + indent + "}");
    } else if (v instanceof List<?> list) {
      if (list.isEmpty()) {
        out.append("[]");
        return;
      }
      boolean inline = list.stream().allMatch(Json::isPlain);
      String inner = indent + "  ";
      out.append(inline ? "[" : "[\n");
      for (int i = 0; i < list.size(); i++) {
        if (i > 0) {
          out.append(inline ? ", " : ",\n");
        }
        if (!inline) {
          out.append(inner);
        }
        write(out, list.get(i), numbers, inner);
      }
      out.append(inline ? "]" : "\n" + indent + "]");
    } else {
      throw new IllegalArgumentException("can't write " + v.getClass());
    }
  }

  private static void writeString(StringBuilder out, String s) {
    out.append('"');
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      switch (c) {
        case '"' -> out.append("\\\"");
        case '\\' -> out.append("\\\\");
        case '\n' -> out.append("\\n");
        case '\r' -> out.append("\\r");
        case '\t' -> out.append("\\t");
        default -> {
          if (c < 0x20) {
            out.append(String.format("\\u%04x", (int) c));
          } else {
            out.append(c);
          }
        }
      }
    }
    out.append('"');
  }

  // ------------------------------------------------------------------------------ reading

  /** Parses a JSON document. */
  public static Object parse(String text) {
    Reader r = new Reader(text);
    r.skipSpace();
    Object v = r.value();
    r.skipSpace();
    if (r.pos != text.length()) {
      throw r.error("unexpected text after the value");
    }
    return v;
  }

  private static final class Reader {
    private final String text;
    private int pos;

    Reader(String text) {
      this.text = text;
    }

    IllegalArgumentException error(String message) {
      return new IllegalArgumentException("JSON: " + message + " at offset " + pos);
    }

    void skipSpace() {
      while (pos < text.length() && Character.isWhitespace(text.charAt(pos))) {
        pos++;
      }
    }

    void expect(char c) {
      if (pos >= text.length() || text.charAt(pos) != c) {
        throw error("expected '" + c + "'");
      }
      pos++;
    }

    Object value() {
      if (pos >= text.length()) {
        throw error("unexpected end");
      }
      char c = text.charAt(pos);
      return switch (c) {
        case '{' -> object();
        case '[' -> array();
        case '"' -> string();
        case 't' -> literal("true", Boolean.TRUE);
        case 'f' -> literal("false", Boolean.FALSE);
        case 'n' -> literal("null", null);
        default -> number();
      };
    }

    private Object literal(String word, Object v) {
      if (!text.startsWith(word, pos)) {
        throw error("bad literal");
      }
      pos += word.length();
      return v;
    }

    private Map<String, Object> object() {
      Map<String, Object> map = new LinkedHashMap<>();
      expect('{');
      skipSpace();
      if (pos < text.length() && text.charAt(pos) == '}') {
        pos++;
        return map;
      }
      while (true) {
        skipSpace();
        String key = string();
        skipSpace();
        expect(':');
        skipSpace();
        map.put(key, value());
        skipSpace();
        if (pos < text.length() && text.charAt(pos) == ',') {
          pos++;
          continue;
        }
        expect('}');
        return map;
      }
    }

    private List<Object> array() {
      List<Object> list = new ArrayList<>();
      expect('[');
      skipSpace();
      if (pos < text.length() && text.charAt(pos) == ']') {
        pos++;
        return list;
      }
      while (true) {
        skipSpace();
        list.add(value());
        skipSpace();
        if (pos < text.length() && text.charAt(pos) == ',') {
          pos++;
          continue;
        }
        expect(']');
        return list;
      }
    }

    private String string() {
      expect('"');
      StringBuilder sb = new StringBuilder();
      while (true) {
        if (pos >= text.length()) {
          throw error("unterminated string");
        }
        char c = text.charAt(pos++);
        if (c == '"') {
          return sb.toString();
        }
        if (c != '\\') {
          sb.append(c);
          continue;
        }
        if (pos >= text.length()) {
          throw error("unterminated escape");
        }
        char e = text.charAt(pos++);
        switch (e) {
          case '"', '\\', '/' -> sb.append(e);
          case 'b' -> sb.append('\b');
          case 'f' -> sb.append('\f');
          case 'n' -> sb.append('\n');
          case 'r' -> sb.append('\r');
          case 't' -> sb.append('\t');
          case 'u' -> {
            if (pos + 4 > text.length()) {
              throw error("bad unicode escape");
            }
            sb.append((char) Integer.parseInt(text.substring(pos, pos + 4), 16));
            pos += 4;
          }
          default -> throw error("bad escape");
        }
      }
    }

    private Double number() {
      int start = pos;
      while (pos < text.length() && "+-0123456789.eE".indexOf(text.charAt(pos)) >= 0) {
        pos++;
      }
      if (start == pos) {
        throw error("unexpected character");
      }
      try {
        return Double.parseDouble(text.substring(start, pos));
      } catch (NumberFormatException ex) {
        throw error("bad number");
      }
    }
  }
}
