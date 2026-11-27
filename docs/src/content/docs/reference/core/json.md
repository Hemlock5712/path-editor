---
title: Json
description: The small JSON reader and writer the library uses for its files.
sidebar:
  order: 20
---

`Json` is a small JSON reader and writer, so that the core of the library needs no other libraries.

Robot code does not need this class. [PathFile](/reference/core/pathfile/) and [RobotFile](/reference/core/robotfile/) use it for the library's files, and `PathLibrary` uses it for field.json and the project file.

Package: `frc.team5712.path.core`.

## Example

```java
Object parsed = Json.parse("{ \"x\": 1.5, \"tags\": [1, 2, 3] }");
Map<String, Object> root = PathFile.asMap(parsed, "example");
double x = PathFile.number(root, "x", 0.0);              // 1.5

// Write it back, each number rounded to 5 decimal places
String text = Json.write(root, (double v) -> Json.fixed5(v));
```

## Methods

The class has only static methods and cannot be constructed.

| Method | What it does |
| --- | --- |
| `static Object parse(String text)` | Parses one JSON document and returns its value. Throws `IllegalArgumentException` for text that is not valid JSON, or for text left over after the value. The message starts with `JSON:` and gives the character offset, for example `JSON: expected ':' at offset 12`. |
| `static String write(Object value, DoubleFunction<String> numbers)` | Writes a value as JSON text ending in a newline. `numbers` turns each `double` into text. `value` may be a `Map` with `String` keys, a `List`, a `String`, a `Number`, a `Boolean` or `null`, nested to any depth. Anything else throws `IllegalArgumentException`. |
| `static String fixed5(double x)` | Rounds to 5 decimal places and drops trailing zeros. `1.5` gives `1.5`, `3.0` gives `3`, and `-0.00001` gives `-0.00001`. A value that rounds to zero gives `0`. Throws `IllegalArgumentException` for `NaN` or an infinite value, with a message that starts `JSON can't hold`. |
| `static String exact(double x)` | The shortest text that reads back to exactly the same `double`. Whole numbers smaller in size than 1e15 are written without a decimal point. Used for test data. Throws the same as `fixed5`. |

## What parse returns

| JSON | Java |
| --- | --- |
| Object | `LinkedHashMap<String, Object>`, with the keys in the order they appear |
| Array | `List<Object>` |
| Number | `Double`, always, including whole numbers |
| String | `String` |
| `true`, `false` | `Boolean` |
| `null` | `null` |

## How write lays text out

| Case | Output |
| --- | --- |
| Indents | 2 spaces |
| An object or array that holds only plain values | One line, such as `{ "x": 1.5, "y": 0 }` |
| An object or array that holds another object or array | Spread out, one entry per line |
| Keys | In the map's own order. Use a `LinkedHashMap` and insert the keys in the order you want |
| An `Integer` or `Long` | A whole number. Every other number goes through `numbers` |
| Empty | An empty object is `{}` and an empty array is `[]` |

## Good to know

- **It is public**, so you can use it to read a small JSON file of your own. It is not a general-purpose JSON library.
- **The layout is deterministic.** `fixed5` is what the path file writer uses, and the editor's writer prints the same characters, so a file saved by either one is identical.
- **Pass `fixed5` or `exact` to `write`**, or call them on their own.

## More

- [PathFile](/reference/core/pathfile/): reads and writes path files with it.
- [RobotFile](/reference/core/robotfile/): reads robot.json with it.
