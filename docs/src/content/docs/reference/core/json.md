---
title: Json
description: The small JSON reader and writer the library uses for its files.
sidebar:
  order: 20
---

`Json` is a small JSON reader and writer. It exists so that the core of the library needs no other libraries. It parses text into plain Java maps, lists and values, and writes them back in a fixed layout. The class has only static methods and cannot be constructed.

Package: `frc.team5712.path.core`.

## Where you meet it

Robot code does not need this class. [PathFile](/reference/core/pathfile/) and [RobotFile](/reference/core/robotfile/) use it to read and write the library's files, and `PathLibrary` uses it for field.json and the project file. It is public, so you can use it to read a small JSON file of your own. It is not a general-purpose JSON library.

## Example

```java
Object parsed = Json.parse("{ \"x\": 1.5, \"tags\": [1, 2, 3] }");
Map<String, Object> root = PathFile.asMap(parsed, "example");
double x = PathFile.number(root, "x", 0.0);              // 1.5

String text = Json.write(root, (double v) -> Json.fixed5(v));
```

## Reading

| Method | Description |
| --- | --- |
| `static Object parse(String text)` | Parses one JSON document and returns its value. |

What each JSON type becomes:

| JSON | Java |
| --- | --- |
| Object | `LinkedHashMap<String, Object>`, with the keys in the order they appear |
| Array | `List<Object>` |
| Number | `Double`, always, including whole numbers |
| String | `String` |
| `true`, `false` | `Boolean` |
| `null` | `null` |

`parse` throws `IllegalArgumentException` for text that is not valid JSON. The message starts with `JSON:` and gives the character offset, for example `JSON: expected ':' at offset 12`. Text left over after the value is an error too.

## Writing

| Method | Description |
| --- | --- |
| `static String write(Object value, DoubleFunction<String> numbers)` | Writes a value as JSON text ending in a newline. `numbers` turns each `double` into text. |

`value` may be a `Map` with `String` keys, a `List`, a `String`, a `Number`, a `Boolean` or `null`, nested to any depth. Anything else throws `IllegalArgumentException`.

The layout is deterministic:

- Indents are 2 spaces.
- An object or array that holds only plain values goes on one line, such as `{ "x": 1.5, "y": 0 }`.
- An object or array that holds another object or array is spread out, one entry per line.
- Keys come out in the map's own order. Use a `LinkedHashMap` and insert the keys in the order you want.
- An `Integer` or `Long` is written as a whole number. Every other number goes through `numbers`.
- An empty object is `{}` and an empty array is `[]`.

## Number formats

Pass one of these to `write`, or call them on their own.

| Method | Description |
| --- | --- |
| `static String fixed5(double x)` | Rounds to 5 decimal places and drops trailing zeros. `1.5` gives `1.5`, `3.0` gives `3`, and `-0.00001` gives `-0.00001`. A value that rounds to zero gives `0`. |
| `static String exact(double x)` | The shortest text that reads back to exactly the same `double`. Whole numbers smaller in size than 1e15 are written without a decimal point. Used for test data. |

Both throw `IllegalArgumentException` for `NaN` or an infinite value, with a message that starts `JSON can't hold`.

`fixed5` is what the path file writer uses. The editor's writer prints the same characters, so a file saved by either one is identical.
