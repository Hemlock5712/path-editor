// A MessagePack decoder, just enough for NT4's binary frames: each frame is one or more
// [topic id, timestamp µs, type, value] arrays back to back.

export type MsgValue = null | boolean | number | bigint | string | Uint8Array | MsgValue[] | { [key: string]: MsgValue };

export class MsgReader {
  private at = 0;
  private readonly view: DataView;

  constructor(private readonly bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }

  get done(): boolean {
    return this.at >= this.bytes.length;
  }

  read(): MsgValue {
    const b = this.u8();
    if (b <= 0x7f) return b;
    if (b >= 0xe0) return b - 0x100;
    if ((b & 0xe0) === 0xa0) return this.str(b & 0x1f);
    if ((b & 0xf0) === 0x90) return this.array(b & 0x0f);
    if ((b & 0xf0) === 0x80) return this.map(b & 0x0f);
    switch (b) {
      case 0xc0:
        return null;
      case 0xc2:
        return false;
      case 0xc3:
        return true;
      case 0xc4:
        return this.bin(this.u8());
      case 0xc5:
        return this.bin(this.u16());
      case 0xc6:
        return this.bin(this.u32());
      case 0xca:
        return this.take(4, (v, o) => v.getFloat32(o));
      case 0xcb:
        return this.take(8, (v, o) => v.getFloat64(o));
      case 0xcc:
        return this.u8();
      case 0xcd:
        return this.u16();
      case 0xce:
        return this.u32();
      case 0xcf:
        return this.int64(false);
      case 0xd0:
        return this.take(1, (v, o) => v.getInt8(o));
      case 0xd1:
        return this.take(2, (v, o) => v.getInt16(o));
      case 0xd2:
        return this.take(4, (v, o) => v.getInt32(o));
      case 0xd3:
        return this.int64(true);
      case 0xd9:
        return this.str(this.u8());
      case 0xda:
        return this.str(this.u16());
      case 0xdb:
        return this.str(this.u32());
      case 0xdc:
        return this.array(this.u16());
      case 0xdd:
        return this.array(this.u32());
      case 0xde:
        return this.map(this.u16());
      case 0xdf:
        return this.map(this.u32());
      default:
        throw new RangeError(`msgpack type 0x${b.toString(16)} not supported`);
    }
  }

  private take<T>(n: number, get: (v: DataView, offset: number) => T): T {
    if (this.at + n > this.bytes.length) throw new RangeError("msgpack frame ends early");
    const v = get(this.view, this.at);
    this.at += n;
    return v;
  }

  private u8 = () => this.take(1, (v, o) => v.getUint8(o));
  private u16 = () => this.take(2, (v, o) => v.getUint16(o));
  private u32 = () => this.take(4, (v, o) => v.getUint32(o));

  private int64(signed: boolean): number {
    const big = this.take(8, (v, o) => (signed ? v.getBigInt64(o) : v.getBigUint64(o)));
    return Number(big);
  }

  private bin(n: number): Uint8Array {
    const out = this.bytes.slice(this.at, this.at + n);
    this.at += n;
    return out;
  }

  private str(n: number): string {
    return new TextDecoder().decode(this.bin(n));
  }

  private array(n: number): MsgValue[] {
    const out: MsgValue[] = [];
    for (let i = 0; i < n; i++) out.push(this.read());
    return out;
  }

  private map(n: number): { [key: string]: MsgValue } {
    const out: { [key: string]: MsgValue } = {};
    for (let i = 0; i < n; i++) out[String(this.read())] = this.read();
    return out;
  }
}

/** Every value in a frame. */
export function decodeAll(bytes: Uint8Array): MsgValue[] {
  const r = new MsgReader(bytes);
  const out: MsgValue[] = [];
  while (!r.done) out.push(r.read());
  return out;
}

/** Encodes integers and arrays of them: all NT4's binary messages from a client need (time sync pings). */
export function encode(value: number | readonly number[]): Uint8Array {
  const out: number[] = [];
  const int = (n: number) => {
    if (!Number.isInteger(n)) throw new RangeError(`not an integer: ${n}`);
    if (n >= 0 && n <= 0x7f) out.push(n);
    else if (n < 0 && n >= -32) out.push(0x100 + n);
    else {
      // int64, big-endian
      const b = new DataView(new ArrayBuffer(8));
      b.setBigInt64(0, BigInt(n));
      out.push(0xd3, ...new Uint8Array(b.buffer));
    }
  };
  if (typeof value === "number") int(value);
  else {
    if (value.length > 15) throw new RangeError("only short arrays");
    out.push(0x90 | value.length);
    for (const v of value) int(v);
  }
  return new Uint8Array(out);
}
