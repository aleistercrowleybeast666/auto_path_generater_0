# V4.0 BIN IO

Phase 2 keeps the exact V4.0 packed formats defined in Phase 1:

```text
Header   104 bytes
Node      16 bytes
Segment   24 bytes
Action    22 bytes
```

The low-level struct implementation remains in
`src/hjmb_pathgen/py_io/codecs/binary_layout.py`. Public APIs are provided by
`src/hjmb_pathgen/py_io/codecs/bin_codec.py`:

```text
encode_trajectory(compiled)
decode_trajectory(data, expected_filename=None)
load_bin(path)
save_bin(path, compiled)
```

## Validation Scope

The decoder and encoder validate:

- magic, version, struct sizes
- required header flags and unknown flag bits
- nominal field size
- counts, offsets, file size, and trailing bytes
- all reserved fields
- CRC-32/IEEE with `file_crc32` zeroed during calculation
- `Pxxxx.BIN` filename/traj_id identity
- START uniqueness and zero velocity
- ARRIVAL `EXACT_PASS`, zero velocity, and contiguous arrival IDs
- unique `FINISH_ARM` at the final drop ARRIVAL
- reserved `SAFE_END` bits are zero in formal V4.0 output
- reserved `FINISH_CLEAR` segment bits are zero in formal V4.0 output
- legacy half-plane/brake finish fields are zero
- segment IDs, boundaries, `s` values, adjacency, and planned-time sum
- action sequence and mode-specific field combinations

Phase 2 does not implement Phase 4+ dynamics, wheel rpm, topology, or collision
validation.

## Single Codec Path

Single-case output and partial batch output both call
`compile_case_to_trajectory()` and `encode_trajectory()`. No GUI, batch, or
portable path packs BIN structs independently.
