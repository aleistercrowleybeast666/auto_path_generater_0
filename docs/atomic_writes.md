# Phase 2 Atomic Writes

`src/hjmb_pathgen/services/atomic_writer.py` implements same-directory atomic
writes for JSON, BIN, and report files.

## Write Sequence

1. Create a unique temp file in the final file's directory.
2. Write all bytes.
3. Flush and `os.fsync()` the temp file.
4. Run the caller-provided validator on the temp file.
5. Replace the final path with `os.replace(temp, final)`.
6. Best-effort fsync the directory on platforms that support it.

If any step fails, the temp file is removed and the previous final file is left
unchanged.

## Write-Back Validation

JSON saves reload the temp JSON into the same typed model. Case and portable
case saves validate against the final canonical `Pxxxx` filename, not the temp
filename.

BIN saves decode the temp BIN, validate CRC and filename/traj_id identity, then
re-encode and require byte identity.

Reports are parsed or byte-compared before replacement.

## Tested Failure Modes

The Phase 2 test suite covers:

- successful atomic write
- validator failure
- replace failure
- old final bytes preserved after failure
- temp cleanup after failure
