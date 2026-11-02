# RapidJSON (vendored, header-only)

JSON parser/writer used by the vendored OpenCASCADE glTF reader/writer (TKDEGLTF, TKRWMesh,
see `thirdparty/opencascade`). Not used by KiCad sources directly.

- Source: https://github.com/Tencent/rapidjson/archive/24b5e7a8b27f42fa16b96fc70aade9106cf7102f.tar.gz
  (master of 2025-02-05, SHA-256 `2d2601a82d2d3b7e143a3c8d43ef616671391034bc46891a9816b79cf2d3e7a8`),
  MIT; bundled `msinttypes` BSD-3-Clause (`license.txt`). Release 1.1.0 (2016) does not compile
  with GCC 14 and later (`GenericStringRef::operator=` assigns const members).
- Unmodified files; kept from the archive: `include/`, `license.txt`. Removed: everything else
  (tests, examples, docs, `bin/` with the JSON-licensed jsonchecker, build and CI files).
- Update: unpack the new archive, keep the same parts, rebuild OpenCASCADE (`KICAD_OCC=vendored`)
  and check the glTF export.
