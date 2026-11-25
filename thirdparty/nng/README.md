# nng 2.1.0 (vendored)

The IPC API's transport (`libs/kinng`, `common/api/api_client.cpp`): built from this tree at configure
time as a static library (`cmake/BuildNng.cmake`), linked into kinng and its users. No system nng.

- Source: https://github.com/nanomsg/nng/archive/refs/tags/v2.1.0.tar.gz
  (SHA-256 `e2fdcdf126b530e331b4727812b710629cdb5d90d230ce8f080e2eb520299d8d`), MIT (`LICENSE.txt`).
- Unmodified files; removed from the release: `docs/`, `demo/`, `tests/`, `tools/`, `etc/`, the unit
  tests `src/**/*_test.c`, `CODE_OF_CONDUCT.md`, `UKRAINE.md`, dot files.
- Built without TLS, HTTP and WebSocket (`NNG_CMAKE_ARGS` in `cmake/BuildNng.cmake`).
- Update: unpack the new release, remove the same parts, update `NNG_VERSION` in `cmake/BuildNng.cmake`,
  check the migration notes of a new major version (`docs/ref/migrate/` upstream).
- Windows: `ipc://` is AF_UNIX in nng 2 and AF_UNIX is not enabled by 2.1.0's CMake check (its header list
  is passed unquoted), so KiCad's `ipc://` URLs go to nng as `winpipe://` there (`KinngTransportUrl`), the
  named pipe nng 1.x used for `ipc://`. After an update run `qa_kinng` on Windows.
