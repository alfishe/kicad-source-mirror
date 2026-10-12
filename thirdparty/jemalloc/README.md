# jemalloc 5.3.0 (vendored)

Process allocator of Linux builds (`KICAD_ALLOCATOR=auto|jemalloc`, see `cmake/BuildJemalloc.cmake`):
built from this tree at configure time, linked statically into every executable.

- Source: https://github.com/jemalloc/jemalloc/releases/download/5.3.0/jemalloc-5.3.0.tar.bz2
  (SHA-256 `2db82d1e7119df3e71b7640219b6dfe84789bc0537983c3b7ac4f7189aecfeaa`), BSD-2-Clause (`COPYING`).
- Unmodified files; removed from the release: tests (except configure inputs `test/**/*.in`),
  documentation (except `doc/*.in`), `msvc/`, `doc_internal/`, `scripts/`, `m4/`, `configure.ac`,
  `autogen.sh`, CI files, ChangeLog. `configure` is the release's own (no autotools needed).
- Update: unpack the new release, remove the same parts, check `configure` still runs from a
  separate build directory, update the version in `cmake/BuildJemalloc.cmake`.
