# Open CASCADE Technology 7.9.3 (vendored)

STEP/IGES/glTF/VRML/STL import and export of pcbnew and the 3D model plugins
(`KICAD_OCC=vendored`, the default; see `cmake/BuildOpenCascade.cmake`). Built from this tree as
static libraries and linked into the modules that use it, so a KiCad build needs no OpenCASCADE
package and a KiCad install carries no OpenCASCADE shared libraries. `KICAD_OCC=system` uses an
installed OpenCASCADE instead (`cmake/FindOCC.cmake`).

- Source: https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/V7_9_3.tar.gz
  (SHA-256 `5ecf094ec6b12d5413dfb851d8c3590c354058aee556e32e408bdfbf8c357d57`),
  LGPL-2.1 with the Open CASCADE exception (`LICENSE_LGPL_21.txt`, `OCCT_LGPL_EXCEPTION.txt`).
- Unmodified files. Kept: `CMakeLists.txt`, `adm/` (build scripts, module and resource lists)
  without `adm/vcpkg`, `.clang-format` (read by `CMakeLists.txt`), `.gitattributes`, the licence
  files, `README.md`, and in `src/` the 48 toolkits of the modules FoundationClasses,
  ModelingData, ModelingAlgorithms, Visualization (without the OpenGL/GLES/D3D/VTK drivers),
  ApplicationFramework and DataExchange, their packages and the resource directories of
  `adm/RESOURCES`.
- Removed from the release: `data/`, `dox/`, `samples/`, `tests/`, `tools/`, `.github/`,
  `.gitignore`, `.tgitconfig`, `.git-blame-ignore-revs`, `adm/vcpkg`, and from `src/` the Draw
  test harness and the drivers that are not built:
  BOPTest BRepTest D3DHost D3DHostTest DBRep DDataStd DDF DDocStd DNaming DPrsStd Draw DrawDim
  DRAWEXE DrawFairCurve DrawTrSurf Expr Express ExprIntrp ExpToCasExe GeometryTest GeomliteTest
  HLRTest IVtk IVtkDraw IVtkOCC IVtkTools IVtkVTK MeshTest OpenGl OpenGlTest OS QABugs QADNaming
  QADraw QANCollection SWDRAW TKD3DHost TKD3DHostTest TKDCAF TKDraw TKExpress TKIVtk TKIVtkDraw
  TKOpenGl TKOpenGles TKOpenGlesTest TKOpenGlTest TKQADraw TKTObjDRAW TKTopTest TKViewerTest
  TKXDEDRAW TKXSDRAW TKXSDRAWDE TKXSDRAWGLTF TKXSDRAWIGES TKXSDRAWOBJ TKXSDRAWPLY TKXSDRAWSTEP
  TKXSDRAWSTL TKXSDRAWVRML TObjDRAW ViewerTest XDEDRAW XSDRAW XSDRAWDE XSDRAWGLTF XSDRAWIGES
  XSDRAWOBJ XSDRAWPLY XSDRAWSTEP XSDRAWSTL XSDRAWVRML.
  OCCT's header collection warns that `src/TKOpenGl` has no FILES list; that is expected.
- Dependencies: FreeType (KiCad's own), RapidJSON (`thirdparty/rapidjson`); TBB, FreeImage,
  FFmpeg, VTK, Draco, OpenVR, Tcl/Tk and OpenGL are off.
- GCC 16 and later: built with `-fno-ipa-cp` (IPA constant propagation clones
  `NCollection_UBTree::Select` in `BRepClass3d_SClassifier.cxx` for the wrong selector and STEP
  import crashes, upstream-issues A71).
- Update: unpack the new release, remove the same parts (check `adm/MODULES` and each kept
  toolkit's `PACKAGES`/`EXTERNLIB` for new toolkits or packages), check that a configure from a
  separate build directory finds every kept toolkit, update the version and the link order in
  `cmake/BuildOpenCascade.cmake`, then run the STEP export/import suites on all three OSes.
