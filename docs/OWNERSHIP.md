# Ownership boundaries

## Scope

The release preserves the generator's established object graph while making
the ownership rules used by the maintained command-line path explicit. A
repository-wide conversion to smart pointers is intentionally deferred because
curves, patches, geometry and meshes contain shared non-owning relationships
whose numerical behavior is covered by the scientific regressions.

## Stable ownership rules

| Owner | Owned objects | Non-owning references |
|---|---|---|
| `Geometry` | Points, vectors, curves and patches stored as `std::unique_ptr` | None exposed as ownership transfers |
| `MeshAdaptive` | Its positioned `SubMesh` instances | None |
| `SubMesh` | Nodes and elements stored as `std::unique_ptr` | The associated `Patch` |
| `Model` | Meshes inserted through `InsertMeshAdaptive` | Its `Geometry` |
| `GeneratorAdaptive` | Current mesh, saved meshes, communicator and ID manager | Geometry/model relationships used during an execution |
| `PatchReader` | Only its unconsumed look-ahead patch objects | Parsed objects after transfer to `Geometry` |

`Geometry::InsertCurve`, `Geometry::InsertPatch`, `Geometry::AddPoint` and
`Geometry::AddVector` transfer ownership through `std::unique_ptr`.
`MeshAdaptive::InsertSubMeshAdaptive*`, `SubMesh::SetNoh` and
`SubMesh::SetElement` are legacy raw-pointer entry points that take ownership;
new code should prefer the explicit `std::unique_ptr` overloads where
available.

## Shared references

- A `CurveAdaptive` does not own the points in its sampling list or the
  incident patches in its patch list. Those objects are shared by the geometry
  and mesh graph.
- A `Patch` does not own its `SubMesh`; the active `MeshAdaptive` owns it.
- A `SubMesh` does not own its associated `Patch`; the `Geometry` owns it.
- A `Model` does not own its `Geometry`, but it does delete meshes inserted
  into its mesh list.
- Raw pointers returned by getters are borrowed views. Callers must not delete
  them unless an API explicitly documents an ownership transfer.

## Temporary objects

The `.bp` reader interns control vertices while constructing geometry. Parsed
patch objects are temporary until their data are moved into a
geometry-registered patch; the temporary object is then destroyed.
`PatchReader` owns only a final unconsumed look-ahead object at destruction.

During reconstruction, candidate submeshes and points remain owned by local
smart pointers until accepted. Ownership is transferred only when insertion
into `MeshAdaptive` or `SubMesh` succeeds. Rejected candidates are destroyed
before control returns to the main execution path.

## Release evidence

The maintained Linux sanitizer gate runs:

- the runtime-configuration unit test;
- the complete CLI error-contract test set;
- smoke profiles for Book, Decor Shelf and Eistute;
- AddressSanitizer with leak detection;
- UndefinedBehaviorSanitizer with immediate failure.

Any ownership change must keep this gate clean and preserve the Eistute
stage-4/7,184-face regression. A broader RAII conversion requires dedicated
object-graph tests and belongs to the post-v1 roadmap.
