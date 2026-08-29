# Requirements Document

## Revision Record

| Version | Date | Summary |
|---|---|---|
| 1.0 | 2026-08-26 | Consolidated review: unified the positive-inside field convention, replaced heuristic CE classification with certified conservative classification, corrected feature/support semantics, and added rigorous classification proofs. |
| 1.a | 2026-08-26 | Final consistency refinement for certified evaluation, verified sampling, feature intersections, query validity, and explicit recovery. |

## Introduction

IPG_C (In-Process Geometry — Compute Representation) is a static machining-target representation for CAM computation. IPG_C reuses the established OpenVDB sparse-grid, `MicroGridCell`, and adaptive surface-sampling concepts used by IPW, while adding immutable links from computational cells to source BREP features.

IPG_C uses one canonical signed-distance field throughout the specification: positive values represent material/interior, negative values represent air/exterior, and zero represents the geometric boundary. This convention matches the existing `MicroGridCell` sign convention. CE classification is conservative: AIR and SOLID are certified whole-cell classifications, while stored BOUNDARY means only that the CE could not be certified as either AIR or SOLID at the available accuracy. A BOUNDARY CE is therefore a boundary candidate and does not imply that a zero crossing exists.

FEATURE faces represent real machining targets. SUPPORT faces may close an otherwise open machining surface into a watertight solid, but SUPPORT geometry is not itself a machining target. FEATURE/SUPPORT roles affect feature-aware queries and adaptive surface sampling; the roles do not change field accuracy, CE topology, or conservative classification thresholds.

## Glossary

- **IPG_C**: In-Process Geometry Compute Representation — the immutable machining-target compute representation specified by this document.
- **IPG_C_Builder**: The component that validates input geometry and constructs IPG_C.
- **IPW**: In-Process Workpiece — the evolving workpiece representation used during cutting simulation.
- **Geometry_Field (φ)**: The canonical exact signed-distance field. `φ(x) > 0` denotes material/interior, `φ(x) < 0` denotes air/exterior, and `φ(x) = 0` denotes the geometric boundary. Geometry_Field is required to be 1-Lipschitz.
- **Geometry_Field_Evaluator**: The interface that returns CERTIFIED or UNCERTIFIED field-evaluation results and, where defined, field gradients.
- **Field_Estimate (φ_hat(x))**: A computed estimate of exact field value `φ(x)` at evaluation point `x`.
- **CERTIFIED**: A field-evaluation result status indicating that the evaluator established the Certified_Result bound.
- **Certified_Result**: A field-evaluation result containing finite `φ_hat(x)` and finite `ε(x) ≥ 0` that satisfy `|φ_hat(x) - φ(x)| ≤ ε(x)`.
- **Certified_Error (ε(x))**: The finite non-negative error bound contained in a Certified_Result.
- **UNCERTIFIED**: A field-evaluation result status used when the evaluator cannot establish the Certified_Result bound; any accompanying numeric estimate is not a Certified_Error.
- **External_Field_Adapter**: An adapter that converts an external geometry-kernel field to the canonical Geometry_Field sign convention.
- **Voxel**: An OpenVDB leaf-scale spatial cell containing `8 × 8 × 8 = 512` CEs when materialized as a `MicroGridCell`.
- **CE**: Computational Element — a closed axis-aligned cube `C` within a Voxel.
- **c**: The center of CE cube `C`.
- **s**: The CE edge length, equal to `voxelSize / 8`.
- **ρ (rho)**: The CE circumradius, equal to `sqrt(3) × s / 2`.
- **MicroGridCell**: The established Voxel payload containing `activeMask[8]`, `signMask[8]`, and a variable-length quantized field vector named `sdf` for compatibility.
- **AIR**: A certified CE state asserting `φ(x) < 0` for every point `x` in the CE.
- **SOLID**: A certified CE state asserting `φ(x) > 0` for every point `x` in the CE.
- **BOUNDARY_CANDIDATE**: A conservative CE state used when AIR or SOLID cannot be certified. The compatible stored state name is BOUNDARY. A BOUNDARY_CANDIDATE may intersect `φ = 0`, may be entirely AIR, or may be entirely SOLID.
- **Navigation_Value**: A quantized, clamped Field_Estimate stored for a BOUNDARY_CANDIDATE to support approximate navigation. A Navigation_Value is not a zero-crossing certificate and is not used to prove AIR or SOLID.
- **Narrow_Band_Half_Width (w)**: The field quantization range, defaulting to `3 × s`.
- **Narrow_Band**: The region `|φ(x)| ≤ w` in which the target evaluator accuracy is assessed.
- **BREP**: Boundary Representation — a CAD boundary model composed of Faces, Edges, and Vertices.
- **Feature_Entity**: A non-degenerate BREP Face, Edge, or Vertex with a stable opaque identifier.
- **Face_Role**: A face-owned designation with value FEATURE or SUPPORT.
- **FEATURE**: A Face_Role identifying a real machining-target surface.
- **SUPPORT**: A Face_Role identifying synthetic geometry used only to close topology.
- **Feature_Precision**: A positive adaptive-sampling precision owned by a BREP Face. The default is global tolerance `t` for FEATURE and `K_support × t` for SUPPORT, with default `K_support = 10`.
- **Inherited_Role**: The set of incident Face_Roles inherited by an Edge or Vertex. An Edge or Vertex is FEATURE-eligible when at least one incident face is FEATURE and is support-only when every incident face is SUPPORT.
- **Entity_Effective_Precision**: The minimum Feature_Precision of all faces incident to a Feature_Entity; a Face uses the Face's own Feature_Precision.
- **CE_Effective_Precision**: The minimum Entity_Effective_Precision among Feature_Entities geometrically intersecting a CE, or global tolerance `t` when no Feature_Entity association exists.
- **Feature_Index**: The immutable bidirectional association between geometrically intersecting BOUNDARY_CANDIDATE CEs and Feature_Entities.
- **SurfaceSample**: A sampled boundary point and an outward unit normal.
- **Outward_Unit_Normal**: Where Geometry_Field is differentiable and `∇φ ≠ 0`, the vector `-normalize(∇φ)`. The minus sign follows from the positive-inside field convention.
- **IPG_C_Query_Service**: The component that performs spatial, state, role-filtered, and feature queries on IPG_C.
- **IPG_Comparison_Service**: The component that compares aligned IPG_C and IPW representations.
- **Comparison_Eligible_CE**: An IPG_C BOUNDARY_CANDIDATE associated with at least one FEATURE entity and selected by the active comparison filter.
- **Genuine_Closed_Design_Solid**: A watertight design solid containing no synthetic SUPPORT faces and no volume created solely for topological closure.
- **Synthetic_Closure_Volume**: Volume whose interior classification exists solely because SUPPORT faces close an otherwise open machining surface.
- **DEGRADED**: An explicit build mode used when the achieved certified narrow-band accuracy is coarser than `t / 10` or when some evaluations remain uncertified.
- **UNRESOLVED**: A read-only state in which serialized Feature_Entity identifiers are available but source BREP geometry is unavailable or unvalidated.
- **RECOVERY**: An explicit deserialization mode that recovers recognized validated chunks from an otherwise unsupported file into a read-only UNRESOLVED state.
- **IPG_C_Serializer**: The component that writes and reads the portable IPG_C file format.
- **Construction_Domain**: The finite, declared world-space region covered by an IPG_C build.
- **Global_Tolerance (t)**: The requested machining tolerance in the supported range `0.001 mm ≤ t ≤ 0.5 mm`.

## Requirements

### Requirement 1: Certified IPG_C Grid Construction

**User Story:** As a CAM algorithm developer, I want machining target geometry converted into a conservative MicroGridCell-compatible sparse representation, so that AIR and SOLID cells are geometrically trustworthy.

#### Acceptance Criteria

1. WHEN Global_Tolerance `t` is provided, THE IPG_C_Builder SHALL derive `voxelSize = clamp(K × t, 0.02 mm, 5.0 mm)` with configurable positive `K` defaulting to 30, derive `s = voxelSize / 8`, and derive `ρ = sqrt(3) × s / 2`.
2. WHEN a Construction_Domain and voxel transform are accepted, THE IPG_C_Builder SHALL construct an OpenVDB sparse representation whose materialized Voxel payloads preserve the established `MicroGridCell` layout and CE indexing.
3. WHEN a CE center evaluation returns a Certified_Result containing `φ_hat(c)` and `ε(c)`, THE IPG_C_Builder SHALL classify the CE as SOLID iff `φ_hat(c) - ε(c) > ρ`, as AIR iff `φ_hat(c) + ε(c) < -ρ`, and as BOUNDARY_CANDIDATE otherwise.
4. IF a CE center evaluation returns UNCERTIFIED, THEN THE IPG_C_Builder SHALL classify the CE as BOUNDARY_CANDIDATE, record the CE as uncertified, and emit a diagnostic instead of classifying the CE as AIR or SOLID.
5. WHEN a BOUNDARY_CANDIDATE has a finite Field_Estimate, THE IPG_C_Builder SHALL store `q = round(clamp(φ_hat(c), -w, w) / w × 32767)` as the compatible quantized Navigation_Value without using `q` as classification evidence.
6. WHEN a BOUNDARY_CANDIDATE lacks a finite Field_Estimate, THE IPG_C_Builder SHALL store a neutral Navigation_Value of zero and preserve the uncertified diagnostic status.
7. WHEN CE states are encoded in a `MicroGridCell`, THE IPG_C_Builder SHALL use `activeMask = 1` for BOUNDARY_CANDIDATE, use `activeMask = 0` and `signMask = 1` for SOLID, and use `activeMask = 0` and `signMask = 0` for AIR.
8. WHEN construction completes successfully, THE IPG_C_Builder SHALL make the grid and Feature_Index immutable and expose no cutting, Boolean subtraction, or topology-changing operation.
9. IF the input contains a degenerate Face, zero-length Edge, isolated Vertex, or zero-area mesh primitive, THEN THE IPG_C_Builder SHALL report the entity identifier and type, skip the degenerate entity, and revalidate the remaining topology.
10. IF the usable input geometry after degenerate-entity removal is not a closed orientable manifold, THEN THE IPG_C_Builder SHALL reject the build and report that upstream closure with explicitly identified SUPPORT faces is required.
11. IF the Construction_Domain or transform cannot map every covered CE to supported grid coordinates without integer overflow, THEN THE IPG_C_Builder SHALL reject the build and report the unsupported extent or resolution.
12. WHEN a benchmark build is executed, THE IPG_C_Builder SHALL report the model identifier, bounding-box extent, tolerance, face count, materialized Voxel count, BOUNDARY_CANDIDATE count, field-evaluation count, feature-association count, build time, and peak memory on both the Mac mini M4 and ThinkPad X1 benchmark platforms.

### Requirement 2: Certified Geometry Field Evaluation

**User Story:** As a CAM algorithm developer, I want BREP and mesh inputs evaluated through one certified positive-inside field interface, so that conservative classification remains valid across geometry sources and accuracy levels.

#### Acceptance Criteria

1. THE Geometry_Field_Evaluator SHALL return exactly one evaluation status, CERTIFIED or UNCERTIFIED, for each requested field evaluation.
2. WHEN the Geometry_Field_Evaluator returns CERTIFIED at `x`, THE Geometry_Field_Evaluator SHALL return finite `φ_hat(x)` and finite `ε(x) ≥ 0` satisfying `|φ_hat(x) - φ(x)| ≤ ε(x)`.
3. IF the Geometry_Field_Evaluator cannot establish `|φ_hat(x) - φ(x)| ≤ ε(x)` with finite `φ_hat(x)` and finite `ε(x) ≥ 0`, THEN THE Geometry_Field_Evaluator SHALL return UNCERTIFIED and SHALL not designate any accompanying numeric estimate as Certified_Error.
4. THE Geometry_Field_Evaluator SHALL provide a normal-support operation returning a field gradient where Geometry_Field is differentiable.
5. THE Geometry_Field_Evaluator SHALL define Geometry_Field with `φ(x) > 0` for material/interior, `φ(x) < 0` for air/exterior, and `φ(x) = 0` on the closed boundary.
6. WHEN an external CAD kernel or mesh library returns a conventional negative-inside signed distance, THE External_Field_Adapter SHALL negate the returned field value and field gradient before IPG_C use and preserve the certified error magnitude.
7. WHEN an outward normal is derived from a differentiable Geometry_Field location with nonzero gradient, THE Geometry_Field_Evaluator SHALL compute the normal as `-normalize(∇φ)`.
8. IF any Narrow_Band evaluation in normal non-DEGRADED mode returns UNCERTIFIED or returns a Certified_Error greater than `t / 10`, THEN THE IPG_C_Builder SHALL reject the build and report the affected evaluations.
9. WHERE explicit DEGRADED mode is selected, WHEN one or more Narrow_Band evaluations return UNCERTIFIED or a Certified_Error greater than `t / 10`, THE IPG_C_Builder SHALL permit construction while preserving each available Certified_Result and reporting achieved certified accuracy, uncertified evaluation count, and affected BOUNDARY_CANDIDATE count.
10. WHEN a triangle mesh is supplied, THE Geometry_Field_Evaluator SHALL validate closed orientable manifold topology and use a spatial acceleration structure whose query cost scales sublinearly with face count for spatially selective queries.
11. IF the supplied mesh fails closed orientable manifold validation, THEN THE Geometry_Field_Evaluator SHALL reject the mesh and report the detected boundary or non-manifold topology.
12. WHERE parallel construction is enabled, THE Geometry_Field_Evaluator SHALL support race-free evaluation across independent Voxels and produce classifications satisfying the same certificates as serial evaluation.
13. WHEN serial and parallel benchmark builds use the same model, transform, tolerance, and evaluator configuration, THE IPG_C_Builder SHALL report throughput and scaling without requiring a fixed speedup ratio.

### Requirement 3: Adaptive Surface Sampling

**User Story:** As a CAM algorithm developer, I want conservative boundary candidates sampled on demand at feature-aware precision, so that downstream computations obtain useful surface points without assuming every candidate contains a surface.

#### Acceptance Criteria

1. WHEN a BOUNDARY_CANDIDATE surface query begins, THE IPG_C SHALL compute CE_Effective_Precision as the minimum positive precision among geometrically intersecting Feature_Entities, or as `t` when no Feature_Entity association exists, and SHALL enforce `0 < CE_Effective_Precision ≤ configured SUPPORT precision`.
2. WHEN adaptive sampling is invoked for a CE with edge length `s` and effective precision `p`, THE IPG_C SHALL configure refinement with `chordalTol = p` and `maxDepth = max(0, ceil(log2(s / p)))`.
3. WHILE a surface-sample query is actively evaluating a support-only CE, THE IPG_C SHALL use the inherited SUPPORT precision only for adaptive sampling.
4. THE IPG_C SHALL preserve identical Geometry_Field accuracy requirements and conservative CE classification thresholds for FEATURE, SUPPORT, and mixed-role CEs.
5. WHEN a SurfaceSample at position `x` is returned with effective precision `p`, THE IPG_C SHALL provide a Certified_Result satisfying `|φ_hat(x)| + ε(x) ≤ p / 10`, thereby proving `|φ(x)| ≤ p / 10`, and an outward normal with length error no greater than `1e-6`.
6. IF Geometry_Field is non-differentiable or has an unusable gradient at a returned sample position, THEN THE IPG_C SHALL derive an outward normal from a tolerance-bounded incident BREP face or report that no reliable sample normal is available.
7. IF no candidate at `maxDepth` satisfies both the certified residual requirement and the reliable-normal requirement at precision `p`, THEN THE IPG_C SHALL return zero SurfaceSamples as a successful result with explicit status `not verified at precision p`, preserve the CE state, and report that true-surface existence remains undetermined.
8. WHERE precomputation is enabled, THE IPG_C_Builder SHALL cache SurfaceSamples using the same query semantics and effective precision rules as on-demand sampling.
9. WHILE IPG_C operates without a precomputed sample cache, THE IPG_C SHALL retain access to a compatible Geometry_Field_Evaluator for each surface-sample query.
10. WHEN a tighter certified interval becomes available for a BOUNDARY_CANDIDATE before immutable publication, THE IPG_C_Builder SHALL permit reclassification to certified AIR or SOLID only according to the Requirement 1 inequalities.
11. IF a tighter certified interval becomes available after immutable publication, THEN THE IPG_C SHALL place any reclassification in a separately versioned derived result and preserve the published IPG_C unchanged.

### Requirement 4: Complete Feature Anchoring

**User Story:** As a CAM algorithm developer, I want every relevant boundary cell linked completely to the BREP entities that truly intersect the cell, so that analytical geometry remains traceable without silent association loss.

#### Acceptance Criteria

1. WHEN a BREP Face is ingested, THE Feature_Index SHALL record the Face-owned FEATURE or SUPPORT role and the Face's positive Feature_Precision.
2. WHEN an Edge or Vertex is ingested, THE Feature_Index SHALL derive Entity_Effective_Precision as the minimum precision of incident Faces and inherit all incident Face_Roles.
3. IF a Face has a non-positive Feature_Precision or a SUPPORT precision coarser than the configured SUPPORT precision limit, THEN THE IPG_C_Builder SHALL reject the precision configuration and identify the Face.
4. THE IPG_C_Builder SHALL require every confirmed Feature_Entity-versus-CE intersection to map to a BOUNDARY_CANDIDATE in the complete Feature_Index.
5. IF a final exact or tolerance-bounded entity-versus-CE test confirms an intersection with a CE certified as AIR or SOLID, THEN THE IPG_C_Builder SHALL treat the result as a certificate/geometry inconsistency, fail the build or publication of the affected derived result without replacing the certified state, and report the CE, Feature_Entity identifier, Field_Estimate, Certified_Error, and applied intersection tolerance.
6. WHERE an expanded entity AABB is used for broad-phase selection, THE IPG_C_Builder SHALL confirm every final Feature_Index association with an exact or tolerance-bounded entity-versus-CE intersection test.
7. THE Feature_Index SHALL contain only associations confirmed by the final entity-versus-CE intersection test.
8. THE Feature_Index SHALL maintain the bidirectional invariant that every CE-to-Feature association has one corresponding Feature-to-CE association and every Feature-to-CE association has one corresponding CE-to-Feature association.
9. WHEN the number of valid associations exceeds inline or configured memory capacity, THE Feature_Index SHALL use complete variable-length or overflow storage without truncating or dropping associations.
10. IF complete Feature_Index storage exceeds a configured build resource limit, THEN THE IPG_C_Builder SHALL warn the caller and require explicit continuation or fail the build without publishing a partial Feature_Index.
11. IF a BREP entity is degenerate, THEN THE Feature_Index SHALL omit the entity consistently with Requirement 1 and preserve the associated diagnostic.
12. WHEN the input is a mesh without BREP topology, THE Feature_Index SHALL remain empty and return empty feature result sets without error.
13. WHEN Feature_Index construction completes, THE Feature_Index SHALL remain immutable with the published IPG_C.

### Requirement 5: Spatial and Feature Queries

**User Story:** As a toolpath planner, I want predictable state, region, role, and feature queries, so that machining computations can distinguish real target geometry from synthetic closure geometry.

#### Acceptance Criteria

1. WHEN a point in the Construction_Domain is queried, THE IPG_C_Query_Service SHALL return the containing CE state, Navigation_Value availability, certification status, and associated Feature_Entities through a bounded-depth grid lookup and CE index computation.
2. WHEN an AABB region is queried, THE IPG_C_Query_Service SHALL return the materialized non-AIR CEs intersecting the region using sparse-tree traversal with cost proportional to traversal overhead plus returned data.
3. WHEN a Feature_Entity identifier is queried, THE IPG_C_Query_Service SHALL return all associated BOUNDARY_CANDIDATE CEs through the Feature_Index reverse mapping.
4. WHERE a FEATURE, SUPPORT, FEATURE-eligible, or support-only filter is specified, THE IPG_C_Query_Service SHALL return only associations and SurfaceSamples matching the requested inherited role semantics.
5. IF a point or AABB query contains a non-finite coordinate or an AABB has any minimum coordinate greater than the corresponding maximum coordinate, THEN THE IPG_C_Query_Service SHALL reject the query as invalid.
6. IF a valid point lies outside the Construction_Domain, THEN THE IPG_C_Query_Service SHALL return an out-of-domain result distinct from AIR and from an empty result.
7. IF a valid point, AABB, or Feature_Entity query has no actual matching data, THEN THE IPG_C_Query_Service SHALL return the applicable empty result set without error.
8. IF matching data exists but is UNRESOLVED, THEN THE IPG_C_Query_Service SHALL return opaque identifiers and unresolved status rather than an empty result.

### Requirement 6: IPW Interoperability and Target-Aware Comparison

**User Story:** As a simulation engineer, I want IPG_C and IPW compared in a shared frame with explicit FEATURE eligibility, so that synthetic closure geometry cannot create false stock or gouging reports.

#### Acceptance Criteria

1. WHEN IPG_C and IPW are prepared for direct correspondence, THE IPG_Comparison_Service SHALL verify identical world-to-index transform, voxelSize, origin, axis orientation, and CE indexing.
2. IF any required transform or resolution component differs, THEN THE IPG_Comparison_Service SHALL reject direct comparison and report each mismatch without silent resampling.
3. THE IPG_Comparison_Service SHALL restrict default machining allowance and gouging evaluation to Comparison_Eligible_CEs and FEATURE SurfaceSamples.
4. THE IPG_Comparison_Service SHALL exclude support-only surfaces and Synthetic_Closure_Volume from default machining-target violation reports.
5. WHEN corresponding comparison-eligible IPG_C and IPW CEs are both BOUNDARY_CANDIDATE, THE IPG_Comparison_Service SHALL automatically compute `Δφ = φ_IPW - φ_IPG` from compatible decoded Navigation_Values and label `Δφ` as a raw field diagnostic without assigning stock or gouging semantics from the sign of `Δφ` alone.
6. WHEN an IPW surface point `q` is matched within a configured correspondence tolerance to a FEATURE target sample `p` with outward normal `n_out`, THE IPG_Comparison_Service SHALL compute the surface-oriented deviation `d_n = (q - p) · n_out` and classify `d_n` above positive tolerance as remaining stock, `d_n` below negative tolerance as gouging, and values within tolerance as matched.
7. IF a reliable FEATURE sample, outward normal, or IPW correspondence is unavailable, THEN THE IPG_Comparison_Service SHALL return an indeterminate comparison result with a diagnostic instead of deriving stock or gouging from CE state alone.
8. WHERE the IPG_C input is a Genuine_Closed_Design_Solid, THE IPG_Comparison_Service SHALL permit direct all-CE state comparison in addition to FEATURE-oriented comparison.
9. IF the IPG_C input contains any synthetic SUPPORT face, THEN THE IPG_Comparison_Service SHALL disable direct all-CE machining-violation classification by default and require an explicit non-machining diagnostic mode to expose raw all-CE differences.
10. WHEN direct all-CE diagnostic comparison is permitted, THE IPG_Comparison_Service SHALL compare AIR and SOLID through state bits and BOUNDARY_CANDIDATE values through decoded Navigation_Values with constant work per corresponding CE.

### Requirement 7: Portable Serialization and Safe Recovery

**User Story:** As a CAM system integrator, I want IPG_C persisted portably with safe version handling and recoverable identifiers, so that sessions can be resumed across macOS ARM64 and Windows x86_64 without unsafe geometry use.

#### Acceptance Criteria

1. THE IPG_C_Serializer SHALL write a portable chunked file containing grid masks, quantized Navigation_Values, classification and DEGRADED metadata, construction parameters, transforms, Face_Roles, Feature_Precisions, Feature_Index mappings, optional SurfaceSamples, and opaque Feature_Entity identifiers.
2. WHEN an IPG_C file is written, THE IPG_C_Serializer SHALL encode numeric fields with fixed-width types and little-endian byte order.
3. WHEN a supported file is serialized and deserialized, THE IPG_C_Serializer SHALL preserve grid masks and quantized values bit-for-bit, preserve Feature_Index associations and role metadata exactly, and preserve cached sample positions within `1e-10 mm` and normal angular deviation within `1e-6 rad`.
4. THE IPG_C_Serializer SHALL include a file-format version and the minimum reader version required by the file.
5. IF the file version or required capabilities exceed reader support during normal loading, THEN THE IPG_C_Serializer SHALL reject the file without entering RECOVERY and report the minimum required reader version.
6. WHEN a supported file is loaded on macOS ARM64 or Windows x86_64, THE IPG_C_Serializer SHALL produce equivalent states, mappings, roles, parameters, and certification metadata without platform-specific conversion by the caller.
7. WHERE explicit RECOVERY mode is requested for an unsupported-version file, THE IPG_C_Serializer SHALL validate and recover only recognized chunks, ignore unknown chunks, and publish the recovered data only in read-only UNRESOLVED state.
8. WHILE recovered data remains UNRESOLVED and unvalidated, THE IPG_C SHALL permit state and grid inspection and SHALL reject geometry computation, surface sampling, and machining comparison.
9. IF source BREP geometry is unavailable for an otherwise supported file, THEN THE IPG_C_Serializer SHALL load grid and Feature_Index data with opaque identifiers in UNRESOLVED state while preserving state, grid, role, and identifier queries.
10. IF an analytical feature operation requires unresolved BREP geometry, THEN THE IPG_C SHALL fail the operation and identify each unresolved Feature_Entity identifier required by the operation.
11. WHEN recovered or unresolved data is validated against compatible source geometry or migrated to a supported format, THE IPG_C_Serializer SHALL require successful integrity and association validation before enabling geometry computation.

## Appendix A: Proof of Conservative CE Classification

### A.1 Definitions and Assumptions

Let `C` be a closed cubic CE with center `c`, edge length `s`, and circumradius

`ρ = sqrt(3) × s / 2`.

For every `x ∈ C`, the center-to-point distance satisfies

`||x - c|| ≤ ρ`.

Let `φ` be the exact signed-distance field with positive-inside convention. Let the evaluator return a Certified_Result at `c` containing finite `φ_hat(c)` and finite `ε(c) ≥ 0` satisfying

`|φ_hat(c) - φ(c)| ≤ ε(c)`.

Equivalently,

`φ_hat(c) - ε(c) ≤ φ(c) ≤ φ_hat(c) + ε(c)`.

The classification rules are:

- SOLID iff `φ_hat(c) - ε(c) > ρ`.
- AIR iff `φ_hat(c) + ε(c) < -ρ`.
- BOUNDARY_CANDIDATE otherwise.

Strict inequalities are required for AIR and SOLID certificates. Equality remains BOUNDARY_CANDIDATE.

### A.2 1-Lipschitz Property

The required exact Geometry_Field satisfies

`|φ(x) - φ(y)| ≤ ||x - y||`

for all points `x` and `y`.

For points on the same side of the boundary, this follows from the standard distance-to-a-closed-set inequality. For points on opposite sides, the segment joining the points crosses the closed boundary at some point `z`; therefore `|φ(x)| ≤ ||x - z||` and `|φ(y)| ≤ ||y - z||`, giving

`|φ(x) - φ(y)| = |φ(x)| + |φ(y)| ≤ ||x - z|| + ||z - y|| = ||x - y||`.

The CE proofs below require this property. An approximate field that is not itself 1-Lipschitz is acceptable only when the certified interval still bounds the exact 1-Lipschitz Geometry_Field at each classification center.

### A.3 SOLID Certificate

Assume

`φ_hat(c) - ε(c) > ρ`.

For any `x ∈ C`, the error certificate and 1-Lipschitz property give

`φ(x) ≥ φ(c) - ||x - c||`

`φ(x) ≥ φ_hat(c) - ε(c) - ||x - c||`

`φ(x) ≥ φ_hat(c) - ε(c) - ρ > 0`.

Therefore every point in `C` is inside/material, so the SOLID classification has no false-positive whole-cell classification.

### A.4 AIR Certificate

Assume

`φ_hat(c) + ε(c) < -ρ`.

For any `x ∈ C`, the error certificate and 1-Lipschitz property give

`φ(x) ≤ φ(c) + ||x - c||`

`φ(x) ≤ φ_hat(c) + ε(c) + ||x - c||`

`φ(x) ≤ φ_hat(c) + ε(c) + ρ < 0`.

Therefore every point in `C` is outside/air, so the AIR classification has no false-positive whole-cell classification.

### A.5 Completeness for Boundary Intersections

Assume that `C` intersects the geometric boundary. Then there exists `z ∈ C` with `φ(z) = 0`.

If `C` satisfied the SOLID certificate, Section A.3 would imply `φ(z) > 0`, which contradicts `φ(z) = 0`. If `C` satisfied the AIR certificate, Section A.4 would imply `φ(z) < 0`, which also contradicts `φ(z) = 0`.

Therefore every CE intersecting `φ = 0` must be classified as BOUNDARY_CANDIDATE. Because every valid Face, Edge, and Vertex lies on the BREP boundary, every CE geometrically intersecting a valid Feature_Entity must also be a BOUNDARY_CANDIDATE.

### A.6 Conservative Meaning of BOUNDARY_CANDIDATE

BOUNDARY_CANDIDATE is the logical complement of the two sufficient certificates. Failure to prove `φ > 0` over the entire CE and failure to prove `φ < 0` over the entire CE do not prove that `φ = 0` occurs in the CE.

Consequently, a BOUNDARY_CANDIDATE may have any of the following geometries:

1. The zero surface intersects the CE.
2. The CE is entirely inside, but the available interval is too wide to certify SOLID.
3. The CE is entirely outside, but the available interval is too wide to certify AIR.
4. The evaluation is uncertified or non-finite.

Therefore a surface query may validly return zero SurfaceSamples. The stored Navigation_Value cannot strengthen the classification because quantization and clamping do not provide a certified bound.

### A.7 Refinement Monotonicity

Represent the center certificate as the interval

`I = [φ_hat(c) - ε(c), φ_hat(c) + ε(c)]`.

A refined evaluation is certificate-monotone when the refined interval

`I' = [φ_hat'(c) - ε'(c), φ_hat'(c) + ε'(c)]`

is valid for the same exact `φ(c)` and satisfies `I' ⊆ I`. For an unchanged estimate, `ε'(c) ≤ ε(c)` is sufficient to establish interval containment.

If a BOUNDARY_CANDIDATE receives a narrower valid interval, the refined lower endpoint may become greater than `ρ`, certifying SOLID, or the refined upper endpoint may become less than `-ρ`, certifying AIR.

If an original interval certifies SOLID, then `inf(I) > ρ`. Any contained refined interval satisfies `inf(I') ≥ inf(I) > ρ`, so the SOLID certificate remains valid. If an original interval certifies AIR, then `sup(I) < -ρ`. Any contained refined interval satisfies `sup(I') ≤ sup(I) < -ρ`, so the AIR certificate remains valid.

A newly computed evaluator result with a smaller numeric `ε'` but an inconsistent or non-contained interval does not establish classification monotonicity. The original AIR or SOLID geometric conclusion remains valid only while the original certified bound remains valid; arbitrary inconsistent evaluator outputs must be diagnosed rather than used to revoke or contradict a valid certificate.

### A.8 Sign and Normal Consequences

Because Geometry_Field is positive inside, moving from the boundary in the outward direction decreases `φ`. At a differentiable boundary point with nonzero gradient, `∇φ` points toward increasing field values and therefore points inward. The outward unit normal is consequently

`n_out = -normalize(∇φ)`.

For aligned IPG_C and IPW fields, `Δφ = φ_IPW - φ_IPG` is a difference between two positive-inside scalar estimates. The sign of `Δφ` at a CE center does not by itself prove a surface offset direction or a machining violation. Stock and gouging classification therefore uses a FEATURE surface point, a verified outward normal, and an IPW surface correspondence as specified in Requirement 6.
