# Precision Intersection & Dynamic Billet Priming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Phase 1.5 for dynamic IPW0 boundary sampling and upgrade Phase 2 QuadTree intersection logic using a Feature-Aware Local SDF Engine.

**Architecture:** 
1. `LocalSurfaceEngine` (internal to MicroCut) computes a multi-normal half-space SDF using k-NN from existing voxel point data.
2. `MicroCut::primeBilletBoundaries` dynamically samples analytical or mesh billets for `NEW_BOUNDARY` tasks that lack point data.
3. `quadtreeEval` uses the `LocalSurfaceEngine` to discard sampling points that lie outside the local reconstructed IPW material ($SDF_{old} \ge -t$), eliminating "air sampling".

**Tech Stack:** C++17, OpenVDB 11, TBB.

---

### Task 1: Create LocalSurfaceEngine (Feature-Aware Local SDF)

**Files:**
- Create: `core/LocalSurfaceEngine.h`
- Create: `core/LocalSurfaceEngine.cpp`
- Test: `tests/test_localsurface.cpp`

- [ ] **Step 1: Write the failing test**

Create `tests/test_localsurface.cpp`:
```cpp
#include <gtest/gtest.h>
#include "core/LocalSurfaceEngine.h"

using namespace midgard;

TEST(LocalSurfaceEngineTest, MultiNormalSDF) {
    std::vector<Vec3f> pos = { Vec3f(0,0,0), Vec3f(0,0,0) };
    std::vector<Vec3f> nrm = { Vec3f(1,0,0), Vec3f(0,1,0) }; // 90-degree ridge
    
    LocalSurfaceEngine engine(pos, nrm);
    
    // Inside the corner (both planes negative)
    EXPECT_LT(engine.eval(Vec3d(-1, -1, 0)), 0.0);
    
    // Outside the corner (at least one plane positive)
    EXPECT_GT(engine.eval(Vec3d(1, -1, 0)), 0.0);
    EXPECT_GT(engine.eval(Vec3d(-1, 1, 0)), 0.0);
    EXPECT_GT(engine.eval(Vec3d(1, 1, 0)), 0.0);
}
```

- [ ] **Step 2: Modify CMakeLists.txt to include new files**

Modify `CMakeLists.txt`:
```cmake
# Add to midgard library:
core/LocalSurfaceEngine.cpp

# Add to midgard_tests executable:
tests/test_localsurface.cpp
```

- [ ] **Step 3: Run test to verify it fails**
Run: `cd build && cmake .. && make midgard_tests && ./midgard_tests --gtest_filter=LocalSurfaceEngineTest*`
Expected: Compile error (missing header/class).

- [ ] **Step 4: Write minimal implementation**

Create `core/LocalSurfaceEngine.h`:
```cpp
#pragma once
#include "core/Types.h"
#include <vector>

namespace midgard {

class LocalSurfaceEngine {
public:
    LocalSurfaceEngine(const std::vector<Vec3f>& positions, const std::vector<Vec3f>& normals);
    
    // Evaluates multi-normal half-space constraint SDF
    double eval(const Vec3d& p) const;
    
    bool isEmpty() const { return mPositions.empty(); }

private:
    std::vector<Vec3f> mPositions;
    std::vector<Vec3f> mNormals;
    bool mIsRidge;
};

}
```

Create `core/LocalSurfaceEngine.cpp`:
```cpp
#include "core/LocalSurfaceEngine.h"
#include <cmath>

namespace midgard {

LocalSurfaceEngine::LocalSurfaceEngine(const std::vector<Vec3f>& positions, const std::vector<Vec3f>& normals)
    : mPositions(positions), mNormals(normals), mIsRidge(false) 
{
    if (mNormals.size() > 1) {
        double minDot = 1.0;
        for (size_t i = 0; i < mNormals.size(); ++i) {
            for (size_t j = i + 1; j < mNormals.size(); ++j) {
                double d = mNormals[i].dot(mNormals[j]);
                if (d < minDot) minDot = d;
            }
        }
        // 30 degrees = cos(30) ~ 0.866
        if (minDot < 0.866) mIsRidge = true;
    }
}

double LocalSurfaceEngine::eval(const Vec3d& p) const {
    if (mPositions.empty()) return 1e9; // Air
    
    // For simplicity, evaluate against all points and take max.
    // In production, a k-NN would be used for large point sets.
    double maxSdf = -1e9;
    for (size_t i = 0; i < mPositions.size(); ++i) {
        Vec3d diff = p - Vec3d(mPositions[i]);
        double sdf = diff.dot(Vec3d(mNormals[i]));
        if (sdf > maxSdf) maxSdf = sdf;
    }
    return maxSdf;
}

}
```

- [ ] **Step 5: Run test to verify it passes**
Run: `cd build && make midgard_tests -j8 && ./midgard_tests --gtest_filter=LocalSurfaceEngineTest*`
Expected: PASS

- [ ] **Step 6: Commit**
```bash
git add core/LocalSurfaceEngine.* tests/test_localsurface.cpp CMakeLists.txt
git commit -m "feat: add feature-aware LocalSurfaceEngine"
```

---

### Task 2: Implement Phase 1.5 Dynamic Billet Priming

**Files:**
- Modify: `core/MicroCut.h`
- Modify: `core/MicroCut.cpp`
- Test: `tests/test_microcut.cpp`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_microcut.cpp`:
```cpp
TEST_F(MicroCutTest, PrimeBilletBoundaries) {
    ToleranceConfig config(1.0/30.0, ToleranceConfig::INTERACTIVE, 30.0);
    GeometryDef geom{GeometryDef::BOX, Vec3d(0), Vec3d(10, 10, 10)};
    auto ipw = IPWBuilder().build(geom, config);
    
    // Create a mock task for NEW_BOUNDARY
    VoxelTask task;
    task.origin = openvdb::Coord(5, 5, 9); // Z=9 surface
    task.aabb = openvdb::BBoxd(Vec3d(5,5,9), Vec3d(6,6,10));
    task.classification = VoxelClass::NEW_BOUNDARY;
    std::vector<VoxelTask> tasks = { task };
    
    MicroCut microcut;
    auto buffers = microcut.primeBilletBoundaries(tasks, ipw.macroGrid, config);
    
    ASSERT_TRUE(buffers.find(task.origin) != buffers.end());
    EXPECT_GT(buffers[task.origin].positions.size(), 0);
}
```

- [ ] **Step 2: Run test to verify it fails**
Run: `cd build && make midgard_tests && ./midgard_tests --gtest_filter=MicroCutTest.PrimeBilletBoundaries`
Expected: Compile error (primeBilletBoundaries not declared).

- [ ] **Step 3: Write minimal implementation**

Modify `core/MicroCut.h` (add to public section):
```cpp
    std::unordered_map<openvdb::Coord, PointBuffer>
    primeBilletBoundaries(
        const std::vector<VoxelTask>& tasks,
        const openvdb::FloatGrid::Ptr& billetGrid,
        const ToleranceConfig& config);
```

Modify `core/MicroCut.cpp` (implement method):
```cpp
std::unordered_map<openvdb::Coord, PointBuffer>
MicroCut::primeBilletBoundaries(
    const std::vector<VoxelTask>& tasks,
    const openvdb::FloatGrid::Ptr& billetGrid,
    const ToleranceConfig& config)
{
    std::unordered_map<openvdb::Coord, PointBuffer> buffers;
    if (!billetGrid) return buffers;
    
    auto acc = billetGrid->getConstAccessor();
    const auto& xform = billetGrid->transform();
    
    for (const auto& task : tasks) {
        if (task.classification != VoxelClass::NEW_BOUNDARY) continue;
        
        // Voxel center
        Vec3d center = (task.aabb.min() + task.aabb.max()) * 0.5;
        double sdf = acc.getValue(xform.worldToIndex(center));
        
        // If near boundary
        if (std::abs(sdf) < config.voxelMacro) {
            // Simplified sampling: just drop the center point onto the zero-isosurface via gradient
            Vec3d g(
                acc.getValue(xform.worldToIndex(center + Vec3d(1e-4,0,0))) - acc.getValue(xform.worldToIndex(center - Vec3d(1e-4,0,0))),
                acc.getValue(xform.worldToIndex(center + Vec3d(0,1e-4,0))) - acc.getValue(xform.worldToIndex(center - Vec3d(0,1e-4,0))),
                acc.getValue(xform.worldToIndex(center + Vec3d(0,0,1e-4))) - acc.getValue(xform.worldToIndex(center - Vec3d(0,0,1e-4)))
            );
            g.normalize();
            
            Vec3d surfacePoint = center - g * sdf;
            
            if (task.aabb.isInside(surfacePoint)) {
                buffers[task.origin].positions.push_back(Vec3f(surfacePoint));
                buffers[task.origin].normals.push_back(Vec3f(g));
            }
        }
    }
    return buffers;
}
```

- [ ] **Step 4: Run test to verify it passes**
Run: `cd build && make midgard_tests -j8 && ./midgard_tests --gtest_filter=MicroCutTest.PrimeBilletBoundaries`
Expected: PASS

- [ ] **Step 5: Commit**
```bash
git add core/MicroCut.* tests/test_microcut.cpp
git commit -m "feat: implement dynamic billet priming (Phase 1.5)"
```

---

### Task 3: Integrate LocalSurfaceEngine into QuadTree Evaluation

**Files:**
- Modify: `core/MicroCut.h`
- Modify: `core/MicroCut.cpp`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_microcut.cpp`:
```cpp
TEST_F(MicroCutTest, QuadTreeUsesLocalEngine) {
    ToleranceConfig config(1.0/30.0, ToleranceConfig::INTERACTIVE, 30.0);
    ToolDef tool{ToolType::BALL_END, 5.0, 0.0, 20.0};
    MoveSegment seg{Vec3d(0,0,0), Vec3d(0,0,0)};
    ToolSweepSurface surf(tool, seg);
    ToolSweptSDF sdf(tool, seg);
    
    // Simulate a CUT voxel that already has material deep inside
    PointBuffer existingData;
    existingData.positions.push_back(Vec3f(0, 0, -5)); // Old surface at Z=-5
    existingData.normals.push_back(Vec3f(0, 0, 1));
    
    VoxelTask task;
    task.aabb = openvdb::BBoxd(Vec3d(-1,-1,-1), Vec3d(1,1,1)); // Origin around (0,0,0)
    
    PointBuffer output;
    MicroCut microcut;
    // We expect the quadTree to NOT output points because the tool surface (Z=0) 
    // is in the air relative to the old surface (Z=-5). SDF_old(0,0,0) = (0 - -5)*1 = +5 > 0.
    microcut.quadtreeEval(0, 1, 0, 1, task.aabb, surf, config.chordalLimit, 0, output, 
                          nullptr, nullptr, &existingData, config.user_t);
                          
    EXPECT_EQ(output.positions.size(), 0);
}
```

- [ ] **Step 2: Run test to verify it fails**
Run: `cd build && make midgard_tests && ./midgard_tests --gtest_filter=MicroCutTest.QuadTreeUsesLocalEngine`
Expected: Compile error (wrong number of arguments to quadtreeEval).

- [ ] **Step 3: Write minimal implementation**

Modify `core/MicroCut.h` `quadtreeEval` signature:
```cpp
    void quadtreeEval(
        double u0, double u1, double v0, double v1,
        const openvdb::BBoxd& voxelAABB,
        const ToolSweepSurface& surface,
        double chordalLimit,
        int depth,
        PointBuffer& output,
        const openvdb::FloatGrid::ConstAccessor* billetAcc = nullptr,
        const openvdb::math::Transform* billetXform = nullptr,
        const PointBuffer* existingData = nullptr,
        double cullThreshold = 0.01);
```

Modify `core/MicroCut.cpp` `quadtreeEval` implementation:
Update signature and add the engine check before generating points:
```cpp
#include "core/LocalSurfaceEngine.h"

// ... existing quadtreeEval code until output generation ...

    // Replace the billet check inside max depth and chordal branches:
    auto validateAndOutput = [&](const Vec3d& p, const Vec3d& n) {
        if (existingData && !existingData->positions.empty()) {
            LocalSurfaceEngine engine(existingData->positions, existingData->normals);
            double s_old = engine.eval(p);
            if (s_old > -cullThreshold) return; // Point is in air or removed region
        } else if (billetAcc && billetXform) {
            if (billetAcc->getValue(openvdb::Coord::round(billetXform->worldToIndex(p))) > cullThreshold) return;
        }
        output.positions.push_back(Vec3f(p));
        output.normals.push_back(Vec3f(n));
    };

    // In MAX_DEPTH block:
    if (depth >= MAX_DEPTH) {
        // ...
        if (inside) {
            Vec3d n = surface.normal(uMid, vMid);
            validateAndOutput(pCenter, n);
        }
        return;
    }
    
    // In Chordal Limit block:
    if (maxChordal <= chordalLimit) {
        // ...
        if (inside) {
            Vec3d n = surface.normal(uMid, vMid);
            validateAndOutput(pCenter, n);
            return;
        }
        if (depth >= MAX_DEPTH - 2) return;
    }
```

- [ ] **Step 4: Run test to verify it passes**
Run: `cd build && make midgard_tests -j8 && ./midgard_tests --gtest_filter=MicroCutTest.QuadTreeUsesLocalEngine`
Expected: PASS

- [ ] **Step 5: Commit**
```bash
git add core/MicroCut.* tests/test_microcut.cpp
git commit -m "feat: integrate LocalSurfaceEngine into quadtree intersection logic"
```

---

### Task 4: Connect Phase 1.5 in the Main Loop

**Files:**
- Modify: `core/MicroCut.cpp`

- [ ] **Step 1: Wire `primeBilletBoundaries` into `sampleNewSurface`**

Modify `sampleNewSurface` in `core/MicroCut.cpp`:
```cpp
std::unordered_map<openvdb::Coord, PointBuffer>
MicroCut::sampleNewSurface(
    const std::vector<VoxelTask>& tasks,
    const ToolSweepSurface& surface,
    const ToolSweptSDF& sdf,
    const ToleranceConfig& config,
    const openvdb::FloatGrid::Ptr& billetGrid)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    // PHASE 1.5: Prime missing boundaries
    auto primedBuffers = primeBilletBoundaries(tasks, billetGrid, config);

    // Existing parallel_for setup...
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, tasks.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                // ...
                const PointBuffer* existingData = nullptr;
                if (tasks[i].classification == VoxelClass::NEW_BOUNDARY) {
                    auto it = primedBuffers.find(tasks[i].origin);
                    if (it != primedBuffers.end()) existingData = &it->second;
                }
                // NOTE: For CUT voxels, we would ideally fetch the existing points from MicroGrid here.
                // For this implementation, we pass the primed buffers for NEW_BOUNDARY.

                quadtreeEval(tasks[i].u_min, tasks[i].u_max,
                             tasks[i].t_min, tasks[i].t_max,
                             tasks[i].aabb, surface, config.chordalLimit,
                             0, output, accPtr, xformPtr, existingData, config.user_t);
                // ...
```

- [ ] **Step 2: Compile and Run Global Tests**
Run: `cd build && make midgard_tests -j8 && ./midgard_tests`
Expected: All tests PASS.

- [ ] **Step 3: Commit**
```bash
git add core/MicroCut.cpp
git commit -m "feat: wire Phase 1.5 dynamic billet priming into sampleNewSurface"
```
