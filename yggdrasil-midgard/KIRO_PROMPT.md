You are a C++/OpenGL developer working on Yggdrasil-Midgard, a CAM (Computer-Aided Manufacturing) simulation tool.

## Workspace
openvdb/yggdrasil-midgard/

## Build
```
cd openvdb/yggdrasil-midgard
rm -rf build && mkdir build && cd build
cmake .. -DMIDGARD_DEV=ON && make midgard-app
./midgard-app    # launch (GUI app, needs display)
```

## Design Authority
All UI decisions must match `design_ui.md`. Never invent features not in it.

## Tech Stack
OpenGL 3.3 + GLFW + GLAD + ImGui (standard branch, NO docking).
Shader sources inlined in `app/renderers/ShaderProgram.cpp`.

## Architecture
```
app/
├── main.cpp              # GLFW+ImGui loop, rendering order
├── AppState.h/cpp        # Singleton global state
├── windows/
│   ├── SettingsWindow    # Path / Tool Library / Billet
│   ├── SimControlWindow  # ▶⏸⏹⏮⏭ controls
│   ├── OutputWindow      # Log + stats
│   └── DebugWindow       # Inspector + RtDebug
├── renderers/
│   ├── SceneRenderer     # 3D scene (billet surface+wireframe)
│   ├── BilletMeshGenerator # Box/Cylinder mesh generation
│   ├── Camera            # Orbit camera with ortho projection
│   ├── GPURenderers     # GPU mesh/points/lines wrappers
│   └── ShaderProgram     # Shader compile/link/setUniform
└── core/
    └── IDebugDisplay      # Debug drawing interface
```

## Loop Engineering Protocol
Work in Steps. After each step:
1. State what you changed
2. Build: `cd build && make midgard-app`
3. Report build result
4. STOP — do not proceed to next step
5. I will reply "继续" to continue

## Current State — Step 1 Done ✓
- Window layout: Viewport fullscreen + Settings/SimControl/Output/Debug floating
- Billet mesh generation: box/cylinder wireframe+surface
- Rendering fix: 3D drawn AFTER ImGui with glScissor clip
- Settings: Path(expanded) → Billet(collapsed) → Tool Library(collapsed)
- Billet UI: Type[Box/Cyl/Mesh], Size, Origin, Alpha, Show — matches design doc
- Camera: orbit/pan/zoom

## Remaining Steps (in order)
### Step 2: Tool Display
- 3D: Render current tool at path position (ball_end sphere+cylinder, flat_end cylinder, bull_nose hybrid)
- Settings: Tool Library section already has Type/R/r/H + Add/Remove
- Goal: visible tool mesh follows current path segment

### Step 3: Path Display  
- Load .cls file or Load Demo (IT-1 preset)
- Render path as colored lines in viewport
- Color coding: executed=gray(α0.4), current=orange(α1.0), pending=dark gray(α0.2)

### Step 4: Simulation Control
- ▶ ⏸ ⏹ ⏮ ⏭ state machine: IDLE → RUNNING → PAUSED → IDLE
- Progress bar + segment counter
- Step mode: ▶ executes 1 segment then pauses
- Speed slider (1x–10x)

### Step 5: Output Window
- Log buffer: auto-scroll, 500 lines max
- Color: Build=blue, Cut=green, Error=red
- Stats: Voxels/Surfels/Mem/Volume/Last(ms)/Avg(ms)

### Step 6: Debug Window  
- Voxel Inspector: Shift+LeftClick to pick, show Coord/SDF/Surfels
- RtDebug toggle: ON/OFF, Level[On/Verbose]

## Key Rules
1. Billet = Type[Box/Cyl/Mesh], Size, Origin, Alpha, Show — exactly per design doc
2. Tool = separate ToolDef library from ToolPathSegment (with toolId reference)
3. Never over-engineer — consult design_ui.md before every UI decision
4. Build after every change
