# NVIDIA Cosmos-Transfer2.5-2B for Orbital Trajectory Video & Sim-to-Real Data Generation

> **Bottom line for `mjorbit`:** **Yes — Cosmos-Transfer2.5-2B is worth testing, with two sharp guardrails.** First, its official **~65.4 GB single-GPU VRAM** requirement means inference and LoRA fine-tuning must run via NVIDIA NIM or cloud 80 GB GPUs (not our local **8 GB RTX 4060 Laptop**). Second, while zero-shot or fine-tuned Cosmos is promising for **photorealistic full-mission videos**, **far-range `bus_cam` augmentation**, and **domain-shift stress tests**, it should **not** be trusted blindly for **sub-millimeter `wrist_cam` / `wrist_cam2` stereo supervision** without an explicit epipolar and edge-alignment gate — **Blender Cycles PBR** remains the primary precision engine for exact stereo geometry, while **Blender + Real Space Footage → Cosmos Orbital LoRA** unlocks the scalable photorealistic video synthesizer you described.

---

## Executive Summary & Verdict for mjorbit

### 1.1 Why We Are Investigating Cosmos-Transfer2.5-2B Now

Project 7 (`projects/p7_unprepared_client`) already has a complete orbital physics and optical baseline: `MjorbitBackend` runs full LEO dynamics (Encke + J2 + gravity gradient) and renders Earth, tangent-limb atmosphere, starfields, Kapton MLI, brushed aluminum `adapter_ring`, and triple-junction solar cells alongside camera optics (`sim/optics.py`). Yet MuJoCo's fixed-function OpenGL rasterizer still leaves an unmistakable synthetic look: planar shading, simplified specular highlights, and zero multi-bounce golden MLI reflections onto the stainless adapter ring and gripper jaws.

Your goal is two-fold: (1) feed **recorded `mjorbit` trajectories** (such as `runs/replay_p7_earth_g3`, 591 frames with full body poses, Earth nadir vectors, and sun angles) into a generative world model to synthesize **realistic orbital videos**, and (2) examine whether a staged bootstrapping loop — rendering physically based **Blender Cycles** sequences or curating **existing real/HIL docking videos** to **fine-tune (post-train) Cosmos-Transfer2.5-2B** — can turn `mjorbit`'s trajectories into a high-realism visual data factory for future perception models (`BusNet`, `StereoNet`, and vision-based GNC).

### 1.2 Summary Verdict by Use Case

| Use Case in `mjorbit` | Verdict | Why & Key Constraint | Recommended Path |
|---|---|---|---|
| **Photorealistic mission replay videos** (`third`-person orbital views) | **GO (High Value)** | Temporal consistency + Earth/MLI lighting transfer without needing sub-mm pixel labels | Export 93-frame constant-FPS `vis` + `depth` + `edge` clips -> cloud/NIM inference |
| **Far-range `bus_cam` data & stress-testing** (`8–35 m`, `224×224` / `512×512`) | **GO with Gate** | Coarse center-of-brightness / bearing tolerance (`9–11 cm` in `BusNet_orb`) tolerates mild texture shifts | Generate domain variants (eclipse, glare, limb scatter); validate `BusNet_orb` on them |
| **Close-range `wrist_cam` + `wrist_cam2` stereo (`StereoNet_orb`)** | **CAUTION (Do Not Blindly Mix)** | `StereoNet_orb` reaches `0.61 mm` radial p50; 1–2 px generative drift corrupts pose labels; single-view runs break stereo epipolar lock | Use **Blender Cycles PBR** directly for `StereoNet` labels; test `robot-multiview` LoRA later |
| **Local execution on RTX 4060 Laptop (8 GiB VRAM)** | **NO-GO (Use Cloud/NIM)** | Official single-GPU BF16 requirement is **~65.4 GB VRAM**; 8 GB CPU-offload risks PCIe stalls & laptop's `86–87 °C` thermal shutdown | Run **Blender Cycles PBR** locally with `--duty 0.5`; run Cosmos on **NIM / cloud 80 GB A100/H100** |
| **Hybrid Blender PBR + Real Space -> Cosmos LoRA** | **GO (Best Long-Term Play)** | Pairs Blender's exact 3D materials + real ISS / ADRAS-J / SHIRT / SPEED+ look with fast trajectory synthesis | Curate 10–30 93-frame orbital clips -> train `bf16` LoRA on 1× 80 GB GPU |

> **Key Distinction:** Do not evaluate a generative video model only by how convincing a single trailer clip looks to human eyes. In `mjorbit`, cameras are metrology instruments: `BusNet` guides 30 m -> 8 m rendezvous (`1.2 m` lidar gate), and `StereoNet` guides `0.15 m` ring capture at **sub-millimeter** precision. Visual realism without geometric lock helps human demos and stress testing, whereas visual realism *plus* sub-pixel geometric lock is required for supervised training labels.

---

## Foundations: World Foundation Models & Video-to-Video Transfer

Before diving into `Cosmos-Transfer2.5-2B`'s exact tensor flow and scripts, it helps to build intuition from first principles so every architectural knob (`93 frames`, `VAE`, `DiT`, `control_weight`, `DCP`, `LoRA`) makes concrete engineering sense.

### 2.1 From Still Image Generators to World Foundation Models

Standard image diffusion models (such as Stable Diffusion) map random 2D noise into a single image frame conditioned on text. If you feed 100 individual simulation frames through an image-to-image ControlNet independently, each frame looks plausible on its own, **but the video violently flickers**: solar glare jumps frame to frame, Kapton crinkle patterns morph randomly every 33 ms, and metallic bolts appear and disappear. For a spacecraft tracker like `FarNavTracker` or `StereoNet`, temporal flicker is catastrophic.

A **World Foundation Model (WFM)** learns spatiotemporal dynamics across **time AND space** simultaneously. Instead of denoising `H × W` images one by one, it denoises a 3D video volume `T × H × W` so surfaces, shadows, reflections, and rigid spacecraft parts stay coherent from frame `t` to `t + 1`. NVIDIA's **Cosmos** platform splits this capability into:
- **Cosmos-Predict2.5:** Generative world *prediction* — imagines future video frames given text, an initial image, or a short video prefix ("given frame 0..t, hallucinate plausible motion forward").
- **Cosmos-Transfer2.5:** Conditional world *translation* (sim-to-real / real-to-real) — takes known trajectory geometry (`depth`, `edge`, `seg`, blurred `vis`) from a physics simulator or real camera and re-renders its pixels with photorealistic materials, lighting, and sensor physics while obeying the input motion.

For `mjorbit`, we already trust MuJoCo + `mjorbit_warp` for rigid-body dynamics, Encke orbital tides, and contact physics. Therefore we specifically want **Cosmos-Transfer2.5**, not unconstrained `Predict`: we never want an AI model inventing spacecraft trajectories, we want it rendering realistic imagery *locked* to our physics trajectories.

### 2.2 The Three Building Blocks Inside a Latent Video Transfer Pipeline

Modern video diffusion models never run iterative denoising directly on raw `1280 × 720` RGB pixels — doing attention over `93 × 1280 × 720 ≈ 85.7 million` pixels per step would be impossibly slow. Instead, Cosmos-Transfer2.5 operates in a compressed **3D latent space**:

1. **Spatiotemporal Tokenizer (3D Causal VAE):** Compresses raw video pixels down by **8× spatially** and **4× temporally** (following Wan2.1 / AutoencoderKLWan-style causal 3D autoencoder design). A `93 × 720 × 1280` RGB clip compresses into a compact latent grid of `(1 + 92/4) × (720/8) × (1280/8) = 24 × 90 × 160` latent patches. "Causal" means frame `t`'s latent representation depends only on frames `<= t`, avoiding non-physical backward bleed and supporting autoregressive chunk stitching.
2. **Diffusion Transformer (DiT) Core Trunk:** Replaces old U-Net backbones with a stack of transformer blocks operating on spatiotemporal spacetime patches (tokens). Cross-attention injects semantic descriptions from a **text / Vision-Language encoder** (`Qwen2.5-VL` / Cosmos-Reason in the 2.5 stack), telling the denoiser *what* is in the scene (e.g., "crinkled gold Kapton foil, sunlit brushed aluminum ring, dark deep space, curving Earth horizon").
3. **Multi-ControlNet Spatial Conditioning Branches:** Parallel lightweight branches receive encoded `depth`, `edge`, `seg`, and blurred `vis` streams and inject residual features into the DiT trunk during every denoising step. This anchors 3D silhouette, camera perspective, and part boundaries so the DiT is only free to synthesize surface micro-texture, specular lighting, glare, and background atmosphere.

---

## Deep Dive into NVIDIA Cosmos-Transfer2.5-2B

### 3.1 Parameter Scale & Distributed Control Architecture

Contrary to what "smaller model" sounds like, shrinking from **Cosmos-Transfer1-7B** to **Cosmos-Transfer2.5-2B** was an intentional reliability upgrade:
- **Exact Base Scale:** `nvidia/Cosmos-Transfer2.5-2B` uses **2,358,047,744 parameters** (~2.36B) in its core Diffusion Transformer trunk — roughly **3.5× smaller** than Cosmos-Transfer1-7B. Smaller capacity paired with cleaner Predict2.5 pre-training yields faster throughput and measurably **fewer long-horizon object hallucinations**.
- **Distributed Multi-ControlNet (Key Architectural Change):** In Cosmos-Transfer1, four modality control blocks were stacked sequentially at the *front* of the network, meaning control signals decayed or fought each other deeper in long sequences. **Cosmos-Transfer2.5 evenly interleaves its 4 control blocks across the trunk — inserting one control block after every 7 main DiT blocks.** Conditioning therefore refreshes continuously from coarse layout down to fine surface details.

[[fig:arch|Cosmos-Transfer2.5-2B architecture: 3D Causal VAE compresses raw video and 4 spatial controls into latent space, while 4 distributed ControlNet blocks inject guidance after every 7 DiT trunk blocks.]]

### 3.2 Why a "2B" Model Still Demands 65.4 GB VRAM

A frequent source of confusion is seeing "2B parameters" and assuming it runs like a 2B LLM on an 8 GB GPU. In BF16, 2.4B weights alone are only `~4.7 GB`, so where does **~65.4 GB VRAM** come from? Four simultaneous memory consumers stack at runtime:
- **Auxiliary Multi-Control Branches & Encoders:** Besides the 2.4B main DiT, four ControlNet branches plus the **Qwen2.5-VL (`reason1p1_7B`)-class text/caption stack** and optional safety guardrail models sit in GPU memory unless explicitly offloaded or disabled.
- **3D Causal VAE Decode/Encode Scratchpads (~15–25 GB):** Decoding `93` full `1280 × 720` frames through 3D spatial-temporal convolutions expands latent activations back into hundreds of megabytes per intermediate feature map; 3D conv scratchpads spike sharply at 720p.
- **Full Spatiotemporal Self-Attention Maps:** Computing attention across `24 × 90 × 160` tokens (with patchification) and maintaining rolling KV/context buffers across 93 frames scales super-linearly with resolution and frame count.
- **Multiple Control Tensors:** Each active modality (`edge`, `depth`, `seg`, `vis`) passes through the VAE / control encoder to supply aligned latent conditioning tensors throughout denoising.

### 3.3 Supported Modalities, Multi-View & Weight Normalization

Cosmos-Transfer2.5-2B accepts four primary single-view control modalities plus specialized multi-camera checkpoints in `nvidia-cosmos/cosmos-transfer2.5`:

| Modality / Variant | Hugging Face / Config Key | What It Controls | Role for `mjorbit` |
|---|---|---|---|
| **Canny / Structural Edge** | `general/edge` (`"edge"`) | Hard silhouettes, seams, bolt lines, adapter ring rim | **Critical** — prevents 3D rim/wing drift |
| **Depth Map** | `general/depth` (`"depth"`) | 3D surface proximity and foreground/background separation | **Critical** — locks range & bus vs Earth separation |
| **Semantic Segmentation** | `general/seg` (`"seg"`) | Distinct per-part regions (bus MLI vs aluminum ring vs wings vs Earth) | **High** — stops gold foil bleeding onto aluminum ring |
| **Blurred RGB / Visual** | `general/vis` (`"vis"`) | Low-frequency lighting, color palette, rough shading direction | **High** — preserves crisp sun-angle / eclipse shadow geometry |
| **Multi-View Robotics / Auto** | `robot-multiview` / `multiview` | Cross-camera view consistency (up to 7 cameras, 720p) | **Research Candidate** for synchronized `bus_cam` + stereo pairs |

Two crucial configuration rules from the official `controlnet_specs` mechanics directly impact how we tune `mjorbit`:
1. **Control Weight Normalization:** You assign a float `control_weight` to each modality. If `sum(weights) <= 1.0`, weights apply directly as specified. If `sum(weights) > 1.0`, Cosmos **automatically rescales them proportionally to sum to `1.0`** (e.g., `edge: 1.0, depth: 0.8, seg: 0.4, vis: 0.6` -> normalized `~0.36 / 0.29 / 0.14 / 0.21`). Knowing this avoids silently diluting geometric lock when adding extra controls.
2. **Spatiotemporal Masks (`mask_path`):** Each control entry optionally accepts a binary video mask controlling *where* and *when* conditioning overrides or yields to free synthesis (e.g., ultra-strict `edge + depth` on the `adapter_ring` pixels, looser generation on the Earth limb and starfield).

### 3.4 Resolution, Chunking, Distilled Models & Licensing

- **Resolution & Aspect Ratio:** Native target is **720p (`1280 × 720`, 16:9)**; consumer/single-GPU workarounds regularly step down to **480p** inside `spec.json` to slash 3D VAE activation spikes. Square MuJoCo renders (`384×384`, `224×224`, `192×192`, `512×512`) must either be re-rendered in native 16:9 wide FOV (for `third`) or padded/composited carefully so intrinsic focal lengths stay calibrated.
- **Chunking (`93 Frames`):** The architecture is natively tuned to multiples of **93 frames** (`93, 186, 279, ...`). At `15–16 fps`, 93 frames spans `~5.8–6.2 s` of physical time; at `24–30 fps`, `~3.1–3.9 s`. NVIDIA also provides **distilled** variants operating specifically on 93-frame chunks with fewer denoising steps for faster turnaround.
- **License & Weight Access:** Released under the **NVIDIA Open Model License**. Commercial and non-commercial use are permitted, creating and distributing **Derivative Models (including LoRA fine-tunes)** is allowed, and **NVIDIA claims zero ownership over generated synthetic outputs**. Weights live at Hugging Face (`nvidia/Cosmos-Transfer2.5-2B`, gated behind license click-through + `huggingface-cli`), code lives at `github.com/nvidia-cosmos/cosmos-transfer2.5`, and managed containers/APIs run via **NVIDIA NIM / `build.nvidia.com`**.

---

## Bridging Recorded mjorbit Trajectories to Cosmos

The encouraging discovery from auditing `projects/p7_unprepared_client/` is that **90% of the trajectory plumbing already exists**.

### 4.1 What `record_replay.py` and `SensorSuite` Already Record

Inspecting `viz/record_replay.py` and `gnc/sensors.py` shows that our existing replay directory (`runs/replay_p7_earth_g3`, `591` frames, `complete`, `0` collisions) captures far more than flat JPEGs:
- **`scene3d.json`:** Full scene geometry hierarchy — `10` named bodies (`track_bids` from `render_km`, excluding `mjo_earth`/`mjo_atmo`), `134` geoms, `89` triangle meshes, local transforms, colors, and material tags (`mli`, `solar`, `metal`, `radiator`, `dish`, `nozzle`).
- **`manifest.json` Frame Telemetry:** Every recorded frame stores `chaser` pose `[x,y,z,qw,qx,qy,qz]`, `client` pose, full per-body poses (`bodies`), arm joint points (`arm`), orbital radial vector `r_hat`, exact `sun` vector, `eclipse` status, `phase`, and `range_m`.
- **Native MuJoCo Multi-Modal Renderers:** `gnc/sensors.py` (`SensorSuite`) and `viz/record_replay.py` already use `mujoco.Renderer.enable_depth_rendering()` (via `M.rig.rl`) and `mujoco.Renderer.enable_segmentation_rendering()` (`self.seg` masking self-geoms cleanly), alongside `render_km` (`earth_bg=True`).

| Stream in `mjorbit` | `record_replay.py` Res | `SensorRigSpec` Mission Res | Cosmos Role |
|---|---|---|---|
| **`third` (3rd-person orbit)** | `384 × 384` | Free orbital tracking cam | Best candidate for photorealistic full-mission video (`720p` / `480p` 16:9) |
| **`bus_cam` (nav camera)** | `224 × 224` | `512 × 512` (`bus_cam_res`) | Monocular `8–35 m` `BusNet_orb` domain augmentation & glare stress-testing |
| **`wrist_cam` / `wrist_cam2`** | `192 × 192` | `128 × 128` (`wrist_res`) | High-risk sub-mm stereo pair (50 mm baseline) — use Blender PBR for truth |
| **`bus_lidar` (`M.rig.rl`)** | `128 × 128` (`log1p` RGB) | `128 × 128` depth/seg | Source geometry for normalized `depth` & `seg` control channels |

[[fig:pipeline|Bridge from mjorbit recorded trajectories: fixing variable frame timing and exporting aligned RGB/Depth/Seg/Edge streams for Cosmos-Transfer2.5-2B.]]

### 4.2 Two Essential Fixes Before Feeding `mjorbit` Video to Cosmos

If you drop `runs/replay_p7_earth_g3/frames/third/*.jpg` straight into Cosmos-Transfer2.5-2B today, it will fail to render clean physical motion for **two concrete reasons**:

1. **Variable Frame Intervals Break Temporal Priors (`SAMPLE_INTERVALS`):** `viz/record_replay.py` uses phase-dependent intervals (`far_app: 6.0 s`, `xfer_to_hold3: 6.0 s`, `hold3: 2.5 s`, `unfold: 0.5 s`, `descent: 0.4 s`) to keep replay downloads compact (`591` frames for `1550 s`). Played at `15–25 fps`, a `6 s/frame` segment accelerates time by `90×–150×`! Video diffusion expects smooth continuous inter-frame motion within each **93-frame chunk**. **Fix:** Add an offline **constant-FPS re-renderer** reading `manifest.json` (or re-running the seeded episode at uniform `dt = 1/16 s` or `1/10 s` around key windows: `xfer_to_hold3` fly-around, `unfold`, `descent -> close -> hold`).
2. **Resolution & Multi-Modal Stack Export:** Upgrade the offline renderer to export four temporally aligned `.mp4` streams at `16:9` (`854×480` for pilot/VRAM safety, or `1280×720` native):
   - **`vis`:** Standard `render_km` RGB (or slight Gaussian-blurred `sim/optics.py` pass so Cosmos re-synthesizes fine MLI crinkles instead of copying flat MuJoCo shading).
   - **`depth`:** Linearized disparity / inverted normalized metric depth (`far=black`, `near=white`, background sky/stars clamped cleanly so 1381 m Earth sphere depth does not wash out foreground dynamic range!). **Important catch:** normalize spacecraft depth over `[0.05 m, 40 m]` distinctly from the far Earth sphere (`>100 m`), otherwise client surface relief gets quantized to 1 grey level!
   - **`seg`:** Color-coded semantic map from `enable_segmentation_rendering()` mapping `client_bus` (gold MLI), `adapter_ring` (brushed Al), `solar_wings` (Si blue/black), `servicer + arm` (white MLI/metal), `earth_sphere` (planetary surface/limb), and `deep_space` to distinct flat RGB colors.
   - **`edge`:** Temporal-flicker-free Canny edges (`cv2.Canny` applied lightly or hybrid depth+normal discontinuities so specular noise doesn't strobe in Canny).

### 4.3 Concrete `spec.json` Recipe for a `mjorbit` Orbital Transfer Clip

When testing an orbital approach (`third` or `bus_cam` chunk), recommended initial weights keep geometry strictly pinned while letting lighting & texture breathe:

```json
{
  "name": "mjorbit_p7_descent_7720000",
  "prompt": "Photorealistic spaceflight camera footage of an orbital servicer approaching an "
            "unprepared tumbling satellite in 550 km LEO. Hard sunlight, black space, "
            "crinkled gold Kapton MLI, blue solar arrays, brushed aluminum 610 mm ring.",
  "guidance": 5.0,
  "num_steps": 35,
  "edge":  { "control_weight": 0.40, "control_path": "edge_720p.mp4" },
  "depth": { "control_weight": 0.30, "control_path": "depth_720p.mp4" },
  "seg":   { "control_weight": 0.15, "control_path": "seg_720p.mp4" },
  "vis":   { "control_weight": 0.15, "control_path": "vis_rgb_720p.mp4" }
}
```

> Notice the weights sum to **1.00** explicitly (`0.40 + 0.30 + 0.15 + 0.15`), preventing implicit rescaling. During close-range `wrist_cam` tests, raising `edge + depth` toward `0.80` (`vis` down to `0.10`) forces stronger geometric loyalty while letting surface metal sheen vary.

---

## Hardware, VRAM, Benchmarks & Cost Analysis

### 5.1 Official Hardware Numbers vs Our RTX 4060 Laptop

Official NVIDIA single-GPU benchmarks for generating **one 93-frame 720p clip** cleanly illustrate the compute regime `Cosmos-Transfer2.5-2B` was engineered for:

| Hardware Class | VRAM | Inference Support | 720p / 93-frame Gen Time | Total E2E Wall Time | Practical Verdict for `mjorbit` |
|---|---|---|---|---|---|
| **NVIDIA B200** | 192 GB | Native BF16 | **~92.25 s** | **~186 s** (~3.1 min) | Ideal for fast cloud batch generation |
| **NVIDIA H100 PCIe** | 80 GB | Native BF16 | **~264.13 s** | **~533 s** (~8.9 min) | Sweet spot for single-GPU LoRA + batch inference |
| **NVIDIA H100 NVL** | 94 GB | Native BF16 | **~445.52 s** | **~895 s** (~14.9 min) | Viable enterprise/cloud node |
| **NVIDIA H20** | 96 GB | Native BF16 | **~683.65 s** | **~1370 s** (~22.8 min) | Sluggish (~23 min per 6 s clip) |
| **RTX 4090 (24 GB)** | 24 GB | Tight (`480p` + heavy offload/FP8) | Slow | Heavy RAM swap | Marginal for 1–2 clips; painful for datasets |
| **RTX 4060 Laptop (Ours)** | **8 GB** | **Fails native (`65.4 GB` req)** | Severe PCIe thrash | **Risk of `87 °C` thermal shutdown** | **Do NOT run Cosmos locally**; use Cloud/NIM |

### 5.2 Why CPU Offloading on an 8 GB Laptop Is a Dead End for Datasets

Even if community `ComfyUI` or `diffusers` (`enable_sequential_cpu_offload()` + FP8 `float8_e4m3fn` + `480p` downscaling) manages to limp through a single short test without crashing on an 8 GB card, three practical blockers rule it out for `mjorbit`:
1. **Throughput Collapse:** Shuttling billions of DiT + Qwen2.5-VL + ControlNet parameters across PCIe for `30–50` denoising steps turns a 4-minute H100 run into tens of minutes or hours per **single 93-frame clip** — impossible for generating `15,000`-sample datasets (`stereo_data_orb.npz` scale).
2. **Thermal Guardrail Violation:** Our `CONTEXT.md` records three prior laptop thermal shutdowns at `86–87 °C` during continuous `70%` GPU training on `StereoNet`, forcing strict `--duty 0.5` throttling. Sustained memory-controller + tensor-core thrashing from video diffusion violates our hardware safety constraints.
3. **Zero Room for Fine-Tuning:** Even single-GPU parameter-efficient **LoRA** training (`bf16`, batch size `1`, gradient checkpointing) on Cosmos 2B architecture requires **an 80 GB A100/H100**.

### 5.3 Practical Cloud vs Local Budget Strategy

- **Step A (Pilot, $0 to ~$5):** Test **NVIDIA NIM / `build.nvidia.com`** hosted Cosmos-Transfer2.5 API (or spin up a spot **1× H100 80 GB** instance for ~1 hour, ~$2–4/hr) on `3–5` 93-frame clips exported from `runs/replay_p7_earth_g3`. Total pilot cost is negligible and proves visual + geometric quality *before* building infrastructure.
- **Step B (Local Heavy Lifting, $0 GPU rent):** Keep **all mass frame rendering and stereo ground-truth synthesis local** on the RTX 4060 using **Blender Cycles PBR with OptiX denoising** (at `192×192`–`512×512`, `15,000` still frames or short stereo bursts easily fit in `8 GB` VRAM and support frame-by-frame `--duty 0.5` sleep pauses).
- **Step C (Cloud LoRA + Batch Cosmos, Targeted Burst):** Only rent a cloud `1× 80 GB H100/A100` (or `8× H100` if scaling) once Pilot Step A proves value, to train an orbital LoRA and synthesize `~50–200` full orbital video trajectories (`93`-frame chunks stitched cleanly).

---

## Robotics-Specific Pitfalls: Geometry, Stereo & Space Priors

Generating impressive MP4s for humans is easy; generating mathematically truthful training data for closed-loop robotic GNC exposes **three deep technical hazards**.

### 6.1 Sub-Millimeter Label Drift (`StereoNet_orb` Hazard)

Let's look at the calibration precision we already measured in `projects/p7_unprepared_client/CONTEXT.md`:
- `stereo_net_in+out_orb.pt` achieves **`err_p50 = [0.61, 1.05, 2.17] mm`** (radial, tangential, axial) and **`[0.59, 0.66] deg`** tilt on orbital validation data!
- At close capture range (`~0.15 m` before jaw contact) on a `128×128` or `192×192` camera (`~40–45°` FOV), **`1 pixel` subtends `~0.6–0.9 mm`**.
- **The Hazard:** Latent VAEs compress space **8×** (`8×8` pixels per latent token), and diffusion decoders hallucinate crisp metallic chamfers, reflections, and high-frequency edge anti-aliasing. A tiny **`1.5–2.5 px` shift** of the `adapter_ring` inner lip between the MuJoCo `depth/edge` input and the Cosmos RGB output introduces **`1–2.3 mm` of systematic label poison** — **2× to 4× larger than `StereoNet_orb`'s entire current error!** Feeding poisoned Cosmos crops directly into supervised `StereoNet` training would *degrade* radial capture precision (`1.88 mm` seat -> abort/jaw strike).

### 6.2 Multi-View Stereo Epipolar Inconsistency (`wrist_cam` + `wrist_cam2`)

`StereoNet` relies on **sub-pixel binocular disparity** across a **50 mm stereo baseline** between `wrist_cam` and `wrist_cam2`.
- If you run single-view `Cosmos-Transfer2.5-2B` independently on `wrist_cam` video and `wrist_cam2` video, **diffusion stochasticity invents different specular glints, different Kapton crinkle highlights, and shifted rim edges in the left vs right eye**.
- Stereo correspondence breaks immediately: two views of the exact same physical millimeter feature no longer obey the camera calibration matrix's epipolar line!
- **Three Mitigations:**
  1. **Use `robot-multiview` ControlNet conditioning** (`Cosmos-Transfer2.5-2B/robot-multiview-control`), which shares cross-view spatiotemporal attention across cameras. *(Caveat: verify whether `robot-multiview` preserves 50 mm macro-stereo disparity at 15 cm range, since it was built mostly for wide-baseline robot + third-person / AV surround cameras!)*
  2. **Side-by-Side / Epipolar Strip Conditioning (Research Hack):** Stitch left + right synchronized control frames side-by-side (`640×720 | 640×720`) inside a single `1280×720` frame and fine-tune LoRA to enforce shared surface textures across both halves.
  3. **Cleanest Separation of Duties:** Use **Blender Cycles PBR** for sub-millimeter stereo close-up training data (guaranteed exact 50 mm ray-traced stereo disparity), and reserve **Cosmos-Transfer2.5** for monocular `bus_cam` (`8–35 m`), `third`-person mission videos, and visual robustness evaluation!

### 6.3 The Terrestrial Pre-Training Prior Gap

`Cosmos-Transfer2.5-2B` learned its visual intuition primarily from internet video and terrestrial robotics/driving datasets (warehouses, kitchens, highways, factories). Without careful conditioning or fine-tuning, zero-shot orbital generation exhibits telltale failure modes:
- **Atmospheric haze & soft bounce light:** Terrestrial models expect ambient sky scattering (`sky blue` shadows) rather than **zero-atmosphere vacuum**, where shaded spacecraft faces plunge nearly black (modulated only by Earthshine) while sunlit edges blow past camera sensor dynamic range.
- **Plausibility vs Orbital Physics:** Without strong `vis` + `depth` + `edge` locks (`control_weight` >= `0.7` combined) and optional `mask_path` protecting the client satellite, background stars can smear into bokeh streaks or Earth clouds can turn into blurry terrestrial terrain.

---

## Post-Training & Fine-Tuning Cosmos-Transfer2.5-2B

Your intuition — *render photorealistic ground-truth orbital videos first (or assemble real/HIL space footage) and fine-tune Cosmos so it learns the true physics of orbital optics* — aligns directly with NVIDIA's own post-training architecture in `nvidia-cosmos/cosmos-transfer2.5`.

### 7.1 Full FSDP Post-Training vs Single-GPU LoRA

The official repository (`docs/post-training.md`, `docs/post-training_singleview.md`, `docs/post-training_auto_multiview.md`) supports two tiers of adaptation:

| Tuning Method | Hardware Requirement | Framework / Format | What Updates | Best Use for `mjorbit` |
|---|---|---|---|---|
| **Full Parameter Post-Training** | **8× H100 / A100 (80 GB)** | `torchrun` + FSDP -> Distributed Checkpoint (`DCP`) -> `.pt` | All DiT / ControlNet layers | Large datasets (`10k+` clips); overkill for Phase 1 |
| **Single-GPU LoRA / DoRA** | **1× A100 / H100 (80 GB)** | `diffusers` + `peft` (`bf16`, bs 1, grad ckpt) -> LoRA `.safetensors` | Low-rank adapters (`~50–300 MB` file) | **Ideal for `mjorbit`** — learns MLI, Earth limb & space optics from `20–200` clips safely |

> **Why LoRA Fits Orbital Domain Adaptation Better Than Full Fine-Tuning:** With small specialized space datasets (`30–300` clips), full parameter tuning risks catastrophic forgetting of temporal motion smoothness. Freezing the 2.36B trunk and training a rank `16–64` LoRA injects sharp orbital lighting, sensor bloom, Earth albedo, and metallic BRDF physics while keeping training fast and preventing overfitting.

### 7.2 Official Dataset & Post-Training Command Pattern

Following `scripts/prepare_videoufo_dataset.py` and `docs/post-training.md` / `docs/post-training_singleview.md` in `nvidia-cosmos/cosmos-transfer2.5`, environment variables `HF_HOME` and `IMAGINAIRE_OUTPUT_ROOT` control weight caching and run outputs, and each training sample `(i)` pairs:
- **Target Photorealistic Video (`target_rgb/`):** `93`-frame MP4 (constant FPS, e.g. `16 fps`, `720p` or `480p`).
- **Paired Control Videos (`edge/`, `depth/`, `seg/`, `vis/`):** Exact frame-count and resolution matches. During training, feed the **crude/blurred MuJoCo (or degraded simulator) render as `vis`**, while setting **`target_rgb`** to **photorealistic Blender Cycles or real-space ground truth**. That explicitly teaches Cosmos the **sim-to-real residual mapping** from MuJoCo's look to physical orbital optics.
- **Rich Text Caption (`captions/`):** Auto-captioned via **Qwen2.5-VL** (or generated deterministically from `mjorbit` telemetry: range `m`, sun elevation deg, eclipse flag, visible Earth limb, target bus face).

Official multi-GPU FSDP launches invoke `scripts.train` with `torchrun` and write **Distributed Checkpoints (DCP)** (consolidated into `.pt` for downstream single-file inference), whereas single-GPU LoRA adapters serialize cleanly to `pytorch_lora_weights.safetensors`:

```bash
# Official full FSDP post-training entry point (8x 80 GB node)
EXP=mjorbit_orbital_transfer
torchrun --nproc_per_node=8 --master_port=12341 -m scripts.train \
  --config=cosmos_transfer2/_src/transfer2/configs/vid2vid_transfer/config.py -- \
  experiment=${EXP}
```

---

## The Hybrid "Blender PBR -> Cosmos Fine-Tune" Strategy

Let's examine your specific proposal: **generate realistic videos in Blender first, then fine-tune Cosmos-Transfer2.5-2B on those + real recorded space datasets, and finally use Cosmos to synthesize new trajectory videos fast**. Does adding Cosmos on top of Blender actually buy us anything, or should we just use Blender alone?

### 8.1 Step-by-Step Hybrid Architecture

1. **Trajectory & Asset Bridge (Local CPU / RTX 4060):** Read `scene3d.json` + dense constant-FPS `manifest.json` from any `mjorbit` run (e.g. different `ClientSpec` variants, tumble rates, sun angles).
2. **Stage-1 Teacher Renderer (Local RTX 4060 — Blender Cycles / BlenderProc):** Import the `89` meshes + procedural textures (`mli_gold`, `solar_cell`, `brushed_metal`, `radiator_osr`, `earth_day_5400.jpg`). Replace flat shaders with physically based BSDFs:
   - *Kapton MLI:* Anisotropic glossy amber/gold + normal/bump map driven by Voronoi/cloth crinkle geometry -> real specular glint streaks.
   - *`adapter_ring`:* Brushed stainless/aluminum anisotropic BSDF -> captures real golden bounce reflections from the bus onto the jaws!
   - *Atmosphere & Sun:* True Rayleigh-approximated limb shell + `0.53°` collimated solar disc (`sun_phase_deg`).
   - *Dual Output Export:* From the **exact same trajectory frame**, Cycles outputs the **photorealistic beauty pass (`target_rgb`)** plus razor-sharp **`depth`, `seg`, and `edge`**, while MuJoCo renders the **crude simulator pass (`vis_sim`)**.
3. **Stage-2 Student Translator (Cloud 1× 80 GB GPU — Cosmos LoRA):** Fine-tune `Cosmos-Transfer2.5-2B` LoRA mixing:
   - **Paired `mjorbit (vis_sim, depth, seg, edge) -> Blender Cycles (target_rgb)`** clips (teaches exact `mjorbit` spacecraft geometry & materials), AND
   - **Real orbital footage (`auto-labeled -> real_rgb`)** from NASA ISS / ADRAS-J / SHIRT (teaches authentic real-camera sensor bloom, auto-exposure clipping, lens flare, micro-motion blur, and CCD/CMOS grain that even Blender struggles to fake perfectly).
4. **Stage-3 Fast Trajectory Synthesis:** Now, for any new `mjorbit` replay (`G3` capture variations, multi-wing clients, failed collisions, edge-case lighting), you export cheap MuJoCo `depth/seg/edge/vis` in seconds and let the orbital-LoRA Cosmos synthesize **hundreds of realistic videos** with varied camera artifacts, cloud patterns, and harsh lighting.

### 8.2 Direct Decision Matrix: Blender Alone vs Zero-Shot Cosmos vs Hybrid

| Dimension | Blender Cycles PBR Directly | Cosmos-Transfer2.5 Zero-Shot | Hybrid: Blender + Real Space -> Cosmos LoRA |
|---|---|---|---|
| **Pixel & Epipolar Accuracy (`StereoNet`)** | **Exact (Sub-pixel / 0.0 mm drift)** | Poor (`1–3+ px` drift, stereo breaks) | Better than zero-shot, **still needs `< 1 px` audit** |
| **Material & Lighting Realism** | Very high (true physical multi-bounce rays) | Medium-high (risks Earth/terrestrial leakage) | **Highest overall** (combines PBR physics + real ISS/ADRAS-J camera grit) |
| **Camera Optics / Lens Flare / Bloom** | Synthetic unless hand-modeled in compositor | Organic natural video artifacts | **Authentic real-world space camera look** |
| **Runs on Our RTX 4060 8 GB?** | **Yes** (OptiX denoised, controllable duty) | **No** (needs NIM / 80 GB cloud) | Cycles local -> LoRA & batch Cosmos on cloud |
| **Speed per Trajectory Video** | Medium (`~0.3–2 s/frame` @ 720p on 4060 -> `30–180 s` / 93 frames) | `~264 s` gen / `~533 s` E2E on H100 | Fast once tuned (plus arbitrary prompt domain randomization!) |

> **Engineering Takeaway:** Notice how close the compute math is! On an RTX 4060 with OptiX denoising, Blender Cycles can actually render a 93-frame `480p`–`720p` PBR clip in **roughly the same wall-clock time** as zero-shot Cosmos on an H100 — *with mathematically exact geometry*. Where Cosmos truly pulls ahead in your hybrid idea is **injecting authentic real-world orbital video optics** (learned from NASA/ADRAS-J/TRON footage) and **fast visual domain diversity** (changing MLI age, thermal blankets, camera exposure, Earth clouds) without rebuilding 3D shaders for every new mission variant.

---

## Existing Real & Hardware-in-the-Loop Space Datasets

Fine-tuning a world model on synthetic Blender data alone only teaches it "CGI realism". To pull Cosmos-Transfer2.5-2B all the way into **real spaceflight optics**, mix in curated public real-orbit and Hardware-In-the-Loop (HIL) robotics datasets.

### 9.1 Curated Catalog of Usable Orbital Datasets

| Dataset / Source | Data Modality & Target | Lighting / Domain Fidelity | Temporal Sequence? | Best Role in Cosmos / `mjorbit` Pipeline |
|---|---|---|---|---|
| **SHIRT** (Stanford SLAB) | RGB sequences + 6-DoF poses of PRISMA **Tango** satellite across LEO trajectories (`ROE1`, `ROE2`) | OpenGL synthetic + **TRON real robotic HIL facility** | **Yes (Full Trajectories)** | **#1 Candidate for video LoRA** — teaches continuous rendezvous motion & HIL shadows |
| **SPEED & SPEED+** (Stanford SLAB / ESA) | 60k+ synthetic + **9,531 TRON HIL real images** with sunlamp & Earth albedo lightbox | **Real calibrated space lighting** (lens flare, harsh shadows, washed albedo) | No (Static frames) | Image/short-window camera-optics reference + evaluating domain gap |
| **SPARK / SPARK 2022–2024** (Univ. Luxembourg SnT) | RGB + **Depth + Segmentation** + bounding boxes + event streams (multi-class + **Proba-2**) | Procedural orbital env + Zero-G Lab HIL | **Yes (2022+ trajectory streams)** | Zero-effort multi-modal training (`depth` + `seg` already provided alongside RGB!) |
| **ADRAS-J Footage** (Astroscale / JAXA) | Real on-orbit fly-around imagery & time-lapse video of derelict **H-IIA upper stage** | **100% Real LEO Flight** (MLI, nozzles, harsh glare, uncooperative tumble) | Yes (Public video clips) | Gold-standard fine-tuning target for authentic uncooperative derelict textures & Sun glare |
| **NASA Image & Video Library / Internet Archive** | Public-domain HD video of **ISS dockings** (Dragon, Cygnus, Soyuz, Shuttle, grapple fixtures) | **100% Real Orbit** (real Earth limb, real radiators, MLI crinkle, real thrusters) | Yes (Long MP4s) | Cleanest public-domain source of 720p/1080p orbital clips (`images.nasa.gov`, non-copyrighted!) |
| **ESA VBN (`MAN-DATA-L1/L2`) / RemoveDEBRIS** | Real robotic rail / Airbus optical facility imagery (**Envisat mockup**) & orbit flight clips | Real optical lab hardware + flight video | Yes | Validation reference for dark-room metallic specular & close-range grapple views |
| **SEENIC & SPADES** (Zenodo) | Neuromorphic **event camera + RGB** proximity trajectories | Real lab proximity lighting | Yes | Future event-camera / HDR extreme-contrast research |
| **Space Robotics Bench** (`arXiv:2509.23328`) | Isaac Sim / Isaac Lab procedural orbital manipulation & docking (`Gymnasium`) | Path-traced procedural PBR | Yes | Direct reference for `ClientSpec` procedural diversity alongside Cosmos |

### 9.2 Practical Auto-Labeling Pipeline for Unlabeled NASA / ADRAS-J Videos

Real ISS docking clips and ADRAS-J fly-around videos do not come with MuJoCo `depth` or `seg` buffers — yet Cosmos post-training expects paired `(rgb, edge, depth, seg, caption)`. We can auto-annotate raw 93-frame space clips cleanly off-the-shelf:
1. **Cut & Standardize:** Extract clean `93`-frame shots (`16 fps` or `24 fps`, `1280×720`, removing NASA TV overlays / picture-in-picture HUD graphics!).
2. **Generate `edge`:** OpenCV Canny edge map (`cv2.Canny` + mild bilateral pre-filter to suppress star noise).
3. **Generate `depth`:** Zero-shot **Depth Anything V2** (normalize spacecraft disparity cleanly; clamp dark background pixels `mean < 5 DN` to zero disparity so deep space stays infinitely far).
4. **Generate `seg`:** Prompted **SAM 2** video segmentation separating `target_spacecraft`, `solar_array`, `robotic_arm / servicer`, `earth_limb`, and `deep_space`.
5. **Generate `vis`:** Gaussian blur + minor color-jitter of the real video (`k=11` or `15`), teaching Cosmos to reconstruct crisp micro-details rather than memorizing pixels.
6. **Generate Caption:** Pass representative frames + telemetry notes into **Qwen2.5-VL** (or `Cosmos-Reason1`) for consistent prompt vocabulary.

---

## Concrete 4-Stage Evaluation Roadmap & Verification Gates

Rather than committing blindly to heavy cloud GPU runs, we recommend a **strictly gated 4-stage roadmap** that validates geometry and perception accuracy at every step.

[[fig:roadmap|Gated 4-stage roadmap from zero-cost cloud pilot to BlenderCycles PBR, Orbital LoRA, and closed-loop perception audit.]]

### Stage Summary & Quantified Go / No-Go Gates

| Stage | Concrete Action | Where It Runs | Est. Budget / Time | Quantitative Go / No-Go Gate |
|---|---|---|---|---|
| **Stage 0: Zero-Cost / Low-Cost Pilot** | Update `record_replay.py` (`--const-fps 16 --export-controls` for `edge/depth/seg/vis` at `16:9`) on **3 clips** (`replay_p7_earth_g3`: `xfer`, `unfold`, `descent`); run zero-shot Cosmos 2.5 2B. | **Local export** (CPU) -> **NVIDIA NIM** or **1× H100** (~15 min) | **$0 – $5** / 1–2 hrs | Judge space/lighting quality AND measure **Canny silhouette IoU / edge shift** (`sim` vs `cosmos`) + run frozen `BusNet_orb` on generated `bus_cam`. |
| **Stage 1: Local Blender Cycles PBR Baseline** | Write `viz/export_blender.py` reading `scene3d.json` + dense poses into **Blender Cycles (OptiX)** for `ClientSpec` / `ServicerSpec`; render `500` stereo pairs + `3` trajectory clips. | **Local RTX 4060 (8 GB)** with `--duty 0.5` thermal guard | **$0** / ~2–4 hrs dev | **Gate 1A:** Zero-shot `StereoNet_orb` eval on Cycles PBR quantifies true sim-to-real `err_p50`. **Gate 1B:** Verify epipolar disparity `< 0.2 px`. |
| **Stage 2: Orbital LoRA Post-Training** | Curate `~30–100` aligned 93-frame clips combining **Blender Cycles (`target_rgb` paired with MuJoCo `vis_sim`)**, **SHIRT/SPARK** HIL sequences, and auto-labeled **NASA ISS / ADRAS-J** clips. Train `bf16` LoRA (`diffusers` + `peft`). | **Cloud 1× A100/H100 (80 GB)** | **~$15 – $40** (~4–8 GPU-hrs) | Validation loss converges smoothly; qualitative blind test shows true black deep space, hard solar terminator shadows, golden Kapton bounce, and **zero terrestrial artifacts**. |
| **Stage 3: Gated Perception Validation (`BusNet` & `StereoNet`)** | Generate `1,000` validation-matched clips using the orbital LoRA; test automated `< 1 px` edge-alignment filtering (`sim` mask vs synthesized boundary). | **Cloud 1× H100** gen -> **Local RTX 4060** `BusNet`/`StereoNet` test | **~$10 – $25** | **Only mix Cosmos outputs into training if:** (1) edge drift `< 1 px` on `bus_cam`, AND (2) mixing Cosmos + Blender improves `BusNet_orb` / `StereoNet_orb` accuracy on **unseen Blender PBR & TRON/SPEED+ test frames** without raising E2E collisions (`0 / 25` must hold). |

> **Recommended Immediate First Experiment (Stage 0):** Before touching Blender shaders or renting training clusters, add a small `--const-fps 16 --export-cosmos-controls` flag to `viz/record_replay.py` (leaving mission physics untouched), extract **one 93-frame `5.8 s` orbital fly-around clip** and **one 93-frame `descent` clip** from seed `7720000`, and run them through **NVIDIA NIM (`build.nvidia.com`)** or a single short cloud H100 call. Seeing our actual `mjorbit` spacecraft come out of vanilla `Cosmos-Transfer2.5-2B` will tell us in 10 minutes exactly how big the space-prior gap is and where LoRA conditioning needs to focus.

## Appendix: Sources & References

- **NVIDIA Cosmos-Transfer2.5 Official Repository (`nvidia-cosmos/cosmos-transfer2.5`):** Architecture, `controlnet_specs`, `docs/post-training.md`, `docs/post-training_singleview.md`, `docs/post-training_auto_multiview.md`, `scripts/prepare_videoufo_dataset.py` — `https://github.com/nvidia-cosmos/cosmos-transfer2.5`
- **Hugging Face Model Card (`nvidia/Cosmos-Transfer2.5-2B` & `robot-multiview-control`):** `2,358,047,744` base parameters, `~65.4 GB` single-GPU VRAM requirement, B200/H100/H20 benchmarks, 93-frame / 720p specs, **NVIDIA Open Model License** — `https://huggingface.co/nvidia/Cosmos-Transfer2.5-2B`
- **NVIDIA Cosmos Consolidated Hub:** `https://github.com/NVIDIA/Cosmos`
- **Space Robotics Bench (`arXiv:2509.23328`):** Orsula et al., *Space Robotics Bench: Robot Learning Beyond Earth* — `https://arxiv.org/abs/2509.23328`
- **SPEED / SPEED+ & SHIRT Datasets:** Stanford Space Rendezvous Laboratory (SLAB) & ESA — PRISMA Tango HIL TRON testbed & LEO ROE1/ROE2 rendezvous trajectory sequences (hosted via Stanford Digital Repository & Zenodo).
- **SPARK Dataset (`SnT / CVI²`, Univ. Luxembourg):** Multi-modal spacecraft RGB/depth/segmentation & Proba-2 trajectory streams.
- **SEENIC & SPADES Datasets (Zenodo):** Neuromorphic event-camera & RGB proximity operations datasets.
- **NASA Image & Video Library (Public Domain):** ISS visiting vehicle docking & proximity footage — `https://images.nasa.gov`
- **ADRAS-J (Astroscale / JAXA) & ESA VBN (`MAN-DATA-L1/L2`):** Real on-orbit H-IIA derelict fly-around imagery and Envisat robotic rail testbed footage.
- **`mjorbit` Project Internals Read Directly:** `projects/p7_unprepared_client/viz/record_replay.py`, `projects/p7_unprepared_client/gnc/sensors.py`, `projects/p7_unprepared_client/specs/servicer_spec.py`, `CONTEXT.md`, and `learning/RESEARCH_SERVICING.md`.
