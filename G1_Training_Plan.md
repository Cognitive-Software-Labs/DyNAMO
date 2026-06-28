# Language-Controllable Pick-and-Place on the Unitree G1

**Experimental Plan — deliverables as repository artifacts**

`Model: pi0 / openpi` · `Platform: Isaac Sim (unitree_sim_isaaclab)` · `End-effector: Dex1 parallel gripper` · `Stage: simulation`

> Deliverables live in the project repository — configs, checkpoints, per-trial results, and media. Quantitative findings are not typed by hand: each evaluation run writes a results file, and a script aggregates these into a single generated `reports/report.md`. This document specifies what each phase commits to the repo and how results flow into that report.

---

## Key terms

Plain definitions for the terms used below, so nothing in the plan is ambiguous:

- **Trial / rollout.** One full attempt at the task by the policy in simulation, from start to finish. A *rollout video* is the screen recording of that single attempt; evaluation runs many trials and averages the outcome.
- **Checkpoint.** A saved snapshot of the trained model that can be reloaded to run inference or resume training.
- **Execution success.** The robot completed a clean pick-and-place, regardless of which object it moved.
- **Instruction-alignment.** The robot acted on the specific object the instruction named — the real test of whether it follows language rather than visual habit.
- **Evaluation conditions.** `matching` = the instruction matches the original task; `swapped` = same scene, different instruction; `novel` = a new combination of already-seen elements.
- **Object-confusion summary.** A small counts table: each row is the object the instruction asked for, each column is the object the policy actually grabbed. Off-diagonal counts expose systematic mix-ups — e.g., always grabbing the block when told "cylinder".
- **Manifest / data card.** A short file listing what a dataset contains — tasks, episode/frame counts, and the target object per episode.
- **LoRA.** A lightweight fine-tuning method that trains a small set of added weights instead of the whole model, keeping memory cost low.
- **Headline KPI.** The single number used to judge overall success — here, the instruction-alignment rate for getting the named object into the basket.

---

## 1. Objective

Develop and validate a language-conditioned manipulation policy that enables the Unitree G1 to perform pick-and-place actions from natural-language instructions. The end goal is that a single instruction — e.g., "put the apple into the basket" — drives the robot to place a specified object into a basket.

> **Headline KPI:** the instruction-alignment rate for placing the specified object into the basket within a multi-object scene (measured in Phase 5; reported in `reports/report.md`).

---

## 2. Research Questions

The study is organized around three progressively harder questions:

- **RQ1 — Multi-task competence:** After joint training on several pick-and-place tasks, can each task be executed correctly in isolation?
- **RQ2 — Language controllability:** With the scene held fixed and only the instruction changed, does the policy switch behavior accordingly — i.e., does it genuinely follow language rather than relying on visual cues alone?
- **RQ3 — Generalization and goal attainment:** Can altering the instruction elicit task compositions absent from training, and can the end goal (placing into a basket) be attained after targeted data collection?

| RQ | Question | Answered by (in the repo / report) |
|----|----------|-------------------------------------|
| RQ1 | Can each task be executed in isolation after joint training? | Per-task execution success % in `report.md` (Phase 3, matching condition), computed from `results/phase3_matching.csv`. |
| RQ2 | Does behaviour switch when only the instruction changes? | Instruction-alignment % for the swapped condition + the Decision-Gate summary, plus swapped-instruction rollouts under `media/phase3/`. |
| RQ3 | Do novel instruction combinations work, and is the basket goal reached? | Novel-combination metrics (Phase 3) and the Phase 5 headline-KPI section of `report.md`, with basket rollouts under `media/phase5/`. |

---

## 3. Methodology

- **Model.** pi0, fine-tuned with openpi via LoRA, to preserve the pretrained backbone's generalization while keeping memory cost bounded.
- **Platform.** Isaac Sim, using the official `unitree_sim_isaaclab` environment. Training and evaluation run in the same simulation domain so that visual domain shift is removed as a confound.
- **Embodiment.** Dex1 parallel gripper. Rationale: (i) official same-domain simulation datasets exist for Dex1 but not for the three-fingered hand; (ii) a simple gripper affords clean attribution — failures separate into "instruction not understood" versus "grasp not executed".
- **Specification.** 16-dimensional state/action (14 arm joints + 2 grippers) with a single head-mounted camera, held identical across data collection and evaluation.
- **Reproducibility.** The exact environment (Isaac Sim, isaaclab, openpi, CUDA, etc.) is pinned in the repo's dependency file and summarized in `README.md` — captured as project state, not as a form to fill in.

---

## 4. Available Data Assets

Three official Dex1 simulation datasets are same-domain and immediately usable; the basket task is implemented as an extension of the official `unitree_sim_isaaclab` environment and its USD asset library, avoiding any from-scratch setup.

| Dataset | Task | Approx. frames | Domain | Status |
|---------|------|----------------|--------|--------|
| Red-block pick-and-place | Pick the red block, place at target | ~160k | Dex1 sim, same-domain | Ready |
| Cylinder pick-and-place | Pick the cylinder, place at target | ~155k | Dex1 sim, same-domain | Ready |
| RGY-block stacking | Stack red / green / yellow blocks | — | Dex1 sim, same-domain | Ready |
| Basket task | Multi-object scene; place instruction-selected target into basket | Collect (Phase 4) | Extension of `unitree_sim_isaaclab` | Not started |

---

## 5. Experimental Plan

Six build phases (0–5) followed by the Decision Gate (Section 7). Each phase has a primary output, an acceptance condition, and a concrete set of artifacts it commits to the repository.

| Phase | Work | Primary output | Done-when (acceptance) |
|:-----:|------|----------------|------------------------|
| 0 | Bring up the simulation environment and the pi0 training/deployment pipeline | Reproducible training–evaluation scaffold | One full train→eval→inference loop runs end-to-end from a committed config. |
| 1 | Train on the official red-block dataset to a high single-task success rate | Single-task baseline | Execution success ≥ **[80%]** over ≥ **[20]** trials (pipeline-readiness threshold). |
| 2 | Consolidate the three tasks; standardize instructions and balance proportions | Multi-task dataset | Merged dataset loads; mix within **[±10%]** of target; instructions standardized in `data/instructions.yaml`. |
| 3 | Jointly train and evaluate language controllability and generalization | Quantitative diagnostic findings | All three eval conditions logged with ≥ **[20]** trials each; `report.md` regenerated with RQ1–RQ3 analysis. |
| 4 | Build a basket task in simulation; collect "multiple objects + instruction-selected target" data | Basket environment and dataset | Basket scene runs; dataset reaches **[target]** episodes; target object labelled per episode in the manifest. |
| 5 | Retrain with basket data; validate "instruction → place into basket" and iterate | Goal-attainment assessment | Headline KPI logged over ≥ **[20]** trials per condition; `report.md` shows the KPI and the iteration history. |

### Repository layout

All deliverables are files in one repo. Suggested structure (paths, not a form):

```
g1-langpick/
├─ README.md                setup + run commands + environment summary
├─ requirements.txt|env     pinned dependencies (replaces any "spec table")
├─ configs/                 train_redblock.yaml, train_multitask.yaml, train_basket.yaml
├─ data/
│   ├─ manifests/           dataset cards: tasks, episodes, frames, mix, target labels
│   └─ instructions.yaml    canonical instruction + paraphrases per task
├─ scripts/
│   ├─ train.py
│   ├─ eval.py              runs N trials, writes one row per trial to results/
│   └─ make_report.py       aggregates results/ → reports/report.md
├─ checkpoints/<phase>/     trained policy + the config that produced it
├─ results/<run>.csv        per-trial evaluation log (schema in §6)
├─ media/<phase>/           screen captures (.png) and rollout videos (.mp4)
└─ reports/report.md        generated; the human-readable findings
```

### What each phase commits

**Phase 0 — bring-up**
- `configs/train_redblock.yaml`; `scripts/train.py` and `scripts/eval.py` running end-to-end; README with the exact commands; pinned dependency file.
- `media/phase0/`: screen captures of the G1+Dex1 scene loaded, a training run in progress, and an inference rollout (one full task attempt played back from a saved checkpoint).

**Phase 1 — single-task baseline**
- `checkpoints/phase1/redblock.ckpt` + its config; `results/phase1_redblock.csv`.
- `media/phase1/`: `training-curve.png` plus [3–5] success and [1] failure rollout videos.

**Phase 2 — multi-task dataset**
- `data/manifests/multitask.json` (episodes / frames / mix per task); `data/instructions.yaml` (canonical + paraphrases).
- `media/phase2/`: sample frames per task with the instruction overlaid.

**Phase 3 — joint training & diagnostics**
- `checkpoints/phase3/multitask.ckpt`; `results/phase3_{matching,swapped,novel}.csv`.
- `media/phase3/`: swapped-instruction rollouts; `report.md` gains the RQ1–RQ3 analysis and an object-confusion summary (which object the policy grabbed vs. the one the instruction asked for), both computed from the CSVs.

**Phase 4 — basket env & data**
- Environment-extension code; `data/manifests/basket.json` (objects present + target object per episode).
- `media/phase4/`: the basket scene and sample frames with instruction overlay.

**Phase 5 — retrain & goal attainment**
- `checkpoints/phase5/basket.ckpt`; `results/phase5_*.csv`; the headline-KPI section of `report.md`.
- `media/phase5/`: correct-object-into-basket successes and a short failure montage.

---

## 6. Evaluation and Reporting

Two metrics are recorded separately, to prevent inflated success rates (succeeding by visual chance rather than by following the instruction):

- **Execution success rate** — whether a clean pick-and-place motion is completed, irrespective of which object.
- **Instruction-alignment rate** — whether the manipulated object is the one specified by the instruction; the true measure of language control.

Evaluation spans three conditions: (i) the original task with its matching instruction; (ii) the same scene with a swapped instruction; and (iii) an instruction formed from previously seen elements in a novel combination.

> **Acceptance.** The single-task (red-block) success rate is the pipeline-readiness threshold; the core KPI is the instruction-alignment rate for placing the specified object into the basket within a multi-object scene.

The flow: `eval.py` logs every trial; `make_report.py` reads `results/` and computes the rates per phase × condition, then writes `report.md`. Each results file follows one schema:

| Field | Type | Meaning |
|-------|------|---------|
| `run_id` | str | Identifies the checkpoint + eval batch (e.g. `phase3_swapped`). |
| `phase` | int | Which phase produced the run. |
| `condition` | enum | `matching` \| `swapped` \| `novel`. |
| `scene` | str | Objects present in the scene. |
| `instruction` | str | The instruction given to the policy. |
| `target_object` | str | Object the instruction specifies. |
| `manipulated_object` | str | Object the policy actually acted on. |
| `exec_success` | 0/1 | Clean pick-and-place completed. |
| `aligned` | 0/1 | `manipulated_object == target_object`. |
| `video` | path | Rollout under `media/<phase>/`. |

*`report.md` then renders, per phase and condition: execution success % = `mean(exec_success)`, instruction-alignment % = `mean(aligned)`, the object-confusion counts (instructed object vs. the object actually grabbed), the embedded key media, and the RQ1–RQ3 / headline-KPI narrative. No table is filled by hand.*

---

## 7. Decision Gate

On completion of Phase 3, a rapid ~10-trial instruction-swap test determines whether the policy genuinely follows language. The outcome sets the emphasis of Phase 4 data collection and prevents uninformed investment of effort.

Mechanically: run `eval.py` in the swapped condition for ~10 trials (logged to `results/`), then read the instruction-alignment rate over those trials. The decision is recorded as a short note at the top of `report.md` — not a separate hand-filled form.

> **Gate rule:** if instruction-alignment ≥ **[7/10]**, the policy follows language → Phase 4 emphasizes scaling object/instruction variety; if < **[7/10]**, Phase 4 emphasizes targeted data to strengthen language grounding before scaling.

---

## Appendix. Artifact map & conventions

Where each deliverable lives:

| Phase | Artifact | Repo path | Format |
|:-----:|----------|-----------|--------|
| 0 | Sim scene / training / rollout captures | `media/phase0/` | PNG, MP4 |
| 0 | Run config + commands + pinned env | `configs/`, `README`, env file | yaml, md |
| 1 | Baseline checkpoint + config | `checkpoints/phase1/` | ckpt, yaml |
| 1 | Per-trial results; training curve; rollouts | `results/`, `media/phase1/` | csv, png, mp4 |
| 2 | Dataset manifest; instruction set | `data/manifests/`, `data/instructions.yaml` | json, yaml |
| 2 | Sample frames with instruction overlay | `media/phase2/` | PNG |
| 3 | Multi-task checkpoint | `checkpoints/phase3/` | ckpt |
| 3 | Results for the 3 conditions; rollouts | `results/`, `media/phase3/` | csv, mp4 |
| 3 | RQ1–RQ3 analysis + confusion summary | `reports/report.md` | md |
| 4 | Basket env code; dataset manifest | (env ext), `data/manifests/basket.json` | code, json |
| 4 | Basket scene + sample frames | `media/phase4/` | PNG |
| 5 | Final checkpoint; results | `checkpoints/phase5/`, `results/` | ckpt, csv |
| 5 | Headline-KPI section; rollouts | `reports/report.md`, `media/phase5/` | md, mp4 |

**Conventions:**

- Captures named `phaseN_description.png`; rollouts named `phaseN_condition_trialNN.mp4`.
- One results CSV per eval run; `report.md` is always regenerated, never edited by hand for numbers.
- Environment is pinned (dependency file + lockfile) and summarized in `README` — there is no separate spec table to maintain.
