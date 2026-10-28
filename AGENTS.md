# AGENTS.md

Agent-facing guide for this repository. It complements `ARCHITECTURE.md`; it does not replace it.

## Project Overview

This repository defines a ROS 2 runtime for a single UR5e cell with a Robotiq 2F-85 gripper, a Robotiq wrist camera, a fixed base camera, and an externally supplied π0-family checkpoint running on an NVIDIA Quadro RTX 6000.

**In scope:** robot runtime and control, policy inference, checkpoint intake and validation, checkpoint evaluation, run recording and replay, safety and supervision, MuJoCo and URSim simulation, tests, CI, and documentation.

**Out of scope:** model training, fine-tuning, LoRA, dataset creation, demonstration collection, mobile manipulation, multi-robot coordination, force-controlled assembly, reinforcement learning inside the runtime, and cloud inference as a production dependency.

A checkpoint is **consumed**, never produced. If a supplied checkpoint is incompatible, reject it and report the issue. Do not retrain, fine-tune, or recompute training statistics locally.

At the time of writing, the tree contains only `ARCHITECTURE.md`, `AGENTS.md`, `README.md`, `.gitignore`, and the OpenCode development container. There is no application code. Verify the actual tree before acting; do not assume that components described in the architecture already exist.

## Source of Truth

`ARCHITECTURE.md` is the authoritative design document. Read the sections relevant to your task before changing anything.

- If this file and `ARCHITECTURE.md` disagree, `ARCHITECTURE.md` wins.
- An explicit user instruction overrides both; if it conflicts with the architecture, state the trade-off instead of silently diverging.
- If a task changes a boundary, contract, safety mechanism, or acceptance criterion, update `ARCHITECTURE.md` and add or amend an ADR in the same change.
- Record unresolved design questions instead of guessing.

## Commands

There is no project build, test, or lint command yet. Do not invent commands, document unverified commands, or claim that a command works.

When the first runnable scaffold is added, record the exact verified commands here:

- dependency installation,
- build,
- unit and integration tests,
- lint and type checks,
- simulation launch,
- guarded hardware launch.

Until then, inspect the tree with `find . -maxdepth 3 -type f -not -path './.git/*'` and read the relevant architecture sections.

Target environment:

| Component | Target |
|---|---|
| Robot runtime | Ubuntu 24.04, ROS 2 Jazzy, C++17, Python 3.12 |
| Policy server | Ubuntu 22.04, Python 3.11, CUDA 12 |
| Simulation | MuJoCo, URSim |
| Containerization | Docker Compose |

The dev container is built from `ros:jazzy-ros-base` and provides ROS 2 Jazzy, `colcon`, and `rosdep` after it is rebuilt. It has no Docker daemon and no GPU. Verify what is actually installed before assuming that a command or runtime is available.

## Repository Layout

The architecture defines the target layout. Create it incrementally; do not add directories or packages that the current requirement does not need.

```text
.devcontainer/         ROS 2 Jazzy development container for OpenCode
docs/
  adr/                 Architecture Decision Records
  architecture/        C4 views, contracts, safety case
  plans/               milestone plans
  runbooks/            start-up, recovery, rollback procedures
src/
  vla_domain/          pure domain: value objects, events, policies
  vla_application/     use cases and ports
  vla_adapters/        ROS 2, openpi, Modbus, MCAP
  vla_safety/          safety filters, supervisor, permits
  vla_msgs/            ROS 2 interfaces
  vla_sim/             MuJoCo model, simulator node, motion demo
  vla_bringup/         launch files and composition root
  vla_recorder/        run recording and flight recorder
  vla_eval/            scenario runner and checkpoint evaluation
services/policy_server/  Python 3.11 openpi inference service
sim/                   MuJoCo and URSim assets
config/                robots, cells, models
tests/                 unit, contract, integration, system, safety, performance
docker/                Dockerfiles and Compose profiles
scripts/               developer and CI automation
```

For a subproject that needs its own rules, add a nested `AGENTS.md`. The closest file in the directory tree takes precedence.

## Architecture Rules

These are mandatory. Do not weaken them for convenience.

1. **Dependency rule.** Dependencies point inward: infrastructure to adapters to application to domain. The domain imports no ROS 2, torch, JAX, openpi, filesystem, or network code.
2. **Policy isolation.** The policy runtime never publishes to actuators and never imports ROS 2. It receives observations and returns action chunks.
3. **Safety independence.** Safety is a deterministic supervisor, not a model. The policy proposes; the supervisor disposes. The E-stop and UR safety functions remain independent of ROS 2 and the model.
4. **Checkpoint contract.** No checkpoint loads without a valid manifest, SHA-256, normalization statistics, action/state spaces, image-slot mapping, horizon, control frequency, and license record (ADR-017).
5. **Temporal hierarchy.** Policy inference runs at 10-15 Hz; the servo loop runs at 125 Hz. Never block the servo loop on inference, file I/O, logging, or a service call.
6. **Action contract.** The model produces six joint deltas plus absolute gripper; the `AbsoluteActions` transform returns absolute joint targets to the runtime. Do not add a second delta conversion.
7. **Hardware-free bootstrap.** The runtime must remain runnable on a CPU-only workstation until a real checkpoint is evaluated. Use `MockPolicy` or `ReplayPolicy` for policy-free operation; a GPU is required only for real inference.
8. **Bootstrap bridge exception.** The direct MuJoCo bridge is an explicit bootstrap exception (ADR-016). `ros2_control` is reconsidered when controller integration is introduced, not added speculatively.
9. **No secrets or artifacts in Git.** No credentials, model weights, checkpoint binaries, MCAP run archives, or generated datasets.
10. **Versioned contracts.** A schema major mismatch is rejected. Unknown fields are ignored only at the boundary and never reach the domain.

## Quality and Scope

The objective is a coherent, correct, and maintainable system, not the fastest possible progress through a checklist. Milestones and gates in `ARCHITECTURE.md` are agreed checkpoints; they are not permission to rush, skip verification, or build ahead of need.

- Build the smallest coherent change that satisfies the current requirement.
- Do not implement work that no current requirement needs.
- Do not add abstractions, configuration knobs, plugin systems, or generic frameworks for hypothetical future needs.
- Do not add a dependency when the standard library or the existing stack is sufficient.
- Do not build a UI, dashboard, database, or orchestration layer without a concrete requirement.
- Prefer a direct, boring implementation over a clever one.
- Correctness, safety, clarity, and testability matter more than feature count.
- A small, complete, well-tested change is better than a large partial one.
- When changing one component, consider boundaries, failure modes, observability, operations, and documentation across the whole system.
- Do not treat any milestone as the only objective. A change that closes a gate but degrades the design is not complete.
- If prerequisites are missing, say so and address the gap rather than producing a shallow imitation.
- Measure performance and reliability; do not assume them.
- If a request implies scope creep, say so and ask before implementing.

## Code Style

| Area | Standard |
|---|---|
| Python formatting and linting | `ruff format`, `ruff check` |
| Python typing | `mypy` or `pyright` in strict mode for domain and application |
| Python architecture | `import-linter` |
| C++ formatting | `clang-format` |
| C++ analysis | `clang-tidy`, warnings as errors |
| ROS 2 tests | `colcon test`, `launch_testing` |
| Commits | Conventional Commits, focused changes |

Additional rules:

- Use hexagonal architecture, explicit ports, and constructor injection.
- Keep the composition root in `vla_bringup`; no global singletons or service locators.
- Use value objects and make illegal states unrepresentable.
- Parse boundary data once into domain types; do not re-validate raw dictionaries downstream.
- Prefer immutable observations, action chunks, manifests, and run records.
- Make commands idempotent with `request_id`.
- Use explicit state machines instead of boolean flags.
- Apply patterns such as Strategy, Adapter, Factory, Decorator, Chain of Responsibility, and Circuit Breaker only where they solve a current problem.
- Do not create a single "manager" node that owns drivers, safety, inference, and recording.
- Use ROS 2 conventions for node, topic, and parameter names.
- Avoid comments that restate the code; explain why, not what.
- Do not leave dead code, commented-out blocks, or unowned TODOs.
- Do not reformat unrelated files in the same change.
- Write code, comments, commit messages, and documentation in English.

## Testing

- Write or adjust tests before or alongside the implementation.
- Every bug fix gets a regression test.
- Every port gets a reusable contract suite.
- Prefer deterministic tests with fake clocks and seeded scenarios.
- Do not retry assertion failures. Retries are only for infrastructure flakiness.
- Keep safety-relevant branches covered explicitly.
- Domain branch coverage target: at least 90%. Safety decision logic: 100% branch coverage.
- Run the relevant test level before claiming completion:
  - L1 unit and L2 contract tests for domain and adapter changes.
  - L3 integration tests for ROS 2 nodes, lifecycle, QoS, and TF.
  - L4 system tests for closed-loop behaviour in MuJoCo or URSim.
- Do not claim completion based on a subset of tests. The relevant acceptance criteria in `ARCHITECTURE.md` and the task request must both be satisfied.

## Safety and Security

- Never bypass the safety supervisor, permit check, watchdog, or E-stop.
- Never give the policy server actuator access.
- Never command the real robot without an operator present and the agreed guarded procedure.
- Start in simulation. Real hardware requires explicit approval and the applicable safety checks.
- Treat a checkpoint as untrusted until SHA-256 and manifest validation pass.
- Treat the Robotiq camera HTTP endpoint as unauthenticated and network-isolated.
- Do not put secrets, credentials, personal data, or model weights in Git, logs, or MCAP.
- Assume the software may be faulty or compromised; safety must not depend on it.

## Workflow

Before starting:

1. Read the relevant `ARCHITECTURE.md` sections and this file.
2. Confirm the task's scope, acceptance criteria, and constraints.
3. Check Section 9.4 of `ARCHITECTURE.md` for unresolved decisions that affect the task.
4. Identify the smallest coherent change that satisfies the requirement.

Before finishing:

1. Tests pass at the appropriate level.
2. Lint, type checks, and formatting pass.
3. Contracts, schemas, and ADRs are updated if a boundary changed.
4. Observability exists for new failure modes.
5. No secrets, generated artifacts, or model weights were added.
6. Documentation describes the behaviour that actually exists.
7. Report what changed, what evidence proves it, and what remains open.

Git rules:

- Do not perform Git operations unless the user explicitly asks.
- Do not commit, push, rewrite history, or create a remote without confirmation.
- When asked to commit, use Conventional Commits and keep the change focused.

Pull requests:

- Title format: `<type>(<scope>): <summary>`.
- Describe what changed, why, which tests were run, and which risks remain.
- Include the rollback path for runtime, configuration, or checkpoint changes.
- Keep refactors separate from behaviour changes unless the refactor is required to make the behaviour correct.

## Research

Do not rely on memory for APIs, versions, or hardware behaviour.

- Prefer primary sources: official ROS 2 Jazzy documentation, Universal Robots documentation, Robotiq manuals, MuJoCo documentation, openpi documentation and source, NVIDIA CUDA documentation.
- Verify the API against the installed or pinned version. Check the actual source or a working example when documentation is ambiguous.
- Read upstream examples before inventing a new integration.
- Pin versions and cite the source URL in the PR, commit, or design note when a decision depends on external behaviour.
- Prefer official specifications over blog posts. If a blog or forum is the only source, label it as unverified.
- Do not copy code with an incompatible license. Check the license before adapting an upstream snippet.
- If external documentation conflicts with `ARCHITECTURE.md`, stop and reconcile it explicitly rather than changing the design silently.

Useful starting points:

- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/>
- MuJoCo: <https://mujoco.readthedocs.io/>
- openpi: <https://github.com/Physical-Intelligence/openpi>
- openpi UR5 example: <https://github.com/Physical-Intelligence/openpi/tree/main/examples/ur5>
- Universal Robots: <https://www.universal-robots.com/manuals/>
- Robotiq support: <https://robotiq.com/support>

## When in Doubt

- Prefer the simpler design.
- Prefer the documented contract over an implicit convention.
- Prefer a measured result over an assumption.
- Prefer stopping to ask over silently expanding scope.
- If a requirement is unclear, state the ambiguity and the options.
- If a task conflicts with the architecture, propose an ADR rather than making an undocumented exception.
- Keep this file current when commands, layout, or conventions change.
