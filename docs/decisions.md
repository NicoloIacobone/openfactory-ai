# Decisions

One entry per decision: context, decision, evidence, status. Newest last. Revisit an entry when its evidence changes.

## D-001 — Keep ROS 2 Jazzy on the existing VM

- **Context:** the project brief suggests Jazzy on Ubuntu 24.04 but allows any working distro.
- **Decision:** use the installed Ubuntu 24.04 + ROS 2 Jazzy; no reinstall, upgrade or migration.
- **Evidence:** audit 2026-10-28 (`docs/environment.md`): `ros2 doctor` passes, pub/sub works, Gazebo Harmonic, MoveIt 2 and UR packages are available as aarch64 binaries.
- **Status:** accepted.

## D-002 — English for repository docs

- **Context:** portfolio project for job applications.
- **Decision:** README, docs, code and commits in English; the assistant talks to the user in Italian.
- **Status:** accepted.

## D-003 — Repo reaches the VM through a second UTM shared folder

- **Context:** the VM already shares only `doppelbot`; sharing all of `~/Desktop/nico` would expose unrelated personal files.
- **Decision:** a dedicated share for `openfactory-ai` mounted at `/mnt/openfactory-ai`; colcon workspace outside it in `~/openfactory_ws`.
- **Status:** proposed — feasibility in UTM 5.0.8 to be verified; fallback rsync over SSH.

## D-004 — Robot arm

- **Context:** the brief suggests a mature arm such as UR5e or Franka Panda.
- **Decision:** open until phase 1. UR5e is the strongest candidate (complete Jazzy apt stack: description, Gazebo simulation, MoveIt config); Franka has no Jazzy apt package here.
- **Status:** open.
