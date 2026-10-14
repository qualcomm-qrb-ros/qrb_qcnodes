# QRB QCNodes README — Design

**Date:** 2026-10-14
**Repo:** `qrb_qcnodes` (https://github.com/qualcomm-qrb-ros/qrb_qcnodes)
**Status:** Approved and implemented

## Problem

The repository was created with a single-line README (`# qrb_qcnodes`) and no
code. It needs a README that tells a visitor what the repo is for, what nodes
it will contain, and what hardware it targets — while being honest that no
code has landed yet.

## Reference

`qualcomm-qrb-ros/qrb_ros_adapter` README, adapted. That README's skeleton:

centered title + tagline + badges → `---` → Overview (repo link, architecture
image, component bullet list) → Table of contents → APIs (component table) →
Supported targets → Installation / Usage / Build from source / Test →
Contributing → License (BSD-3-Clause-Clear). Most body sections honestly read
"not ready yet."

A second house style exists: the in-house QCNode SDK README
(`QC-Graph-Framework-Release_1_0/QcNode/README.md`), which summarizes nodes in
a capability table (`QCNode | QNX PVM | HGY PVM | Library | Processor |
Features`).

## Decisions

| Decision | Choice | Rationale |
| --- | --- | --- |
| Node roster | Generic "sensor QCNodes" framing, IMU and Lidar as the first concrete examples | Repo is empty; a fixed roster would be speculative. Open-ended framing lets nodes be appended without rewriting the premise. |
| Unready sections | Keep the headings, body reads "not ready yet" | Matches the reference. Gives later PRs a fixed place to fill in, and the TOC stays stable. |
| Framing | Graph Framework, not ROS 2 | These are pure GF/QCNode components. No ROS badge, no ROS distribution column. |
| Target hardware | Qualcomm Dragonwing IQ-10 EVK | Per user. Note: no IQ-10 references exist elsewhere in the workspace (only IQ-9075 / IQ-8275), so the exact marketing string is unverified. |
| Layout | One directory per node (`imu/`, `lidar/`) plus shared `common/` | Mirrors the SDK's `source/Node/<Name>/` convention; gives the Overview a concrete component list. |
| Structure | Reference skeleton + SDK-style per-node capability table | The repo's purpose is "a collection of sensor QCNodes," so the first question a visitor has is which nodes exist and what they do. A directory table alone answers only "how is the code arranged." |

## Structure

1. **Header** — centered `<h1>QRB QCNodes</h1>`, tagline "Sensor QCNodes for
   Qualcomm Robotics Platforms", two badges: Qualcomm Ubuntu (links to the
   Qualcomm IoT Ubuntu download page, as in the reference) and Graph Framework
   (unlinked — no canonical public URL available).
2. **Overview** — repo link, the `QCNodeIfs` framing sentence, component bullet
   list (`common`, `imu`, `lidar`), a note that more nodes will be added, and
   the "not ready yet" caveat.
3. **Table of contents** — 8 entries, matching the 8 `##` headings below it.
4. **APIs** — the `QCNodeIfs` / `QCNodeConfigIfs` / `QCNodeMonitoringIfs`
   lifecycle paragraph, then two tables: component/responsibility (directories)
   and the per-node capability table (`QCNode | Sensor interface | Processor |
   Features`) with IMU and Lidar rows.
5. **Supported targets** — `Development hardware | Operating system | Graph
   Framework`, one IQ-10 row.
6. **Installation / Usage / Build from source / Test** — "not ready yet."
7. **Contributing** — "not ready yet for this initial release."
8. **License** — BSD-3-Clause-Clear.

## Deliberate omissions

- **Architecture image.** The reference embeds `./assets/qrb_ros_adapter_arch.png`.
  No equivalent diagram exists for this repo and one cannot be generated here, so
  the `assets/` reference is omitted rather than committed as a broken image link.
  Add it when a diagram exists.
- **ROS Jazzy badge and ROS distribution column.** Dropped per the Graph
  Framework framing decision.

## Supporting files

Ported from the qrb-ros house templates in the same change:

| File | Treatment |
| --- | --- |
| `LICENSE` | Copied verbatim. BSD-3-Clause-Clear, Qualcomm copyright. |
| `CODE-OF-CONDUCT.md` | Copied verbatim. Contributor Covenant 2.1 with the Qualcomm `github.coc@qti.qualcomm.com` enforcement address — nothing repo-specific. |
| `SECURITY.md` | Copied, with the issues URL corrected (see defects below). |
| `CONTRIBUTING.md` | Copied, with the commit-template placeholder filled and style guidance de-ROS-ified (see defects below). |
| `CHANGELOG.md` | Rewritten rather than copied (see defects below). |

With real files in place, the README's Contributing and License sections no
longer read "not ready yet" — they link to `CONTRIBUTING.md`,
`CODE-OF-CONDUCT.md`, `SECURITY.md` and `LICENSE`.

### Defects corrected from the reference templates

Three problems in `qrb_ros_adapter`'s own templates were fixed rather than
propagated:

1. **`SECURITY.md` pointed at a personal fork.** The reference links to
   `github.com/qti-weijshen/qrb_ros_adapter/issues/new` — an individual's fork,
   not the org repo. Corrected to
   `github.com/qualcomm-qrb-ros/qrb_qcnodes/issues/new`.
2. **`CONTRIBUTING.md` shipped an unfilled placeholder.** Line 68 of the
   reference reads literally `<add commit template>`. Replaced with worked
   `git commit -s -m` examples using this repo's scopes. The ROS 2 code-style
   link was also dropped (these are Graph Framework components, not ROS
   packages) in favor of surrounding-code-style guidance, the SPDX header
   requirement, and the `QCNodeIfs` expectation. Example commit scopes were
   changed from `gulp, broccoli, npm` to `cmake, sdk`.
3. **`CHANGELOG.md` was an unedited upstream template.** The reference ships
   fictional `1.0.0` / `0.1.0` releases dated 2023 with placeholder entries
   ("Core functionality for [Project Name]", "Deprecated old API endpoint
   `/v1/users`"). Committing that to a repo with one real commit would be
   false. Rewritten as a single `[Unreleased]` section describing the actual
   state.

### Not included

- **`.github/` directory.** The reference carries issue templates, a PR
  template, and five workflows (commit-lint, cpp-code-style-checker,
  action-lint, qcom-preflight-checks, stale-issues) that call shared
  `qualcomm-qrb-ros/qrb_ros_gh_actions` reusable workflows. Note that
  `CONTRIBUTING.md` references `issues/new?template=bug_report.yaml` and
  `feature_request.yaml`, so those two links will 404 until the issue
  templates are added. Worth a follow-up change.

## Verification

- All 8 TOC anchors match their corresponding `##` headings.
- All three tables have consistent pipe counts across header, separator, and
  body rows.
- Both shields.io badge URLs return HTTP 200; the `logo=qualcomm` slug is valid.
- All 8 `CONTRIBUTING.md` internal anchors resolve to defined `<a name=...>`
  targets.
- Every cross-file link (`CONTRIBUTING.md`, `CODE-OF-CONDUCT.md`,
  `SECURITY.md`, `LICENSE`) points to a file that exists.
- No placeholder or stale-reference strings remain across the committed
  markdown (`add commit template`, `INSERT LINK`, `qti-weijshen`,
  `qrb_ros_adapter`, `Project Name`, `TODO`, `TBD`).

## Follow-ups

- Confirm the exact IQ-10 marketing string (house style elsewhere is
  "Qualcomm Dragonwing™ IQ-9075 EVK").
- Supply a Graph Framework badge target URL, or drop that badge.
- Add `.github/` issue templates — `CONTRIBUTING.md` links to
  `bug_report.yaml` and `feature_request.yaml`, which 404 until they exist.
- Add the `.github/workflows` CI jobs that call the shared
  `qualcomm-qrb-ros/qrb_ros_gh_actions` reusable workflows.
- Fill the `Sensor interface` column once the IMU and Lidar nodes land, and
  revisit the `Processor` column if any node offloads beyond CPU.
- Add the architecture diagram to `assets/` and embed it in Overview.
