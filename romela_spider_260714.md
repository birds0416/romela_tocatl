# RoMeLa TOCATL Spider — Session Log (2026-07-14)

Session type: **status review / orientation** (no code written, no exams
answered). Goal was to read the prior session log and the Notion Spiderbot
page and determine the next step in the project.

Sources read this session:
- `~/Desktop/romela_spider_260709.md` (the 07-09 build session log)
- Notion **Spiderbot** page (`RoMeLa / Hojung's Notion / RoMeLa / Spiderbot`)
- Its two child **Daily Study Notes** pages (Topics 1–3)

---

## Where the project actually stands

The project has two threads, in very different states.

### Thread 1 — The simulator: essentially done
Built 2026-07-09, walks firmly, renders the real CAD body, all 7 acceptance
tests pass. Committed as **`f3686d9`** on branch **`codex/tocatl-mujoco-sim`**
(still **not pushed**). See the 07-09 log for the full build story. Remaining
items are polish/optional:
- Push branch + open draft PR (software-safety CI passes locally) — needs a
  go-ahead. **This is the only loose end on the software side.**
- Articulating 4-bar knee bar (currently fixed to the femur at nominal angle)
- RoMeLa logo on the dome
- Exact-CAD shell petals if Body Shell STLs get exported from SolidWorks

### Thread 2 — The learning curriculum: BLOCKED (this is the real next step)
The Notion page runs a **Robotics Professor Mode** protocol: each topic is
taught with a 10-part structure, ending in a **5-question oral exam Hojung must
answer live** before its curriculum-tracker box is checked. The teaching has
run three topics ahead of demonstrated understanding — every session so far was
an *unattended scheduled run* teaching to an empty room:

| Topic | Taught | Oral exam | Tracker box |
|---|---|---|---|
| 1. Coordinate frames & FK | 07-11 **and** 07-12 (twice) | never answered | ☐ unchecked |
| 2. Inverse kinematics (2-link + law of cosines) | 07-13 | never answered | ☐ unchecked |
| 3. Body frame & body→leg transforms | 07-14 | never answered | ☐ unchecked |

The 07-14 study note states the blocker plainly: *"THREE oral exams are now
pending, and the whole curriculum is blocked on them… The single thing that
will move this project forward is you sitting down interactively and answering
the oral exams (Topics 1, 2, 3)."* Nine further topics (gaits, foot
trajectories, CyberGear MIT law, contacts, ROS 2 architecture, …) stay unlocked
behind them.

---

## The next step (conclusion of this session)

**Sit the pending oral exams — start with Topic 1 (FK).** It has been taught
twice, so it is the ripe one. Recommended order:
1. Answer Topic 1 (FK) oral exam live → critique → check the box.
2. Then Topic 2 (IK), then Topic 3 (body→leg transforms).
3. Once all three boxes are checked, resume teaching at **Topic 4: Gait
   generation & the phase engine**.

Separately, on the software side: **push `codex/tocatl-mujoco-sim` + open the
draft PR** whenever Hojung gives the go-ahead — awaiting authorization, not
blocked on anything technical.

---

## Notion update made this session
Added a short **"Learning curriculum status"** callout to the top of the
Spiderbot page surfacing the blocker (Topics 1–3 taught, all three oral exams
pending), because that critical fact was previously only visible buried in the
child study-notes pages, not on the main page. No tracker boxes were checked
(correctly still unchecked), and the build-status section was left unchanged —
no project state changed this session.
