# Simple GitHub Workflow

`romela-tocatl` is the only local folder used for ongoing development.

## What the pieces mean

- `main`: the approved, current version of the project.
- A branch: a temporary workspace for one task.
- A pull request (PR): the review page comparing that branch with `main`.
- Merge: accept the PR and add its changes to `main`.
- Tag: a permanent name attached to a historical commit.

## Normal workflow

1. Start from an updated `main`.
2. Create one short-lived branch named `codex/<short-task>`.
3. Make and test one coherent change.
4. Push it and open one draft PR targeting `main`.
5. Confirm the `software-safety` GitHub Actions workflow passes.
6. Matthew reviews the summary and changed files. For safety-sensitive changes,
   review the test plan and confirm that no hardware action is hidden in it.
7. When Matthew says **merge**, mark the PR ready and merge it.
8. Delete the merged branch and update the local `main`.

Only one active development PR should exist unless two tasks truly need to
proceed independently. Avoid stacking a new PR on another unmerged branch.

## What Matthew needs to say

- **Review this PR:** explain the change without merging.
- **Change X:** update the same branch and PR.
- **Merge it:** merge the reviewed PR and clean up its branch.
- **Do not merge:** leave it open or close it without changing `main`.

## Recovery exception

PR 1 and PR 2 were stacked to preserve the exact Pi recovery before normal
development started. After they are accepted: tag PR 1's recovered commit,
merge PR 1, retarget PR 2 to `main`, then merge PR 2. Delete the two temporary
branches afterward. All future PRs should target `main` directly.

## Repository and machine roles

- Private GitHub repository: source-code backup and review history.
- Mac `~/dev/romela-tocatl`: primary working copy.
- Raspberry Pi: reviewed deployment and supervised hardware testing only.
- Google Drive: shared plans, photos, CAD, wiring diagrams, and test records.

Never commit passwords, tokens, Wi-Fi profiles, virtual environments, or normal
ROS `build/`, `install/`, and `log/` output.

## Automated checks

The `software-safety` workflow runs on PRs and pushes. It executes unit tests and
the no-live-CAN dry-run gate with the Pi service check skipped, because GitHub
runners are not the robot computer. The full service check still belongs on the
actual Pi before live telemetry or motion testing.
