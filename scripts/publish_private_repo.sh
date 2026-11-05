#!/usr/bin/env bash
# Run after committing. Never force-push or upload to a public repository.
set -eo pipefail
cd "$(dirname -- "${BASH_SOURCE[0]}")/.."
repo_name="${1:-Nanju03/ur5e_team_ws}"
[[ "$repo_name" == Nanju03/ur5e_team_ws ]] || { printf 'This helper is scoped to Nanju03/ur5e_team_ws.\n' >&2; exit 2; }
command -v gh >/dev/null || { printf 'Install GitHub CLI first: sudo apt install gh\n' >&2; exit 1; }
[[ -z "$(git status --porcelain)" ]] || { printf 'Commit the reviewed changes first.\n' >&2; exit 1; }
login="$(gh api user --jq .login)"
[[ "$login" == Nanju03 ]] || { printf 'Authenticate as Nanju03: gh auth login\n' >&2; exit 1; }
if ! gh repo view "$repo_name" --json nameWithOwner >/dev/null 2>&1; then
  gh repo create "$repo_name" --private --description 'UR5e AG95 ROS 2 Humble team workspace with Gazebo diagonal testing and path archives'
fi
visibility="$(gh repo view "$repo_name" --json isPrivate --jq .isPrivate)"
[[ "$visibility" == true ]] || { printf 'Refusing to upload: repository must be private.\n' >&2; exit 1; }
requested_url="https://github.com/$repo_name.git"
current_url="$(git remote get-url origin 2>/dev/null || true)"
if [[ "$current_url" == https://github.com/gamsaon/ur5e.git ]]; then
  if git remote get-url upstream >/dev/null 2>&1; then
    printf 'An upstream remote already exists; inspect remotes before continuing.\n' >&2
    exit 1
  fi
  git remote rename origin upstream
  git remote add origin "$requested_url"
elif [[ -z "$current_url" ]]; then
  git remote add origin "$requested_url"
elif [[ "$current_url" != "$requested_url" ]]; then
  printf 'Origin points to a different repository; inspect remotes before continuing.\n' >&2
  exit 1
fi
# Keep the credential helper local to this repository; no token in argv.
gh_path="$(command -v gh)"
git config --local credential.https://github.com.helper "!\"$gh_path\" auth git-credential"
git push --set-upstream origin HEAD:refs/heads/main
local_sha="$(git rev-parse HEAD)"
remote_sha="$(gh api "repos/$repo_name/commits/main" --jq .sha)"
[[ "$local_sha" == "$remote_sha" ]]
printf 'Private repository uploaded and verified: https://github.com/%s/commit/%s\n' "$repo_name" "$local_sha"
