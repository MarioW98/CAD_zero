#!/bin/sh
# Installa gli hook versionati (scripts/git-hooks/) in questo clone,
# come symlink: il contenuto resta quello del repo — un git pull
# aggiorna anche l'hook, che non puo' mai divergere dal giudice della CI.
#
# Uso (dalla radice del repo):
#   sh scripts/install_git_hooks.sh
set -eu

root=$(git rev-parse --show-toplevel)
cd "$root"

hooks_dir=$(git rev-parse --git-path hooks)
mkdir -p "$hooks_dir"

hook_src="$root/scripts/git-hooks/commit-msg"
if [ ! -f "$hook_src" ]; then
    echo "install: manca $hook_src" >&2
    exit 1
fi
chmod +x "$hook_src"

ln -sfn "$hook_src" "$hooks_dir/commit-msg"

echo "installato: $hooks_dir/commit-msg -> scripts/git-hooks/commit-msg"
echo 'prova:      printf "bozza" >/tmp/msg && scripts/git-hooks/commit-msg /tmp/msg'
echo "            (atteso: FAIL etichetta di progresso, exit 1)"
