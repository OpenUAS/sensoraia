#!/usr/bin/env bash
# Copyright 2026 OpenUAS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

MORA_SSH_TARGET=${1:-air@theatre.local}
MORA_INSTALL_DIR=${2:-${MORA_INSTALL_DIR:-/home/air/sensoraia}}
MORA_PACKAGE_DIR=$(realpath -m -- "${MORA_PACKAGE_DIR:-$SCRIPT_DIR/.build/mora-package}")

if [[ ! "$MORA_INSTALL_DIR" =~ ^/[[:alnum:]_./+-]+$ || "$MORA_INSTALL_DIR" == *..* ]]; then
  echo 'deploy_mora.sh: set MORA_INSTALL_DIR to a safe absolute target path' >&2
  exit 2
fi

for required_command in sha256sum rsync ssh; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    echo "deploy_mora.sh: required command not found: $required_command" >&2
    exit 1
  fi
done

if [[ ! -f "$MORA_PACKAGE_DIR/SHA256SUMS" ]]; then
  echo 'deploy_mora.sh: no release package; run make mora-package first (or make mora-deploy).' >&2
  exit 1
fi
exec 9>"$MORA_PACKAGE_DIR.lock"
flock 9
pushd "$MORA_PACKAGE_DIR" >/dev/null
sha256sum --check --status SHA256SUMS
popd >/dev/null
if [[ ! -r "$MORA_PACKAGE_DIR/usher.service" ]] ||
    ! grep -Fxq "WorkingDirectory=$MORA_INSTALL_DIR" "$MORA_PACKAGE_DIR/usher.service" ||
    ! grep -Fq "ExecStart=$MORA_INSTALL_DIR/usher " "$MORA_PACKAGE_DIR/usher.service"; then
  echo "deploy_mora.sh: package destination differs; rebuild with make mora-package MORA_INSTALL_DIR=$MORA_INSTALL_DIR" >&2
  exit 1
fi

echo "MORA USHER: staging $MORA_SSH_TARGET:$MORA_INSTALL_DIR"

resolve_host_ip() {
  local host=$1 ip
  if command -v getent >/dev/null 2>&1; then
    ip=$(getent ahostsv4 "$host" 2>/dev/null | awk 'NR == 1 { print $1; exit }')
    if [[ -n "$ip" ]]; then
      printf '%s\n' "$ip"
      return 0
    fi
    ip=$(getent ahosts "$host" 2>/dev/null | awk 'NR == 1 { print $1; exit }')
    if [[ -n "$ip" ]]; then
      printf '%s\n' "$ip"
      return 0
    fi
  fi
  return 1
}

is_onlink_ipv4() {
  local ip=$1 route_line
  [[ "$ip" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ && "$ip" != 127.* ]] || return 1
  command -v ip >/dev/null 2>&1 || return 1
  route_line=$(ip -4 -o route get "$ip" 2>/dev/null | head -n 1) || return 1
  [[ -n "$route_line" && "$route_line" != *" via "* && "$route_line" != local\ * ]]
}

wake_mdns_target() {
  local mdns_host=$1 short_host=${1%.local} candidate_ip woke=false
  local -a neighbors=()
  if [[ -n "$short_host" && "$short_host" != "$mdns_host" ]] && command -v getent >/dev/null 2>&1; then
    while IFS= read -r candidate_ip; do
      [[ -n "$candidate_ip" ]] || continue
      if is_onlink_ipv4 "$candidate_ip"; then
        if [[ -z "${onlink_dns_ip:-}" ]]; then
          onlink_dns_ip=$candidate_ip
        fi
        if command -v ping >/dev/null 2>&1; then
          ping -c 1 -W 1 "$candidate_ip" >/dev/null 2>&1 || true
          woke=true
        fi
      fi
    done < <(getent ahostsv4 "$short_host" 2>/dev/null | awk '!seen[$1]++ { print $1 }')
  fi
  if [[ "$woke" == false ]] && command -v ip >/dev/null 2>&1 && command -v ping >/dev/null 2>&1; then
    mapfile -t neighbors < <(ip -4 neigh show 2>/dev/null | awk '/lladdr/ && !seen[$1]++ { print $1 }')
    if (( ${#neighbors[@]} > 0 && ${#neighbors[@]} <= 16 )); then
      for candidate_ip in "${neighbors[@]}"; do
        { is_onlink_ipv4 "$candidate_ip" && ping -c 1 -W 1 "$candidate_ip" >/dev/null 2>&1 || true; } &
      done
      wait || true
    fi
  fi
}

ssh_ctl_dir=$(mktemp -d /tmp/mora-ssh.XXXXXX)
cleanup_ssh() {
  if [[ -S "$ssh_ctl_dir/S" && -x "$ssh_ctl_dir/ssh" ]]; then
    "$ssh_ctl_dir/ssh" -O exit -- "$MORA_SSH_TARGET" >/dev/null 2>&1 || true
  fi
  rm -rf -- "$ssh_ctl_dir"
}
trap cleanup_ssh EXIT

ssh_opts=()
if ssh_cfg=$(ssh -G -- "$MORA_SSH_TARGET" </dev/null 2>/dev/null); then
  ssh_host=$(awk '/^hostname / { print $2; exit }' <<<"$ssh_cfg")
  ssh_hostkey_alias=$(awk '/^hostkeyalias / { print $2; exit }' <<<"$ssh_cfg")
  ssh_proxy_cmd=$(awk '/^proxycommand / { sub(/^proxycommand /, ""); print; exit }' <<<"$ssh_cfg")
  ssh_proxy_jump=$(awk '/^proxyjump / { sub(/^proxyjump /, ""); print; exit }' <<<"$ssh_cfg")
  ssh_control_path=$(awk '/^controlpath / { sub(/^controlpath /, ""); print; exit }' <<<"$ssh_cfg")

  if [[ -z "$ssh_control_path" || "$ssh_control_path" == none ]]; then
    ssh_opts+=(-o ControlMaster=auto -o ControlPersist=60s -o "ControlPath=$ssh_ctl_dir/S")
  fi

  if [[ -n "$ssh_host" && ( -z "$ssh_proxy_cmd" || "$ssh_proxy_cmd" == none ) && ( -z "$ssh_proxy_jump" || "$ssh_proxy_jump" == none ) ]]; then
    resolved_ip=""
    onlink_dns_ip=""
    if ! resolved_ip=$(resolve_host_ip "$ssh_host") && [[ "$ssh_host" == *.local ]]; then
      for _ in 1 2 3; do
        wake_mdns_target "$ssh_host"
        if resolved_ip=$(resolve_host_ip "$ssh_host"); then
          break
        fi
      done
      if [[ -z "$resolved_ip" && -n "$onlink_dns_ip" ]]; then
        resolved_ip=$onlink_dns_ip
      fi
    fi
    if [[ -n "$resolved_ip" && "$resolved_ip" != "$ssh_host" ]]; then
      if [[ -z "$ssh_hostkey_alias" || "$ssh_hostkey_alias" == none ]]; then
        ssh_hostkey_alias=$ssh_host
      fi
      ssh_opts+=(-o "HostName=$resolved_ip" -o "HostKeyAlias=$ssh_hostkey_alias")
    fi
  fi
fi

{
  printf '#!/usr/bin/env bash\nexec ssh'
  for opt in "${ssh_opts[@]}"; do
    printf ' %q' "$opt"
  done
  printf ' "$@"\n'
} >"$ssh_ctl_dir/ssh"
chmod 0700 "$ssh_ctl_dir/ssh"

mora_ssh() {
  "$ssh_ctl_dir/ssh" -T -- "$MORA_SSH_TARGET" "$@"
}

stage=$(mora_ssh \
  "mkdir -p '$MORA_INSTALL_DIR' && mktemp -d '$MORA_INSTALL_DIR/.deploy.XXXXXXXX'")
stage_name=${stage##*/}
if [[ "$stage" != "$MORA_INSTALL_DIR/$stage_name" || ! "$stage_name" =~ ^\.deploy\.[a-zA-Z0-9]+$ ]]; then
  echo "deploy_mora.sh: invalid remote staging path" >&2
  exit 1
fi

rsync -e "$ssh_ctl_dir/ssh" --archive --human-readable --info=progress2 \
  "$MORA_PACKAGE_DIR/" "$MORA_SSH_TARGET:$stage/"

echo "MORA USHER: validating and activating staged release"
if ! mora_ssh "if [[ -f \$HOME/pass.txt && \$(tr -d '\r\n' < \$HOME/pass.txt) != replacewithyourpass ]]; then sudo -S -p '' -v < \$HOME/pass.txt 2>/dev/null || true; fi; bash '$stage/deploy_mora_remote.sh' '$MORA_INSTALL_DIR' '$stage'"; then
  echo "deploy_mora.sh: activation failed; inspect rollback output and $stage/backup" >&2
  mora_ssh \
    "if [[ -f \$HOME/pass.txt && \$(tr -d '\r\n' < \$HOME/pass.txt) != replacewithyourpass ]]; then sudo -S -p '' -v < \$HOME/pass.txt 2>/dev/null || true; fi; \
     sudo -n systemctl --no-pager --full status usher.service || true; \
     sudo -n journalctl --no-pager -u usher.service -n 50 || true"
  exit 1
fi

echo "MORA USHER: deployment complete"
echo "MORA USHER: service status: ssh $MORA_SSH_TARGET 'systemctl status usher.service'"
echo "MORA USHER: live logs: ssh $MORA_SSH_TARGET 'journalctl -fu usher.service'"