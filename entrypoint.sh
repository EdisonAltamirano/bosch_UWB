#!/usr/bin/env bash
set -euo pipefail

: "${LOCAL_USER_ID:=1000}"
: "${LOCAL_GROUP_ID:=1000}"
: "${LOCAL_GROUP_NAME:=hostgroup}"

# Create group if needed (prefer the provided name, fall back if it conflicts)
if ! getent group "${LOCAL_GROUP_ID}" >/dev/null 2>&1; then
  if ! getent group "${LOCAL_GROUP_NAME}" >/dev/null 2>&1; then
    groupadd -g "${LOCAL_GROUP_ID}" "${LOCAL_GROUP_NAME}" >/dev/null 2>&1 || true
  fi
  if ! getent group "${LOCAL_GROUP_ID}" >/dev/null 2>&1; then
    groupadd -g "${LOCAL_GROUP_ID}" hostgroup >/dev/null 2>&1
  fi
fi

# Create user if needed
if ! getent passwd "${LOCAL_USER_ID}" >/dev/null 2>&1; then
  useradd -m -u "${LOCAL_USER_ID}" -g "${LOCAL_GROUP_ID}" -s /bin/bash hostuser >/dev/null 2>&1
fi

USER_NAME="$(getent passwd "${LOCAL_USER_ID}" | cut -d: -f1)"
HOME_DIR="$(getent passwd "${LOCAL_USER_ID}" | cut -d: -f6)"

# Ensure workspace paths exist (bind mount lives at /home/ws/src)
mkdir -p /home/ws /home/ws/src /home/ws/.cache/matplotlib

# Chown the workspace root and any container-side subdirs (build, install, log…).
# Exclude /home/ws/src — that is a bind mount owned by the host user; chowning it
# from inside the container would change ownership on the host and break host access.
chown "${LOCAL_USER_ID}:${LOCAL_GROUP_ID}" /home/ws >/dev/null 2>&1 || true
find /home/ws -maxdepth 1 -mindepth 1 ! -name src -print0 2>/dev/null \
  | xargs -0 -r chown -R "${LOCAL_USER_ID}:${LOCAL_GROUP_ID}" >/dev/null 2>&1 || true

export HOME="${HOME_DIR}"
export MPLCONFIGDIR=/home/ws/.cache/matplotlib
cd /home/ws

exec gosu "${LOCAL_USER_ID}:${LOCAL_GROUP_ID}" "$@"

