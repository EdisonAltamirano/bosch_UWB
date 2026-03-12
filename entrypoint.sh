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
mkdir -p /home/ws /home/ws/src

# Try to make the workspace writable for the mapped user. This is best-effort:
# - bind mounts can only be chowned if the container has permission to do so.
chown -R "${LOCAL_USER_ID}:${LOCAL_GROUP_ID}" /home/ws >/dev/null 2>&1 || true

export HOME="${HOME_DIR}"
cd /home/ws

exec gosu "${LOCAL_USER_ID}:${LOCAL_GROUP_ID}" "$@"

