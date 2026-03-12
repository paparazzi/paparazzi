#!/bin/bash

# Match container user UID/GID with host user to avoid permission issues
USER_NAME=${USERNAME:-pprz}
USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}

if id -u "$USER_NAME" > /dev/null 2>&1; then
    echo "Starting with UID: $USER_ID and GID: $GROUP_ID"
    groupmod -o --gid "$GROUP_ID" "$USER_NAME"
    sed -i "s/$USER_NAME:x:\([0-9]*\):\([0-9]*\):/$USER_NAME:x:$USER_ID:\2:/g" /etc/passwd
else
    echo "Adding new user $USER_NAME with UID: $USER_ID and GID: $GROUP_ID"
    useradd --shell /bin/bash -u "$USER_ID" -o -c "" -m "$USER_NAME"
fi

export HOME=/home/$USER_NAME

# Start speech-dispatcher as the target user so PprzGCS --speech doesn't block.
# Must run as pprz (not root) to avoid creating ~/.config/ with wrong ownership.
gosu "$USER_NAME" speech-dispatcher --spawn 2>/dev/null || true

exec gosu "$USER_NAME" "$@"
