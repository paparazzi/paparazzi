#!/bin/bash

# If user exists, change user and group ids, otherwise add new local user
# Either use the LOCAL_USER_ID if passed in at runtime or
# fallback

USER_NAME=${USERNAME:-pprz}
USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}

if id -u $USER_NAME > /dev/null 2>&1; then
    echo "Starting with UID: $USER_ID and GID: $GROUP_ID"
    # modify existing pprz user instead of creating new one
    #usermod -o --uid $USER_ID $USER_NAME
    groupmod -o --gid $GROUP_ID $USER_NAME
    # usermod goes through all files in home dir and changes the userid, which takes a while
    # since we don't want/need that here, directly change the user id in /etc/passwd
    sed -i "s/$USER_NAME:x:\([0-9]*\):\([0-9]*\):/$USER_NAME:x:$USER_ID:\2:/g" /etc/passwd
else
    echo "Adding new user $USER_NAME with UID: $USER_ID and GID: $GROUP_ID"
    useradd --shell /bin/bash -u $USER_ID -o -c "" -m $USER_NAME
fi

export HOME=/home/$USER_NAME

exec /usr/sbin/gosu $USER_NAME "$@"
