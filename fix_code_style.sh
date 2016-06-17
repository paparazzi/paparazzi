#!/bin/bash

if ! type "astyle" > /dev/null; then
    echo "You need to install astyle to use this script!"
    exit 1
fi

if [ $# -eq 0 ]; then
    echo "Please provide one or multiple files as arguments!"
    exit 1
fi

ASTYLE_VERSION=`astyle --version 2>&1| awk '{print $4}'`
echo "Using astyle version $ASTYLE_VERSION"

set -f

if [ $(bc <<< "$ASTYLE_VERSION >= 2.03") -eq 1 ]; then
    astyle --style=kr   \
        --indent=spaces=2  \
        --convert-tabs \
        --indent-switches    \
        --indent-preprocessor \
        --pad-oper      \
        --pad-header    \
        --unpad-paren   \
        --keep-one-line-blocks  \
        --keep-one-line-statements  \
        --align-pointer=name  \
        --suffix=none   \
        --lineend=linux   \
        --add-brackets \
        --ignore-exclude-errors-x \
        --max-code-length=120 \
        $*
else
    astyle --style=kr   \
        --indent=spaces=2  \
        --convert-tabs \
        --indent-switches    \
        --indent-preprocessor \
        --pad-oper      \
        --pad-header    \
        --unpad-paren   \
        --keep-one-line-blocks  \
        --keep-one-line-statements  \
        --align-pointer=name  \
        --suffix=none   \
        --lineend=linux   \
        --add-brackets \
        $*
fi
