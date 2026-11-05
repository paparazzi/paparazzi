#!/bin/bash

if ! type "astyle" > /dev/null; then
    echo "You need to install astyle to use this script!"
    exit 1
fi

if [ $# -eq 0 ]; then
    echo "Please provide one or multiple files as arguments!"
    exit 1
fi

ASTYLE_VERSION=$(astyle --version 2>&1 | awk '{print $4}' | head -1)
echo "Using astyle version $ASTYLE_VERSION"

set -f

# Safe version parsing with defaults
major=${ASTYLE_VERSION%%.*}
minor=$(echo "$ASTYLE_VERSION" | cut -d. -f2)

# Fix unary operator - quote variables and provide defaults
major=${major:-0}
minor=${minor:-0}

if [ "$major" -ge 3 ]; then
    echo "Using AStyle 3.x options"
    astyle --style=kr   \
        --indent=spaces=2  \
        --convert-tabs \
        --indent-switches    \
        --indent-preproc-block \
        --pad-oper      \
        --pad-header    \
        --unpad-paren   \
        --keep-one-line-blocks  \
        --keep-one-line-statements  \
        --align-pointer=name  \
        --suffix=none   \
        --lineend=linux   \
        --ignore-exclude-errors-x \
        --max-code-length=120 \
        "$@"
elif [ "$major" -ge 2 ]; then
    echo "Using AStyle 2.x options"
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
        "$@"
else
    echo "Using AStyle 1.x options"
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
        "$@"
fi
