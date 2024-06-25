
source ~/.profile

if [ -d "$PAPARAZZI_HOME/pprzEnv" ]; then
    source $PAPARAZZI_HOME/pprzEnv/bin/activate
fi

shell=$(echo $SHELL | cut -d / -f 3)

if [ "$shell" = "bash" ]; then
    # enter bash specific commands here
    echo $shell > /dev/null
elif [ "$shell" = "zsh" ]; then
    # enter zsh specific commands here
    echo $shell > /dev/null
else
    # fallback commands
    echo $shell > /dev/null
fi;
