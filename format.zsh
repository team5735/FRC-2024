#!/bin/zsh
# Requires clang-format to be installed
# Brew is recommended
clang-format -i src/**/*.java

if [ $? != 0 ]
then
    echo "The clang-format command failed! Perhaps you don't have it installed? Brew is suggested, if you can set it up... Talk to me if you really want brew."
    fi
