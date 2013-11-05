#!/usr/bin/env bash

git config --global color.ui auto     # colors for all
git config --global alias.st status   # make `git st` work
git config --global alias.co checkout # make `git co` work
git config --global alias.ci commit   # make `git ci` work
git config --global alias.br branch   # make `git br` work
git config --global alias.up "pull --rebase"   # make `git up` work similar to svn up
git config --global alias.lg "log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit --date=relative"
