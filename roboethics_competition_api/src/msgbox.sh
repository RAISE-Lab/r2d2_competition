#!/bin/bash
xfce4-terminal --hold --geometry 100x10+0+0 --hide-borders --hide-toolbar --hide-menubar --hide-scrollbar -e "python ${1}/src/msgbox.py" --font=10
