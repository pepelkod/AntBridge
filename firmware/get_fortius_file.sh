#!/bin/sh
find / -name "data2.cab" -exec unshield -g HexComponent -d firmware \;

#unshield -g HexComponent -d firmware x ~/mnt/data2.cab
