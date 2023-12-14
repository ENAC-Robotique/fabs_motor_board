set pagination off
target extended-remote /dev/bmp_gdb
monitor swdp_scan
attach 1
set mem inaccessible-by-default off
load
detach
quit

