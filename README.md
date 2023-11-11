# PATS

## [Installation]
Start [install.sh](base/install/install.sh) to install all requirements.

## [Firmwares](config/firmwares/README.md)

## [Simulator](./doc/simulator.md)

## [Hardware](./doc/hardware.md)

## valgrind
`valgrind --tool=memcheck --leak-check=full --track-origins=yes --track-fds=yes --suppressions=../../config/gst.supp --suppressions=/usr/local/share/opencv4/valgrind.supp --suppressions=/usr/local/share/opencv4/valgrind_3rdparty.supp --suppressions=/usr/share/glib-2.0/valgrind/glib.supp --gen-suppressions=all ./pats --generator 2>&1 | tee log.txt`
