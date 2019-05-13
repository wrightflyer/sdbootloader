Start by reading:

http://spaces.atmel.com/gf/project/sdbootloader/

(inluding the project summary, wiki, forum, etc) as there's little point
this README just duplicating what's there.

This project is principally a project to be built using Atmel Studio 6. As such
it comes with .atsln and .cproj files. Load the pfboot.atsln and you should be
good to go.

If you are building in an environment where AS6 is not usable it should still be
possible to build using command line make as the .\Debug and .\avrapp\Debug dierctories
have the Makefile that AS6 auto-generated. True it is "odd" to build in such a sub-
directory and not the top level dir - but ask Atmel about that one not me.

Two programs can be built. The bootloader itself and a small AVR application to go
into the flash section to prove it works. There's nothing "special" about that apart
from a post-build event it includes to run srec_cat (from WinAVR) in order to pad the
binary with 0xFF and embed a CRC16 at the very end.

The project was designed for mega16 but should work on most AVRs that have 2K of BLS or
more. However it has hard-coded 0x3800's and 0x37FE's in it which are specific to the
position of the 2K BLS on the mega16 and these would need to be changed.

The entire software is under BSD licence. I chose this as it's principally an open
licence that allows you to use the code pretty much any way you choose (avoid GPL - 
you know it makes sense!).

Cliff Lawson
Oct 2012