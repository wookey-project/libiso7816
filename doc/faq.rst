FAQ
---

Is the ISO7816 library self-contained?
""""""""""""""""""""""""""""""""""""""
The automatons of the ISO7816-3 are self-contained in the
library, but the library relies on a hardware abstraction
layer to handle the ISO7816 pins (I/O, Vcc, RST, clock).
