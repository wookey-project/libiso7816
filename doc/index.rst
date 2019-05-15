.. _lib_iso7816:

.. highlight:: c

The ISO7816 library
===================

.. contents::

This library is an implementation of the ISO7816-3 standard to
communicate with contact smart cards.

It should be hardware independent and makes use of a hardware
abstraction layer that must provide low level interactions with
the Vcc and RST (Reset) lines, the I/O line (pushing and
getting a byte), the clock and baudrate handling, etc.

This library implements the T=0 and T=1 protocols, ATR (Answer To
Reset) parsing and the PSS/PTS negotiation. It offers high level
functions to send APDUs to and receive Responses from a smart card.

.. include:: api.rst
.. include:: faq.rst
