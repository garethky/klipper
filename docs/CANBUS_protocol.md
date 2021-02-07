This document describes the protocol Klipper uses to communicate over
[CAN bus](https://en.wikipedia.org/wiki/CAN_bus).

# Micro-controller id assignment

Klipper uses only CAN 2.0A standard size CAN bus packets, which are
limited to 8 data bytes and an 11-bit CAN bus identifier. In order to
support efficient communication, each micro-controller is assigned at
run-time a unique CAN bus identifier (`canbus_assigned_id`) for
general Klipper command and response traffic. Klipper command messages
going from host to micro-controller use the `canbus_assigned_id`,
while Klipper response messages from micro-controller to host use
`canbus_assigned_id + 1`.

Each micro-controller has a factory assigned unique chip identifier
that is used during id assignment. This identifier can exceed the
length of one CAN packet, so a hash function is used to generate a
unique 6-byte id (`canbus_uuid`) from the factory id.

During the id assignment process, an encoded id is used
(`canbus_encoded_id`). This is a one byte value (0-255). One can
translate from a `canbus_encoded_id` to a `canbus_assigned_id` using
the formula `canbus_assigned_id = canbus_encoded_id*2 + 256`.

# Admin messages

Admin messages are used for id assignment. Admin messages sent from
host to micro-controller use the CAN bus id `0x3f0` and messages sent
from micro-controller to host use the CAN bus id `0x3f1`. All
micro-controllers listen to messages on id `0x3f0`; that id can be
thought of as a "broadcast address".

## CMD_QUERY_UNASSIGNED message

This command queries all micro-controllers that have not yet been
assigned a `canbus_assigned_id`. Unassigned micro-controllers will
respond with a RESP_NEED_CANID response message.

The CMD_QUERY_UNASSIGNED message format is:
`<1-byte message_id = 0x00>`

## CMD_QUERY message

This command queries the micro-controller with a given `canbus_uuid`.
The given micro-controller will respond with either a RESP_NEED_CANID
or RESP_HAVE_CANID response message.

The CMD_QUERY message format is:
`<1-byte message_id = 0x01><6-byte canbus_uuid>`

## CMD_SET_ID message

This command assigns a `canbus_assigned_id` to the micro-controller
with a given `canbus_uuid`. The micro-controller will respond with a
RESP_HAVE_CANID response message.

The CMD_SET_ID message format is:
`<1-byte message_id = 0x02><6-byte canbus_uuid><1-byte canbus_encoded_id>`

## RESP_NEED_CANID message

The RESP_NEED_CANID message format is:
`<1-byte message_id = 0x20><6-byte canbus_uuid><1-byte static value = 0x00>`

## RESP_HAVE_CANID message

The RESP_HAVE_CANID message format is:
`<1-byte message_id = 0x21><6-byte canbus_uuid><1-byte canbus_encoded_id>`

# Data Packets

A micro-controller that has been assigned an id via the CMD_SET_ID
command can send and receive data packets.

The packet data in CAN bus messages using the micro-controller's
`canbus_assigned_id` are simply appended to a buffer, and when a
complete [mcu protocol message](Protocol.md) is found its contents are
parsed and processed. The data is treated as a byte stream - there is
no requirement for the start of a Klipper message block to align with
the start of a CAN bus packet.

Similarly, mcu protocol message responses are sent from
micro-controller to host by copying the message data into one or more
packets with a CAN bus id of `canbus_assigned_id + 1`.
