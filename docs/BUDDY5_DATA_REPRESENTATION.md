# Buddy 5 data representation

Proposed ranges for team review. Explicit masks/shifts define the binary format;
C bit-field struct layout is not used as a wire format.

| Application field | Valid range | Bits |
|---|---|---:|
| Distance, including invalid placeholder | 20–4000 mm valid; 0 invalid | 12 |
| Distance validity | false/true | 1 |
| Approximate observed width | 0–4095 mm, or unavailable | 12 + 1 validity |
| State | 0–7 | 3 |
| Request | 0–4 | 3 |
| Module fault mask | seven independent conditions | 7 |
| Scan position | 0–24; angle = 30 + 5 × position | 5 |
| Stored sample count | 0–25 | 5 |
| Controller mode | normal/bypass/recovery/reset | 2 |
| Stopped, motion valid, line valid, line found | false/true each | 1 each |
| Cumulative detour path | 0–500 mm; saturates at stop limit | 9 |
| Retry count | 0–3 | 2 |
| Waiting for measurement | false/true | 1 |
| Fine scan endpoints | 0–24 each | 5 each |
| ECHO width | 0–30000 µs | 15 |
| Capture phase / result | four / three codes | 2 each |
| Local millisecond timestamps | full 32-bit monotonic epoch, wraps at 2^32 ms | 32 each |
| Raw hardware timestamps | full 32-bit TIMER counter, wraps at 2^32 µs | 32 each |

The retained 32-bit millisecond epoch allows unambiguous comparison to the local
clock for the module's bounded intervals, including long uptime. A 16-bit timestamp
proposal in the team's document is **not implemented here**: adopting it needs an
agreed retention/stall bound and handling of old data across wraps. Do not label
this choice as an accepted team decision or claim whole-firmware compliance.
Raw counter values and wider temporary arithmetic prevent overflow/rounding loss;
review these against the team's recorded position on transient intermediates.
Kernel handles, mutexes, task stacks and pointers retain the platform ABI types.

## Formats

All offsets are LSB-first bit offsets within a byte array; no pointer casts to
packed structs. Receivers know the exact packet length through their interface.

**Status: 19 bytes / 152 meaningful bits.** State 0:3, request 3:3, faults 6:7;
front 13:12 + valid25; closest26:12 + valid38; width39:12 + valid51;
left52:12 + valid64; right65:12 + valid77; slot78:5; count83:5;
publication time88:32; forward-reading consumption time120:32.

**Feedback: 6 bytes / 47 meaningful bits + 1 zero padding bit.** Mode0:2,
stopped2, motion-valid3, line-valid4, line-found5; cumulative path6:9;
measurement timestamp15:32. The final byte's bit7 must be zero. Encoded paths above
500 are rejected by the decoder; the public publishing API saturates before packing.

**Scan: 41 bytes / 325 meaningful bits + 3 zero padding bits.** Twenty-five
consecutive 13-bit samples, each consisting of 12 distance bits followed by
validity. Invalid/unmeasured samples are all zero. The slot is implicit. Use
`b5_scan_get()` for owner-side decoding; consumers can decode the same layout from
the snapshot. Publication and scan snapshots share the mailbox mutex, but separate
read calls are not an atomic combined observation. Individual ray timestamps are
not retained: the stationary scan has a proposed 15 s bound.

Static widths are proposed, not proof of maximum RAM savings. The Arm 13.2 target
linker reports `core` as 112 B; the host compiler reports 128 B due to ABI differences.
The protected scan snapshot costs 41 B; feedback/status mailboxes cost 6/19 B.
This does not include capture state, feedback working view, kernel objects, stack,
allocator overhead or the rest of the firmware.

MQTT publishing is not present in the uploaded demo. Buddy 1 must agree the topic,
format version, framing and fault mapping before using this binary module payload.
No RAM or message reduction against an equivalent unpacked baseline is claimed;
that comparison remains to be implemented and measured with the team.
