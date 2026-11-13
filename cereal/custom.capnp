using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct CustomReserved0 @0x81c2f05a394cf4af {
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

# NAP camera / vision add-ons share this reserved slot. Same type id as
# the old empty CustomReserved6 (and the briefly-used StoplightNAP name).
# Event @113 is unchanged. Stoplight is a group so ordinals @0-@13 stay
# on the wire and existing stoplight logs stay readable. Future add-ons
# (sign reader, cone/obstacle, etc.) append new groups or fields with
# new numbers starting at @14. Do not reuse or renumber existing fields.
# Do not migrate liveSpeedSignNAP, pathObstacleNAP, or coneLineNAP here;
# those stay on their own reserved slots.
struct NapVision @0xf98d843bfd7004a3 {
  # Stoplight-green chime (selfdrive/stoplightd). Camera only, never
  # brakes, steers, or changes engagement. Published at ~4 Hz while
  # stopped in Drive, 1 Hz otherwise. soundd plays the ding on
  # stoplight.chime; the HUD shows the red flag and a short green flash.
  stoplight :group {
    enabled @0 :Bool;          # NAPStoplightChime
    gated @1 :Bool;            # stopped in Drive; the camera is only read while true
    redLight @2 :Bool;         # a confirmed red signal lamp is lit ahead
    green @3 :Bool;            # red -> green seen at this stop (cleared when the car moves)
    chime @4 :Bool;            # true on the one message where the ding fires
    chimeCount @5 :UInt32;     # +1 per ding this drive
    redCount @6 :UInt8;        # red lamps that passed the lamp test this frame
    greenCount @7 :UInt8;
    redX @8 :Float32;          # full-res px of the lit red (-1 when none)
    redY @9 :Float32;
    reason @10 :Text;          # chime pairing, e.g. "red(1406,233)->green(1404,306)"
    frameReason @11 :Text;     # "" when a frame was scored; no_connection / no_frame / stale_frame / budget
    scanMs @12 :Float32;       # detector time for this frame
    speed @13 :Float32;        # m/s, livePose speed the gate used
  }
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
