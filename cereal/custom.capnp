using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

# NAP live OSM speed-limit sample. SI units. Published by selfdrive.mapd.
struct LiveMapDataNAP @0x81c2f05a394cf4af {
  speedLimit @0 :Float32;            # m/s, 0 if unknown
  speedLimitValid @1 :Bool;
  nextSpeedLimit @2 :Float32;        # m/s, 0 if unknown
  nextSpeedLimitDistance @3 :Float32; # m
  latitude @4 :Float64;
  longitude @5 :Float64;
  bearingDeg @6 :Float32;
  roadName @7 :Text;
  highway @8 :Text;
  wayId @9 :UInt64;
  source @10 :Text;                  # "osm"
  dbLoaded @11 :Bool;
  matchDistance @12 :Float32;        # m, distance to matched way
  onRoundabout @13 :Bool;
  approachingRoundabout @14 :Bool;
  roundaboutDistance @15 :Float32;   # m to circulating way; 0 if on
  roundaboutSpeedLimit @16 :Float32; # m/s, OSM maxspeed on the RB way
  roundaboutWayId @17 :UInt64;
}

# NAP on-drive MUTCD camera speed-sign sample. Display/log only. mph in the field name.
struct LiveSpeedSignNAP @0xaedffd8f31e7b55d {
  mph @0 :Int16;          # posted mph, 0 if none
  conf @1 :Float32;
  valid @2 :Bool;         # live detection including HUD hold
  weightsMissing @3 :Bool; # ONNX failed to load; HUD shows NO WT, never a fake mph
  detectPaused @4 :Bool;   # HUD WAIT if set. Engaged detect leaves this false.
}

# NAP radar cone-line sample. Log only. Does not steer.
# lat*, wouldLimit, and wouldSteer are path-relative device y, +right
# (modelV2.position.y and PathObstacle y). radarState.yRel is +left;
# device y = -yRel. side is the real side of the lane and therefore
# has the opposite sign from lat: +1 left, -1 right.
# wouldSteer is the shift away from the line, capped at 0.4 m, for
# later evaluation. controlsd does not apply it.
struct ConeLineNAP @0xf35cc4560bbf6ec2 {
  active @0 :Bool;
  side @1 :Int8;             # +1 left of the path, -1 right, 0 none
  confidence @2 :Float32;
  count @3 :UInt8;
  latNear @4 :Float32;       # path-relative y at ~15 m, +right
  latMid @5 :Float32;        # ~30 m, +right
  latFar @6 :Float32;        # ~45 m, +right
  wouldLimit @7 :Float32;    # m, +right; shift that would leave ~1 m of clearance
  barrier @8 :Bool;          # continuous guardrail / barrier, not a cone line
  parked @9 :Bool;           # short wide cluster rejected as a parked car
  spanM @10 :Float32;
  wouldSteer @11 :Float32;   # m, +right; proposed offset away from the line, |v| <= 0.4. Not applied.
}

# NAP in-path / roadside obstacle sample. Log only, plus an animal/person
# chime. y is path-relative, +right (device frame; radarState.yRel is +left
# and y = -yRel). Vision fields are NaN until a radar trigger asks for a
# patch. brakeGate is a future hook and is not applied.
struct PathObstacleNAP @0xda96579883444c35 {
  active @0 :Bool;
  trackId @1 :UInt64;
  range @2 :Float32;             # m, camera frame
  lateral @3 :Float32;           # m, path-relative, +right
  vRel @4 :Float32;              # m/s, radar longitudinal relative
  vLat @5 :Float32;              # m/s, +right
  radarConf @6 :Float32;         # 0..1
  visionConf @7 :Float32;        # 0..1, NaN if not evaluated
  visionEvaluated @8 :Bool;
  lightingScore @9 :Float32;     # 0 poor .. 1 good
  wRadar @10 :Float32;
  wVision @11 :Float32;
  fusedScore @12 :Float32;
  agree @13 :Bool;               # both confidences over their bars
  rejectReason @14 :Text;
  inPath @15 :Bool;
  timeToReach @16 :Float32;      # s, NaN if not closing
  objectClass @17 :ObjectClass;
  visionConfHuman @18 :Float32;
  visionConfAnimal @19 :Float32;
  visionConfObstacle @20 :Float32;
  clusterCount @21 :UInt8;
  spanM @22 :Float32;            # lateral span of the cluster, m
  timeToEnter @23 :Float32;      # s, 0 if already in the path
  entering @24 :Bool;
  zone @25 :Zone;
  brakeGate @26 :Bool;           # log-only: agree + in path or entering, any class
  chimed @27 :Bool;
  chimeReason @28 :Text;
  scanUs @29 :Float32;
  visionUs @30 :Float32;
  livelyScore @31 :Float32;     # 0 still .. 1 moved in the last ~2 s
  # Radar-stage fields forwarded to the camera helper. Additive.
  y @32 :Float32;                # device frame, +right
  along @33 :Float32;            # m/s ground speed along the road
  radarClass @34 :ObjectClass;
  memberIds @35 :List(UInt64);
  laneProbMin @36 :Float32;      # min(left, right) lane prob, for lighting
  pathYStd3s @37 :Float32;       # model y std near t=3 s
  overBudget @38 :Bool;
  heartbeat @39 :Bool;
  # Night drive-over. Radard sets modelLeadAgree when a model or radar
  # lead sits on this stationary in-lane return. driveOverReason is the
  # chime-side suppression (light, no_solid, beyond_60) and is empty
  # until the camera helper decides. Daytime lead matches stay quiet.
  modelLeadAgree @40 :Bool;
  driveOverReason @41 :Text;

  enum ObjectClass {
    unknown @0;
    human @1;
    animal @2;
    obstacle @3;
  }

  enum Zone {
    none @0;
    inPath @1;
    entering @2;
    roadside @3;
  }
}

# Camera-helper result. Same type id as the old empty CustomReserved4.
# The chime reads this. radard does not.
struct PathObstacleVisionNAP @0x80ae746ee2596b11 {
  trackId @0 :UInt64;
  visionEvaluated @1 :Bool;
  visionConf @2 :Float32;
  visionConfHuman @3 :Float32;
  visionConfAnimal @4 :Float32;
  visionConfObstacle @5 :Float32;
  lightingScore @6 :Float32;
  wRadar @7 :Float32;
  wVision @8 :Float32;
  fusedScore @9 :Float32;
  agree @10 :Bool;
  objectClass @11 :PathObstacleNAP.ObjectClass;
  zone @12 :PathObstacleNAP.Zone;
  brakeGate @13 :Bool;
  chimed @14 :Bool;
  chimeReason @15 :Text;
  livelyScore @16 :Float32;
  visionUs @17 :Float32;
  # Set when this attempt produced no score. Empty when a score was written.
  # no_connection, no_frame, stale_frame, roi_out_of_frame, budget, model_error.
  visionFailReason @18 :Text;
  # Night drive-over suppression. Empty when the rule did not suppress.
  # light: crop is headlights or taillights. no_solid: nothing solid in
  # the crop and no model lead. beyond_60: farther than low-beam range.
  driveOverReason @19 :Text;
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
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
