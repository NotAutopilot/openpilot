# Minimal version-0 CarParamsSP/CarStateSP schema for cross-version fixtures.
# Unique file ID so this can be loaded beside current custom.capnp.
@0xce11a00000000001;

struct CarParamsSP {
  flags @0 :UInt32;
  safetyParam @1 :Int16;
  neuralNetworkLateralControl @2 :NeuralNetworkLateralControl;
  pcmCruiseSpeed @3 :Bool;
  intelligentCruiseButtonManagementAvailable @4 :Bool;
  enableGasInterceptor @5 :Bool;

  struct NeuralNetworkLateralControl {
    model @0 :Model;
    fuzzyFingerprint @1 :Bool;

    struct Model {
      path @0 :Text;
      name @1 :Text;
    }
  }
}

struct CarStateSP {
  speedLimit @0 :Float32;
}
