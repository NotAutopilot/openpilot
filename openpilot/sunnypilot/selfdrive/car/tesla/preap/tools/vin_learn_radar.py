#!/usr/bin/env python3
import time
import struct
import threading
import argparse
from panda import Panda
from openpilot.common.params import Params

# UDS Constants
TX_ID = 0x641
RX_ID = 0x651

def tesla_radar_security_access_algorithm(seed_bytes):
    if len(seed_bytes) != 4:
        raise ValueError(f"security seed must be 4 bytes, got {len(seed_bytes)}")
    seed_val = int.from_bytes(seed_bytes, byteorder="big")
    k4 = (seed_val >> 5 & 8) | (seed_val >> 0xB & 4) | (seed_val >> 0x18 & 1) | (seed_val >> 1 & 2)
    if seed_val & 0x20000 == 0:
        k32 = ((seed_val & ~(0xFF << k4 & 0xFFFFFFFF)) << (0x20 - k4) & 0xFFFFFFFF) | (seed_val >> k4 & 0xFFFFFFFF)
    else:
        k32 = ((~(0xFF << k4 & 0xFFFFFFFF) << (0x20 - k4) & seed_val & 0xFFFFFFFF) >> (0x20 - k4) & 0xFFFFFFFF) | (seed_val << k4 & 0xFFFFFFFF)
    k2 = (seed_val >> 4) & 2 | (seed_val >> 0x1F)
    if k2 == 0:
        key_int = k32 | seed_val
    elif k2 == 1:
        key_int = k32 & seed_val
    elif k2 == 2:
        key_int = k32 ^ seed_val
    else:
        key_int = k32
    key_int &= 0xFFFFFFFF
    return struct.pack("!I", key_int)

class StandaloneFlasher:
    def __init__(self, vin: str):
        self.vin = vin.ljust(17, '0')[:17].encode('ascii')
        from openpilot.common.params import Params
        self.position = int(Params().get("NAPRadarPosition") or 0)
        self.p = Panda()
        self.p.set_safety_mode(17) # SAFETY_ALLOUTPUT
        self.p.can_clear(0xFFFF)
        self.running = True
        self.bus = 1

    def gateway_thread(self):
        # Transmit 0x2A9 and 0x2B9 so the radar can learn the VIN
        while self.running:
            # 0x2A9 config
            msg_2a9 = bytearray(8)
            msg_2a9[0] = 0x44 # carConfig (US, no air susp, P85)
            # Add XWD bit if VIN says AWD (char 8 is '2' or '4')
            drive = self.vin[7]
            if drive in [b'2', b'4', ord('2'), ord('4')]:
                msg_2a9[0] |= 0x08
            msg_2a9[1] = 0x82 # RWD, EPAS type 2
            msg_2a9[2] = 0x20 # AP1, ParkAssist
            msg_2a9[4] = 0x01 | (self.position << 4) # ForwardRadarHW (Bosch) + Position
            
            # 0x2B9 VIN
            idx = int(time.time() * 10) % 7
            msg_2b9 = bytearray(8)
            msg_2b9[0] = idx
            for i in range(7):
                v_idx = idx * 7 + i
                if v_idx < 17:
                    msg_2b9[i+1] = self.vin[v_idx]
            
            self.p.can_send(0x2A9, bytes(msg_2a9), self.bus)
            self.p.can_send(0x2B9, bytes(msg_2b9), self.bus)
            time.sleep(0.1)

    def uds_request(self, payload, expected_pci, timeout=2.0):
        # Single frame
        data = bytearray([len(payload)]) + bytearray(payload)
        data = data.ljust(8, b'\x00')
        self.p.can_send(TX_ID, bytes(data), self.bus)
        
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            rx = self.p.can_recv()
            for addr, dat, src in rx:
                if src == self.bus and addr == RX_ID:
                    if dat[0] == expected_pci and dat[1] == payload[0] + 0x40:
                        return dat
                    # Negative response: 0x03 0x7F [Service] [NRC]
                    if dat[0] == 0x03 and dat[1] == 0x7F and dat[2] == payload[0]:
                        if dat[3] == 0x21: # Busy
                            return b'BUSY'
                        raise Exception(f"UDS Error: {dat.hex()}")
            time.sleep(0.01)
        raise TimeoutError(f"UDS Request timed out for {bytes(payload).hex()}")

    def run(self):
        print("Starting gateway spoofing...")
        gw = threading.Thread(target=self.gateway_thread)
        gw.start()
        
        try:
            print("Sending Tester Present...")
            self.uds_request([0x3E, 0x00], 0x02)
            
            print("Switching to Default Session...")
            self.uds_request([0x10, 0x01], 0x06)
            
            print("Switching to Extended Session...")
            self.uds_request([0x10, 0x03], 0x06)
            
            print("Requesting Seed...")
            resp = self.uds_request([0x27, 0x11], 0x06)
            seed = resp[3:7]
            print(f"Seed: {seed.hex()}")
            
            key = tesla_radar_security_access_algorithm(seed)
            print(f"Key: {key.hex()}")
            
            print("Submitting Key...")
            self.uds_request([0x27, 0x12] + list(key), 0x02)
            
            print("Starting VIN Learn Routine (0x0A03)...")
            self.uds_request([0x31, 0x01, 0x0A, 0x03], 0x04)
            
            print("Waiting for VIN Learn to complete...")
            for i in range(15):
                time.sleep(2)
                try:
                    resp = self.uds_request([0x31, 0x02, 0x0A, 0x03], 0x04)
                    if resp != b'BUSY':
                        break
                except Exception as e:
                    print(f"Stop routine error (ignoring): {e}")
            
            print("Requesting Results...")
            try:
                resp = self.uds_request([0x31, 0x03, 0x0A, 0x03], 0x04)
                print(f"Results: {resp.hex()}")
            except Exception as e:
                print(f"No results response, but routine finished: {e}")
            print("VIN Learn Complete!")
            
        finally:
            self.running = False
            gw.join()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    
    params = Params()
    vin = params.get("NAPRadarDonorVin")
    if not vin or len(vin) != 17:
        print("Error: NAPRadarDonorVin is not set or invalid length.")
        return
        
    print(f"Starting VIN Learn for {vin}...")
    flasher = StandaloneFlasher(vin)
    flasher.run()
    


if __name__ == "__main__":
    main()
