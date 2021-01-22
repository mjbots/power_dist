#!/usr/bin/python3

# Copyright 2021 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import serial
import struct


def main():
    f = serial.Serial(port='/dev/fdcanusb')
    PACKET = struct.Struct('<BBHHhIhhBB')

    while True:
        line = f.readline()

        if not line.startswith(b"rcv "):
            continue

        fields = line.split(b' ')
        if len(fields) < 3:
            continue

        if fields[1] != b'10004':
            continue

        (switch, lock_time, input_10mV, output_10mV,
         isamp_10mA, energy_uW_hr, fet_temp_C, boot_time,
         state, fault_code) = \
            PACKET.unpack(bytes.fromhex(fields[2].decode('latin1'))[0:PACKET.size])

        print(f"sw: {switch}  lock: {lock_time}  input: {input_10mV / 100:.3f} " +
              f"output: {output_10mV / 100:.3f}  isamp: {isamp_10mA / 100:6.3f} " +
              f"energy Whr: {energy_uW_hr / 1e6:.5f}  fet_temp: {fet_temp_C:3d} " +
              f"time: {boot_time}  state: {state}  fault: {fault_code}")



if __name__ == '__main__':
    main()
