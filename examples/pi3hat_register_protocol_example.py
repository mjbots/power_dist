#!/usr/bin/python3

# Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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

'''Demonstrates how to use the register protocol of the power_dist
from the pi3hat.'''

import asyncio
import moteus
import moteus_pi3hat
import struct
import time


async def main():
    # We will always ask for a query in the same form from the
    # power_dist, so we can unpack it with a fixed struct.  This
    # structure was determined by looking through the moteus reference
    # documentation section A "CAN Format".
    query_data = bytes([
            0x17,  # read 3 int16 registers
            0x10,  # starting at register 0x010 (voltage, current, temp)
            0x19,  # read 1 int32 register
            0x13,  # starting at register 0x013 (energy)
            0x11,  # read 1 int8 register
            0x02,  # switch
            ])

    # The corresponding reply structure will look like:
    #  0x27 reply 3 int16
    #  0x10 at register 0x010
    #    0xDE 0xAD voltage
    #    0xBE 0xEF current
    #    0x77 0x88 temperature
    #  0x29 reply 1 int32
    #  0x23 at register 0x13
    #    0x01 0x02 0x03 0x03  energy
    #  0x21 reply 1 int 8
    #  0x02 at register 0x02
    #    0x00 switch
    fmt = struct.Struct('<bbhhhbbibbb')

    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            # Assume the power_dist (id=32) is on JC5.  This isn't
            # strictly necessary, as we'll be manually creating
            # messages and assigning their bus, but it doesn't hurt.
            5 : [32],
        })

    while True:
        # Generate our raw CAN frame.
        msg = moteus.Command()
        msg.raw = True
        msg.arbitration_id = 0x8020  # from 0 to 32, with query
        msg.bus = 5  # We'll assume the power_dist is on JC5
        msg.reply_required = True  # We expect the power_dist to reply to us
        msg.data = query_data

        # Send the query and wait for any responses.
        results = await transport.cycle([
            msg,

            # You could include commands for individual moteus
            # controllers here too, like the following (note the
            # "make_" variant *NOT* the "set_" variant.
            #
            # c.make_position(position=0.1, velocity=0.2),
        ])

        for result in results:
            if result.arbitration_id == 0x2000 and len(result.data) >= fmt.size:

                print(f"{time.time()} from power_dist='{result.data.hex()}'")

                # This came from the power_dist.
                unpacked = fmt.unpack(result.data[0:fmt.size])

                # Verify that we got the structure that we expect.
                if not (unpacked[0] == 0x27 and
                        unpacked[1] == 0x10 and
                        unpacked[5] == 0x29 and
                        unpacked[6] == 0x13 and
                        unpacked[8] == 0x21 and
                        unpacked[9] == 0x02):
                    print(f"Unexpected result from power_dist {unpacked}")
                    continue

                voltage = unpacked[2] * 0.1
                current = unpacked[3] * 0.1
                temperature = unpacked[4] * 0.1
                energy = unpacked[7] * 0.000001
                switch = unpacked[10] != 0

                print(f"{time.time()} voltage={voltage} current={current} " +
                      f"temperature={temperature} energy={energy} switch={switch}")

        await asyncio.sleep(0.1)

if __name__ == '__main__':
    asyncio.run(main())
