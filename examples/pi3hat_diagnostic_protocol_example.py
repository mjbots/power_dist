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

'''Demonstrates how to use the diagnostic protocol of the power_dist
from the pi3hat.
'''

import asyncio
import moteus
import moteus_pi3hat
import struct
import time


async def main():
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            5 : [32],
        })

    # The diagnostic protocol is stateful, and consumes more CAN
    # bandwidth than the register protocol.  However, as there is
    # currently no python decoding library for the power_dist, if you
    # want nice python objects without much work, it might be useful.
    #
    # To use it, pretend we are talking to moteus controller at id 32,
    # then create a diagnostic protocol `Stream`.

    power_dist = moteus.Controller(id=32, transport=transport)
    stream = moteus.Stream(power_dist)

    while True:
        power = await stream.read_data("power")

        # Show all the things in the diagnostic data structure.
        print(f"{time.time()} {power}")

        # Or you can individual items out.
        voltage = power.input_voltage_V
        output_current = power.output_current_A
        energy_uW_hr = power.energy_uW_hr

        print(f"{time.time()} voltage={voltage} output_current={output_current} energy={energy_uW_hr}")

        await asyncio.sleep(0.1)


if __name__ == '__main__':
    asyncio.run(main())
