# AMD SVI2 Protocol Decoder

Python protocol decoder for libsigrokdecode.\
Current implements:\
AMD Serial VID Interface 2 (SVI2), following AMD publication #48022.

| Required channels | Optional channels |
| --- | --- |
| SVC, SVD, SVT | PWROK |

## Requirements and usage

The decoder uses libsigrokdecode API version 3 and includes compatibility handling
for upstream libsigrokdecode and DSView's libsigrokdecode4DSL.

1. Copy the complete `decoders/svi2` directory into the decoder directory used by
   your libsigrokdecode application, such as PulseView or DSView.
2. Restart the application and open a logic capture.
3. Add the **SVI2** decoder and assign the captured signals as follows:

| Channel | Purpose | Required |
| --- | --- | --- |
| SVC | Serial VID clock | Yes |
| SVD | Processor-to-voltage-regulator data | Yes |
| SVT | Voltage-regulator-to-processor telemetry | Yes |
| PWROK | Power-good qualifier; decoding is suspended and pending packets are reset while low | No |

## SVI2 features

- Independent command and telemetry decoding, including simultaneous packets.
- Command fields for VDD1/VDD2 selection, VID, PSI, TFN, load-line trim, and offset trim.
- ACK/NACK annotations for each command byte.
- Voltage/current telemetry and dual-domain voltage-only telemetry. Current is
  reported as a raw code; voltage codes are converted to volts, with OFF and
  invalid-code handling.
- VOTF completion notifications and warnings for malformed packets.
- Annotation rows for framing, commands, telemetry, decoded voltage, bits, and warnings.
- Structured Python output events: `COMMAND`, `TELEMETRY`, `VOTF_COMPLETE`, and `ERROR`.

## Hardware testbench

The decoder source records the following testbench:

- AMD Qogir-MTS / Ryzen 5 3600X.
- DSLogic U2P with DSView.
- PXLogic with DSView(fork PXLogic)


---

### License and disclaimer:
This project's source code is licensed under [GPL-3.0](https://www.gnu.org/licenses/gpl-3.0.html)  or a later version of the license.


```
/*
 * This file is part of the "libsigrokdecode" project.
 *
 * Copyright (C) 2022 @himko9 <me@himko.dev>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
 ```