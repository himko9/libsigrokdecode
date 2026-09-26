# Logic Analyzer Protocol Decoders

Python protocol decoders for libsigrokdecode.

## Current decoders

| Decoder | ID | Description | Source |
| --- | --- | --- | --- |
| AMD Serial VID Interface 2 | `svi2` | AMD SoC <-> VR Protocol| [decoders/svi2](decoders/svi2/) |

## Requirements and usage

The decoder uses libsigrokdecode API version 3 and includes compatibility handling
for upstream libsigrokdecode and DSView's libsigrokdecode4DSL.

1. Copy the complete `decoders/` directory into the decoder directory used by
   your libsigrokdecode application, such as PulseView or DSView.
2. Restart the application and open a logic capture.
3. Add the decoder and assign the captured signals.

---

### License and disclaimer:
This project's source code is licensed under [GPL-3.0](https://www.gnu.org/licenses/gpl-3.0.html)  or a later version of the license.


```
/*
 * This file is part of the "libsigrokdecode" project.
 *
 * Copyright (C) 2026 @himko9 <me@himko.dev>
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