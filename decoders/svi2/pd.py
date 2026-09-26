# SPDX-License-Identifier: GPL-3.0-or-later
"""
libsigrokdecode decoder for AMD Serial VID Interface 2 (SVI2).

The wire format implemented here follows AMD publication #48022.
SVD and SVT are deliberately decoded by independent state machines:
the specification permits the processor and voltage regulator to start a
packet at the same time.

Hardware testbench:
AMD Qogir-MTS, R5 3600X
DSLogic U2P, DSView


/*
 * This file is part of the "libsigrokdecode" distribution.
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
"""

from __future__ import annotations

import sigrokdecode as srd


def _bits_to_int(bits):
    value = 0
    for bit, _sample in bits:
        value = (value << 1) | bit
    return value


def _decode_serial_vid(code, width):
    """
    Return (status, microvolts) from SVI2 Table.

    Command VID is the eight-bit Table 10 value. Telemetry adds bit 8: values
    below 1.55625 V use ``1_SVID[7:0]`` and values above 1.55000 V count down
    from ``0x0ff``. Only telemetry code 0x1f8 is the valid 0 V/OFF encoding.
    """
    if width == 8:
        if not 0 <= code <= 0xff:
            return "invalid", None
        if code >= 0xf8:
            return "off", None
        return "voltage", 1_550_000 - code * 6_250

    if width == 9:
        if not 0 <= code <= 0x1ff:
            return "invalid", None
        if code <= 0x0ff:
            return "voltage", 1_550_000 + (0x100 - code) * 6_250
        if code <= 0x1f7:
            return "voltage", 1_550_000 - (code & 0xff) * 6_250
        if code == 0x1f8:
            return "off", 0
        return "invalid", None

    return "invalid", None


def _format_microvolts(microvolts):
    return "%d.%05d V" % (
        microvolts // 1_000_000,
        (microvolts % 1_000_000) // 10,
    )


def _format_vid(code, width):
    status, microvolts = _decode_serial_vid(code, width)
    code_text = "0x%0*X" % ((width + 3) // 4, code)
    if status == "voltage":
        return "%s (%s)" % (_format_microvolts(microvolts), code_text)
    if status == "off":
        if microvolts == 0:
            return "OFF / 0.00000 V (%s)" % code_text
        return "OFF (%s)" % code_text
    return "invalid voltage code %s" % code_text


def _matched_channels(condition_channels, matched):
    """
    Normalize upstream tuple and DSView integer match results.

    Upstream libsigrokdecode API v3 exposes ``self.matched`` as one Boolean
    per wait condition.
    
    DSView's libsigrokdecode4DSL exposes the equivalent
    information as an integer bitmask.
    
    Supporting both keeps the decoder usable in PulseView/sigrok-cli and DSView.
    """
    if matched is None:
        return None
    if isinstance(matched, int):
        return {
            channel for index, channel in enumerate(condition_channels)
            if matched & (1 << index)
        }
    return {
        channel for channel, is_match in zip(condition_channels, matched)
        if is_match
    }


class Decoder(srd.Decoder):
    api_version = 3
    id = "svi2"
    name = "SVI2"
    longname = "AMD Serial VID Interface 2"
    desc = "AMD SVI2 command, acknowledgement, and telemetry bus."
    license = "gplv3+"
    inputs = ["logic"]
    outputs = ["svi2"]
    tags = ["Embedded/industrial"]

    channels = (
        {"id": "svc", "name": "SVC", "desc": "Serial VID clock"},
        {"id": "svd", "name": "SVD", "desc": "Processor-to-VR data"},
        {"id": "svt", "name": "SVT", "desc": "VR-to-processor telemetry"},
    )
    optional_channels = (
        {"id": "pwrok", "name": "PWROK", "desc": "Power-good qualifier"},
    )
    options = (
        {
            "id": "show_bits",
            "desc": "Show sampled bit annotations",
            "default": "no",
            "values": ("yes", "no"),
        },
    )
    annotations = (
        ("start", "Start"),
        ("stop", "Stop"),
        ("data-byte", "Data byte"),
        ("ack", "ACK"),
        ("nack", "NACK / no ACK"),
        ("command", "Command"),
        ("field", "Payload field"),
        ("telemetry", "Telemetry"),
        ("votf", "VOTF complete"),
        ("warning", "Warning"),
        ("bit", "Bit"),
        ("pwrok", "PWROK"),
        ("voltage", "Decoded voltage"),
    )
    annotation_rows = (
        ("framing", "Framing", (0, 1, 11)),
        ("command", "SVD command", (2, 3, 4, 5, 6)),
        ("telemetry", "SVT telemetry", (7, 8)),
        ("voltage", "Decoded voltage", (12,)),
        ("bits", "Bits", (10,)),
        ("warnings", "Warnings", (9,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev = None
        self.bit_width = 1
        self.last_svc_rise = None
        self.command_active = False
        self.command_start = 0
        self.command_bits = []
        self.command_stop_clocks = 0
        self.command_framing_low = False
        self.command_framing_error_reported = False
        self.telemetry_active = False
        self.telemetry_start = 0
        self.telemetry_bits = []
        self.telemetry_expected = None
        self.telemetry_stop_clocks = 0
        self.telemetry_framing_low = False
        self.telemetry_framing_error_reported = False
        self.pwrok_low = False
        self.have_pwrok = False

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.show_bits = self.options.get("show_bits", "no") == "yes"
        self.have_pwrok = bool(
            hasattr(self, "has_channel") and self.has_channel(3))

    def _put_ann(self, ss, es, ann, *texts):
        self.put(int(ss), max(int(es), int(ss) + 1), self.out_ann,
                 [ann, list(texts)])

    def _put_python(self, ss, es, kind, data):
        self.put(int(ss), max(int(es), int(ss) + 1), self.out_python,
                 [kind, data])

    def _span(self, bits, first, last):
        ss = bits[first][1]
        if last + 1 < len(bits):
            es = bits[last + 1][1]
        else:
            es = bits[last][1] + self.bit_width
        return ss, es

    def _field(self, bits, first, last, text):
        ss, es = self._span(bits, first, last)
        self._put_ann(ss, es, 6, text)

    def _reset_packets(self, sample, reason=None):
        if reason and (self.command_active or self.telemetry_active):
            start = min(
                self.command_start if self.command_active else sample,
                self.telemetry_start if self.telemetry_active else sample,
            )
            self._put_ann(start, sample, 9, reason, "Reset")
        self.command_active = False
        self.command_bits = []
        self.command_stop_clocks = 0
        self.command_framing_low = False
        self.command_framing_error_reported = False
        self.telemetry_active = False
        self.telemetry_bits = []
        self.telemetry_expected = None
        self.telemetry_stop_clocks = 0
        self.telemetry_framing_low = False
        self.telemetry_framing_error_reported = False

    def _start_command(self, sample):
        if self.command_active:
            self._put_ann(self.command_start, sample, 9,
                          "SVD restart before STOP", "Restart")
        self.command_active = True
        self.command_start = sample
        self.command_bits = []
        self.command_stop_clocks = 0
        self.command_framing_low = False
        self.command_framing_error_reported = False

    def _report_command_framing_error(self, sample, detail):
        if self.command_framing_error_reported:
            return
        self.command_framing_error_reported = True
        self._put_python(self.command_start, sample, "ERROR", {
            "bus": "SVD",
            "reason": "framing",
            "detail": detail,
            "bits": len(self.command_bits),
            "post_ack_clocks": self.command_stop_clocks,
        })

    def _start_telemetry(self, sample):
        if self.telemetry_active:
            self._put_ann(self.telemetry_start, sample, 9,
                          "SVT restart before STOP", "Restart")
        self.telemetry_active = True
        self.telemetry_start = sample
        self.telemetry_bits = []
        self.telemetry_expected = None
        self.telemetry_stop_clocks = 0
        self.telemetry_framing_low = False
        self.telemetry_framing_error_reported = False

    def _report_telemetry_framing_error(self, sample, detail):
        if self.telemetry_framing_error_reported:
            return
        self.telemetry_framing_error_reported = True
        self._put_python(self.telemetry_start, sample, "ERROR", {
            "bus": "SVT",
            "reason": "framing",
            "detail": detail,
            "bits": len(self.telemetry_bits),
            "post_payload_clocks": self.telemetry_stop_clocks,
        })

    def _sample_command_bit(self, bit, sample):
        if not self.command_active:
            return
        # After ACK3, SVC must fall and rise once more while SVD is low so the
        # master can create STOP by raising SVD during that high phase.  That
        # rising edge is framing, not a 28th bit time.
        if len(self.command_bits) == 27:
            self.command_stop_clocks += 1
            if self.command_stop_clocks == 1:
                self.command_framing_low = bit == 0
                if not self.command_framing_low:
                    self._put_ann(
                        self.command_start, sample + self.bit_width, 9,
                        "SVD was not low on the post-ACK framing clock",
                        "Bad STOP framing")
                    self._report_command_framing_error(
                        sample + self.bit_width, "framing_clock_not_low")
            elif self.command_stop_clocks == 2:
                self._put_ann(self.command_start, sample + self.bit_width, 9,
                              "SVD STOP was not observed", "Missing STOP")
                self._report_command_framing_error(
                    sample + self.bit_width, "missing_stop")
            return
        self.command_bits.append((bit, sample))
        index = len(self.command_bits)
        if self.show_bits:
            self._put_ann(sample, sample + self.bit_width, 10,
                          "SVD bit %d: %d" % (index, bit), str(bit))
        if index > 27:
            self._put_ann(self.command_start, sample + self.bit_width, 9,
                          "SVD packet exceeds 27 bit times", "Too long")

    def _sample_telemetry_bit(self, bit, sample):
        if not self.telemetry_active:
            return
        # The STOP transition may occur in the SVC-high period following the
        # selected 2- or 20-bit payload.  Some sources provide a complete low
        # framing clock before raising SVT.  That clock is not payload bit 3
        # or 21.
        if self.telemetry_expected is not None and \
                len(self.telemetry_bits) >= self.telemetry_expected:
            self.telemetry_stop_clocks += 1
            if self.telemetry_stop_clocks == 1:
                self.telemetry_framing_low = bit == 0
                if not self.telemetry_framing_low:
                    self._put_ann(
                        self.telemetry_start, sample + self.bit_width, 9,
                        "SVT was not low on the post-payload framing clock",
                        "Bad STOP framing")
                    self._report_telemetry_framing_error(
                        sample + self.bit_width, "framing_clock_not_low")
            elif self.telemetry_stop_clocks == 2:
                self._put_ann(self.telemetry_start,
                              sample + self.bit_width, 9,
                              "SVT STOP was not observed", "Missing STOP")
                self._report_telemetry_framing_error(
                    sample + self.bit_width, "missing_stop")
            return
        self.telemetry_bits.append((bit, sample))
        index = len(self.telemetry_bits)
        if self.show_bits:
            self._put_ann(sample, sample + self.bit_width, 10,
                          "SVT bit %d: %d" % (index, bit), str(bit))
        if index == 2:
            prefix = _bits_to_int(self.telemetry_bits)
            self.telemetry_expected = 2 if prefix == 0b10 else 20

    def _finish_command(self, sample):
        bits = self.command_bits
        self._put_ann(self.command_start,
                      bits[0][1] if bits else sample, 0,
                      "SVD START", "START", "S")
        self._put_ann(bits[-1][1] if bits else self.command_start,
                      sample, 1, "SVD STOP", "STOP", "P")
        if len(bits) != 27:
            self._put_ann(self.command_start, sample, 9,
                          "Bad SVD length: %d (expected 27)" % len(bits),
                          "Length %d" % len(bits))
            self._put_python(self.command_start, sample, "ERROR", {
                "bus": "SVD", "reason": "length", "bits": len(bits),
            })
            self.command_active = False
            self.command_bits = []
            self.command_stop_clocks = 0
            self.command_framing_low = False
            self.command_framing_error_reported = False
            return

        byte1 = _bits_to_int(bits[0:8])
        byte2 = _bits_to_int(bits[9:17])
        byte3 = _bits_to_int(bits[18:26])
        raw = (byte1 << 16) | (byte2 << 8) | byte3
        valid_header = (byte1 >> 3) == 0b11000 and not (byte1 & 1)
        framing_ok = self.command_stop_clocks == 1 and \
            self.command_framing_low
        if not framing_ok:
            if self.command_stop_clocks == 0:
                detail = "stop_before_framing_clock"
                warning = "SVD STOP preceded the required post-ACK framing clock"
            elif not self.command_framing_low:
                detail = "framing_clock_not_low"
                warning = "SVD was not low on the post-ACK framing clock"
            else:
                detail = "late_stop"
                warning = "SVD STOP followed more than one framing clock"
            self._put_ann(self.command_start, sample, 9,
                          warning, "Bad STOP framing")
            self._report_command_framing_error(sample, detail)

        for number, (first, last, value) in enumerate((
                (0, 7, byte1), (9, 16, byte2), (18, 25, byte3)), 1):
            ss, es = self._span(bits, first, last)
            self._put_ann(ss, es, 2,
                          "Byte %d: 0x%02X" % (number, value),
                          "B%d=%02X" % (number, value), "%02X" % value)

        acknowledgements = []
        for number, index in enumerate((8, 17, 26), 1):
            bit = bits[index][0]
            ss, es = self._span(bits, index, index)
            acknowledgements.append(bit == 0)
            if bit == 0:
                self._put_ann(ss, es, 3, "ACK%d (low)" % number,
                              "ACK%d" % number, "A")
            else:
                self._put_ann(ss, es, 4,
                              "No ACK%d (high)" % number,
                              "NACK%d" % number, "N")

        domain_mask = ((byte1 >> 2) & 1) | (((byte1 >> 1) & 1) << 1)
        vid = ((byte2 & 0x7f) << 1) | ((byte3 >> 7) & 1)
        psi0_l = (byte2 >> 7) & 1
        psi1_l = (byte3 >> 6) & 1
        tfn = (byte3 >> 5) & 1
        ll_trim = (byte3 >> 2) & 7
        offset_trim = byte3 & 3

        self._field(bits, 0, 4,
                    "Fixed prefix: {:05b}".format(byte1 >> 3))
        self._field(bits, 5, 6,
                    "Domains: VDD1=%d VDD2=%d" %
                    (domain_mask & 1, (domain_mask >> 1) & 1))
        self._field(bits, 7, 7, "Reserved: %d" % (byte1 & 1))
        self._field(bits, 9, 9, "PSI0_L: %d" % psi0_l)
        self._field(bits, 10, 16, "VID[7:1]: 0x%02X" % (vid >> 1))
        self._field(bits, 18, 18, "VID[0]: %d" % (vid & 1))
        self._field(bits, 19, 19, "PSI1_L: %d" % psi1_l)
        self._field(bits, 20, 20, "TFN: %d" % tfn)
        self._field(bits, 21, 23, "Load-line trim: %d" % ll_trim)
        self._field(bits, 24, 25, "Offset trim: %d" % offset_trim)

        domains = "+".join(name for selected, name in (
            (domain_mask & 1, "VDD1"), (domain_mask & 2, "VDD2")) if selected)
        if not domains:
            domains = "none"
        vid_status, vid_microvolts = _decode_serial_vid(vid, 8)
        voltage_ss, voltage_es = self._span(bits, 10, 18)
        self._put_ann(
            voltage_ss, voltage_es, 12,
            "%s requested voltage: %s" % (domains, _format_vid(vid, 8)),
            "%s %s" % (domains, _format_vid(vid, 8)),
            _format_vid(vid, 8))
        summary = "SVI2 command %s VID=0x%02X PSI=%d/%d TFN=%d LL=%d OFFSET=%d" % (
            domains, vid, psi0_l, psi1_l, tfn, ll_trim, offset_trim)
        self._put_ann(self.command_start, sample, 5, summary,
                      "%s VID=%02X" % (domains, vid), "CMD")
        if not valid_header:
            self._put_ann(self.command_start, sample, 9,
                          "Invalid fixed/reserved command bits", "Bad header")

        self._put_python(self.command_start, sample, "COMMAND", {
            "raw": raw,
            "bytes": (byte1, byte2, byte3),
            "domain_mask": domain_mask,
            "vid": vid,
            "voltage_status": vid_status,
            "voltage_uv": vid_microvolts,
            "voltage_v": (vid_microvolts / 1_000_000.0
                           if vid_microvolts is not None else None),
            "psi0_l": psi0_l,
            "psi1_l": psi1_l,
            "tfn": tfn,
            "load_line_trim": ll_trim,
            "offset_trim": offset_trim,
            "ack": tuple(acknowledgements),
            "valid_header": valid_header,
            "framing_ok": framing_ok,
            "valid": valid_header and framing_ok,
        })
        self.command_active = False
        self.command_bits = []
        self.command_stop_clocks = 0
        self.command_framing_low = False
        self.command_framing_error_reported = False

    def _finish_telemetry(self, sample):
        bits = self.telemetry_bits
        self._put_ann(self.telemetry_start,
                      bits[0][1] if bits else sample, 0,
                      "SVT START", "START", "S")
        self._put_ann(bits[-1][1] if bits else self.telemetry_start,
                      sample, 1, "SVT STOP", "STOP", "P")

        if len(bits) not in (2, 20):
            self._put_ann(self.telemetry_start, sample, 9,
                          "Bad SVT length: %d (expected 2 or 20)" % len(bits),
                          "Length %d" % len(bits))
            self._put_python(self.telemetry_start, sample, "ERROR", {
                "bus": "SVT", "reason": "length", "bits": len(bits),
            })
        elif len(bits) == 2:
            code = _bits_to_int(bits)
            if code == 0b10:
                self._put_ann(self.telemetry_start, sample, 8,
                              "VOTF complete", "VOTF", "V")
                self._put_python(self.telemetry_start, sample,
                                 "VOTF_COMPLETE", {"code": code})
            else:
                self._put_ann(self.telemetry_start, sample, 9,
                              "Invalid 2-bit SVT code: {:02b}".format(code),
                              "Bad SVT")
        else:
            raw = _bits_to_int(bits)
            kind = raw >> 18
            voltage = (raw >> 9) & 0x1ff
            kind_names = {
                0b00: "VDD1 voltage/current",
                0b01: "VDD2 voltage/current",
                0b11: "voltage-only",
            }
            name = kind_names.get(kind, "reserved")
            ss, es = self._span(bits, 0, 1)
            self._put_ann(ss, es, 6,
                          "SVT type: {:02b} ({})".format(kind, name))
            if kind == 0b11:
                voltage2 = raw & 0x1ff
                self._field(bits, 2, 10, "VDD1 voltage: 0x%03X" % voltage)
                self._field(bits, 11, 19, "VDD2 voltage: 0x%03X" % voltage2)
                voltage1_status, voltage1_uv = _decode_serial_vid(voltage, 9)
                voltage2_status, voltage2_uv = _decode_serial_vid(voltage2, 9)
                ss, es = self._span(bits, 2, 10)
                self._put_ann(ss, es, 12,
                              "VDD1 measured voltage: %s" %
                              _format_vid(voltage, 9),
                              "VDD1 %s" % _format_vid(voltage, 9),
                              _format_vid(voltage, 9))
                ss, es = self._span(bits, 11, 19)
                self._put_ann(ss, es, 12,
                              "VDD2 measured voltage: %s" %
                              _format_vid(voltage2, 9),
                              "VDD2 %s" % _format_vid(voltage2, 9),
                              _format_vid(voltage2, 9))
                details = {"mode": "voltage", "vdd1": voltage,
                           "vdd2": voltage2,
                           "vdd1_voltage_status": voltage1_status,
                           "vdd1_voltage_uv": voltage1_uv,
                           "vdd1_voltage_v": (
                               voltage1_uv / 1_000_000.0
                               if voltage1_uv is not None else None),
                           "vdd2_voltage_status": voltage2_status,
                           "vdd2_voltage_uv": voltage2_uv,
                           "vdd2_voltage_v": (
                               voltage2_uv / 1_000_000.0
                               if voltage2_uv is not None else None)}
                short = "VDD1=%03X VDD2=%03X" % (voltage, voltage2)
            elif kind in (0b00, 0b01):
                current = raw & 0xff
                reserved = (raw >> 8) & 1
                domain = 1 if kind == 0 else 2
                self._field(bits, 2, 10,
                            "VDD%d voltage: 0x%03X" % (domain, voltage))
                self._field(bits, 11, 11, "Reserved: %d" % reserved)
                self._field(bits, 12, 19,
                            "VDD%d current: 0x%02X" % (domain, current))
                voltage_status, voltage_uv = _decode_serial_vid(voltage, 9)
                ss, es = self._span(bits, 2, 10)
                self._put_ann(ss, es, 12,
                              "VDD%d measured voltage: %s" %
                              (domain, _format_vid(voltage, 9)),
                              "VDD%d %s" %
                              (domain, _format_vid(voltage, 9)),
                              _format_vid(voltage, 9))
                details = {"mode": "voltage_current", "domain": domain,
                           "voltage": voltage, "current": current,
                           "reserved": reserved,
                           "voltage_status": voltage_status,
                           "voltage_uv": voltage_uv,
                           "voltage_v": (
                               voltage_uv / 1_000_000.0
                               if voltage_uv is not None else None)}
                short = "VDD%d V=%03X I=%02X" % (domain, voltage, current)
                if reserved:
                    self._put_ann(self.telemetry_start, sample, 9,
                                  "Reserved telemetry bit is not zero",
                                  "Reserved=1")
            else:
                details = {"mode": "reserved"}
                short = "reserved"
                self._put_ann(self.telemetry_start, sample, 9,
                              "Reserved SVT type 10b", "Reserved type")

            self._put_ann(self.telemetry_start, sample, 7,
                          "SVI2 telemetry 0x%05X: %s" % (raw, short),
                          "TEL %s" % short, "TEL")
            details.update({"raw": raw, "type": kind})
            self._put_python(self.telemetry_start, sample, "TELEMETRY", details)

        self.telemetry_active = False
        self.telemetry_bits = []
        self.telemetry_expected = None
        self.telemetry_stop_clocks = 0
        self.telemetry_framing_low = False
        self.telemetry_framing_error_reported = False

    def _handle_sample(self, pins, sample, event_channels=None):
        svc, svd, svt = pins[0], pins[1], pins[2]
        pwrok = pins[3] if self.have_pwrok else None
        current = (svc, svd, svt, pwrok)
        if event_channels is None:
            if self.prev is None:
                event_channels = {0, 1, 2}
                if self.have_pwrok:
                    event_channels.add(3)
            else:
                event_channels = {
                    index for index, (old, new) in enumerate(
                        zip(self.prev, current)) if old != new
                }
        if self.prev is None:
            self.pwrok_low = self.have_pwrok and pwrok == 0
            prev_svc = prev_svd = prev_svt = prev_pwrok = None
        else:
            prev_svc, prev_svd, prev_svt, prev_pwrok = self.prev
        svc_rise = 0 in event_channels and svc == 1 and prev_svc != 1
        svd_fall = 1 in event_channels and svd == 0 and prev_svd != 0
        svd_rise = 1 in event_channels and svd == 1 and prev_svd != 1
        svt_fall = 2 in event_channels and svt == 0 and prev_svt != 0
        svt_rise = 2 in event_channels and svt == 1 and prev_svt != 1

        if self.last_svc_rise is not None and svc_rise:
            self.bit_width = max(1, sample - self.last_svc_rise)
        if svc_rise:
            self.last_svc_rise = sample

        if self.have_pwrok and 3 in event_channels and \
                (prev_pwrok is None or pwrok != prev_pwrok):
            if pwrok == 0:
                self._put_ann(sample, sample + 1, 11,
                              "PWROK deasserted", "PWROK low")
                self._reset_packets(sample, "PWROK deasserted")
                self.pwrok_low = True
            else:
                self._put_ann(sample, sample + 1, 11,
                              "PWROK asserted", "PWROK high")
                self.pwrok_low = False

        if not self.pwrok_low:
            if svd_fall and svc == 1:
                self._start_command(sample)
            if svt_fall and svc == 1:
                self._start_telemetry(sample)

            # At modest capture rates DSView can report the STOP transition
            # and the next SVC rising edge at the same sample.  Once the
            # payload is already complete, finish before considering that
            # edge as another bit clock.
            telemetry_complete_at_stop = svt_rise and svc == 1 and \
                self.telemetry_active and \
                self.telemetry_expected is not None and \
                len(self.telemetry_bits) >= self.telemetry_expected
            if telemetry_complete_at_stop:
                self._finish_telemetry(sample)

            if svc_rise:
                self._sample_command_bit(svd, sample)
                if self.telemetry_active:
                    # If STOP and the final sample are quantized together,
                    # the level immediately before STOP is the sampled bit.
                    telemetry_bit = prev_svt if svt_rise and \
                        prev_svt is not None else svt
                    self._sample_telemetry_bit(telemetry_bit, sample)

            if svd_rise and svc == 1 and self.command_active:
                self._finish_command(sample)
            if svt_rise and svc == 1 and self.telemetry_active:
                self._finish_telemetry(sample)

        self.prev = current

    def decode(self):
        conditions = [{0: "e"}, {1: "e"}, {2: "e"}]
        condition_channels = [0, 1, 2]
        if self.have_pwrok:
            conditions.append({3: "e"})
            condition_channels.append(3)
        while True:
            # DSView's libsigrokdecode4DSL reuses one internal tuple for all
            # wait() results.  Copy it immediately: retaining that tuple in
            # ``pins`` while entering the next wait() makes its C extension
            # fail when it tries to refill the still-referenced tuple.
            pins = list(self.wait(conditions))
            event_channels = _matched_channels(
                condition_channels, getattr(self, "matched", None))
            self._handle_sample(pins, self.samplenum, event_channels)
