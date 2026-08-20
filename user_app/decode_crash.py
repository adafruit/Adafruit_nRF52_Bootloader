#!/usr/bin/env python3
#
# The MIT License (MIT)
#
# Copyright (c) 2026 Alastair D'Silva
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

"""
Adafruit nRF52 UF2 Bootloader - Crash Dump Decoder & Symbol Resolver

Resolves raw PC, LR, and call stack addresses from 'CRASH.TXT' or serial logs
into exact source filenames, function names, and line numbers using addr2line.
"""

import sys
import os
import re
import shutil
import argparse
import subprocess

CFSR_FLAGS = {
    # UsageFault
    (1 << 25): "DIVBYZERO (Divide by zero trap)",
    (1 << 24): "UNALIGNED (Unaligned access trap)",
    (1 << 19): "NOCP (No coprocessor / FPU disabled)",
    (1 << 18): "INVPC (Invalid PC load / EXC_RETURN)",
    (1 << 17): "INVSTATE (Invalid EPSR state / ARM mode)",
    (1 << 16): "UNDEFINSTR (Undefined instruction)",
    # BusFault
    (1 << 15): "BFARVALID (BFAR holds valid fault address)",
    (1 << 13): "LSPERR (BusFault during FP lazy state preservation)",
    (1 << 12): "STKERR (BusFault on exception entry stacking)",
    (1 << 11): "UNSTKERR (BusFault on exception return unstacking)",
    (1 << 10): "IMPRECISERR (Imprecise data bus error)",
    (1 << 9):  "PRECISERR (Precise data bus error)",
    (1 << 8):  "IBUSERR (Instruction bus error)",
    # MemManage
    (1 << 7):  "MMARVALID (MMFAR holds valid fault address)",
    (1 << 5):  "MLSPERR (MemManage fault during FP lazy preservation)",
    (1 << 4):  "MSTKERR (MemManage fault on exception stacking)",
    (1 << 3):  "MUNSTKERR (MemManage fault on exception unstacking)",
    (1 << 1):  "DACCVIOL (Data access violation / MPU fault)",
    (1 << 0):  "IACCVIOL (Instruction access violation)",
}

HFSR_FLAGS = {
    (1 << 31): "DEBUGEVT (HardFault triggered by debug event)",
    (1 << 30): "FORCED (Forced HardFault - escalated from configurable fault)",
    (1 << 1):  "VECTTBL (Vector table read fault)",
}

REASON_NAMES = {
    0x00000001: "HardFault (CPU Exception)",
    0x00000002: "MemManage (MPU/Memory Protection Fault)",
    0x00000003: "BusFault (Bus / AHB / APB Access Error)",
    0x00000004: "UsageFault (Undefined instruction / Unaligned / DivByZero)",
    0x00000005: "Kernel Panic (OS Fatal Error)",
    0x00000006: "Assertion Failure",
    0x0000001A: "Zephyr CPU Exception (K_ERR_CPU_EXCEPTION / 0x1A)",
    0x0000001B: "Zephyr Kernel OOPS (K_ERR_KERNEL_OOPS / 0x1B)",
    0x0000001C: "Zephyr Kernel Panic (K_ERR_KERNEL_PANIC / 0x1C)",
    0x0000001D: "Zephyr Spurious IRQ (K_ERR_SPURIOUS_IRQ / 0x1D)",
    0x5744544F: "Watchdog Timeout ('WDTO')",
}

def find_default_addr2line(user_specified=None):
    if user_specified and shutil.which(user_specified):
        return user_specified
    candidates = [
        "arm-none-eabi-addr2line",
        "arm-zephyr-eabi-addr2line",
        "llvm-symbolizer",
    ]
    for c in candidates:
        if shutil.which(c):
            return c
    
    # Check common Zephyr SDK locations
    import glob
    home = os.path.expanduser("~")
    sdk_patterns = [
        os.path.join(home, ".local/opt/zephyr-sdk-*/arm-zephyr-eabi/bin/arm-zephyr-eabi-addr2line"),
        os.path.join(home, "zephyr-sdk-*/arm-zephyr-eabi/bin/arm-zephyr-eabi-addr2line"),
        "/opt/zephyr-sdk-*/arm-zephyr-eabi/bin/arm-zephyr-eabi-addr2line",
    ]
    for pattern in sdk_patterns:
        matches = glob.glob(pattern)
        if matches:
            return matches[-1]
            
    return None

def find_crash_txt_auto():
    # Common mount points on Linux and macOS
    search_dirs = [
        "/run/media",
        "/media",
        "/Volumes",
    ]
    for base in search_dirs:
        if not os.path.exists(base):
            continue
        for root, dirs, files in os.walk(base):
            for f in files:
                if f.upper() == "CRASH.TXT":
                    return os.path.join(root, f)
            # Limit depth
            if root.count(os.sep) - base.count(os.sep) >= 2:
                dirs.clear()
    return None

def resolve_address(addr2line_cmd, elf_path, address):
    try:
        cmd = [addr2line_cmd, "-e", elf_path, "-f", "-C", "-p", f"0x{address:08x}"]
        res = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode("utf-8", errors="replace").strip()
        return res
    except Exception:
        return "??:0"

def decode_flags(val, flag_map):
    matches = []
    for mask, desc in flag_map.items():
        if (val & mask) == mask:
            matches.append(desc)
    return matches

def main():
    parser = argparse.ArgumentParser(
        description="Decode Adafruit nRF52 UF2 Bootloader CRASH.TXT with ELF symbols."
    )
    parser.add_argument(
        "-e", "--elf",
        required=True,
        help="Path to application ELF binary with debug symbols (e.g. firmware.elf or zephyr.elf)"
    )
    parser.add_argument(
        "-f", "--file",
        help="Path to CRASH.TXT or serial log file (auto-detected from mounted USB drive if omitted)"
    )
    parser.add_argument(
        "-a", "--addr2line",
        help="Custom path or command name for addr2line utility"
    )

    args = parser.parse_args()

    if not os.path.exists(args.elf):
        print(f"Error: ELF file '{args.elf}' not found.", file=sys.stderr)
        sys.exit(1)

    addr2line = find_default_addr2line(args.addr2line)
    if not addr2line:
        print("Error: No addr2line utility found (arm-none-eabi-addr2line or arm-zephyr-eabi-addr2line).", file=sys.stderr)
        sys.exit(1)

    input_text = ""
    source_name = ""

    if args.file:
        if not os.path.exists(args.file):
            print(f"Error: Crash file '{args.file}' not found.", file=sys.stderr)
            sys.exit(1)
        with open(args.file, "r", encoding="utf-8", errors="replace") as f:
            input_text = f.read()
        source_name = args.file
    elif not sys.stdin.isatty():
        input_text = sys.stdin.read()
        source_name = "<stdin>"
    else:
        auto_path = find_crash_txt_auto()
        if auto_path and os.path.exists(auto_path):
            print(f"Auto-detected mounted crash file: {auto_path}")
            with open(auto_path, "r", encoding="utf-8", errors="replace") as f:
                input_text = f.read()
            source_name = auto_path
        else:
            print("Error: No input file specified and CRASH.TXT not detected on mounted USB drive.", file=sys.stderr)
            parser.print_help()
            sys.exit(1)

    print("=" * 70)
    print(" Adafruit nRF52 UF2 Bootloader - Crash Dump Analysis")
    print(f" Source:    {source_name}")
    print(f" Symbols:   {args.elf}")
    print(f" Tool:      {addr2line}")
    print("=" * 70)

    # Parse Version
    version_match = re.search(r"Version:\s*(\d+)", input_text, re.IGNORECASE)
    if version_match:
        print(f"\n[Dump Version] v{version_match.group(1)}")

    # Parse Reason
    reason_match = re.search(r"Reason:\s*0x([0-9a-fA-F]+)", input_text, re.IGNORECASE)
    if reason_match:
        reason_val = int(reason_match.group(1), 16)
        reason_name = REASON_NAMES.get(reason_val, f"Custom / Unknown (0x{reason_val:08X})")
        if not version_match:
            print(f"\n[Crash Reason] 0x{reason_val:08X} -> {reason_name}")
        else:
            print(f"[Crash Reason] 0x{reason_val:08X} -> {reason_name}")

    # Parse Fault Registers
    cfsr_match = re.search(r"CFSR:\s*0x([0-9a-fA-F]+)", input_text, re.IGNORECASE)
    if cfsr_match:
        cfsr_val = int(cfsr_match.group(1), 16)
        flags = decode_flags(cfsr_val, CFSR_FLAGS)
        print(f"[Fault Status] CFSR: 0x{cfsr_val:08X}")
        for flag in flags:
            print(f"               * {flag}")

    hfsr_match = re.search(r"HFSR:\s*0x([0-9a-fA-F]+)", input_text, re.IGNORECASE)
    if hfsr_match:
        hfsr_val = int(hfsr_match.group(1), 16)
        flags = decode_flags(hfsr_val, HFSR_FLAGS)
        if flags:
            print(f"[Fault Status] HFSR: 0x{hfsr_val:08X}")
            for flag in flags:
                print(f"               * {flag}")

    # Parse PC and LR
    pc_match = re.search(r"PC:\s*0x([0-9a-fA-F]+)", input_text, re.IGNORECASE)
    lr_match = re.search(r"LR:\s*0x([0-9a-fA-F]+)", input_text, re.IGNORECASE)

    if pc_match:
        pc_val = int(pc_match.group(1), 16)
        sym = resolve_address(addr2line, args.elf, pc_val)
        print(f"\n[Crash Location (PC)] 0x{pc_val:08X} -> {sym}")

    if lr_match:
        lr_val = int(lr_match.group(1), 16)
        sym = resolve_address(addr2line, args.elf, lr_val)
        print(f"[Caller Return (LR)]  0x{lr_val:08X} -> {sym}")

    # Parse Stack Pointers & Memory Words
    print("\n[Call Stack Return Addresses in Flash]:")
    found_symbols = 0
    seen_addresses = set()
    hex_addresses = re.findall(r"0x([0-9a-fA-F]{8})", input_text)

    for addr_str in hex_addresses:
        addr = int(addr_str, 16)
        # Avoid duplicates and check if address falls within flash code range
        if addr in seen_addresses:
            continue
        seen_addresses.add(addr)

        # Skip registers already shown (ignoring Thumb bit 0)
        if pc_match and (addr & ~1) == (int(pc_match.group(1), 16) & ~1):
            continue
        if lr_match and (addr & ~1) == (int(lr_match.group(1), 16) & ~1):
            continue

        # nRF52 flash execution space (0x00000000 - 0x00100000)
        if 0x1000 <= addr <= 0x00100000:
            sym = resolve_address(addr2line, args.elf, addr)
            if sym and not sym.startswith("??") and "??:?" not in sym and "??:0" not in sym:
                print(f"  [Stack] 0x{addr:08X} -> {sym}")
                found_symbols += 1

    if found_symbols == 0:
        print("  (No additional flash code symbols found in top stack words)")

    print("=" * 70)

if __name__ == "__main__":
    main()
