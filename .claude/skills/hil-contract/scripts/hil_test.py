#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The bootloader's HIL battery on one rig board (.claude/skills/hil-contract/SKILL.md). The last stdout line is JSON.

  hil_test.py run -b BOARD --build-record REC --base-dir DIR [--profile P] [--run-root DIR] [--lock-wait S]
                [--fail-at STEP] ROSTER
  hil_test.py install -b BOARD --build-record REC --under-hold REASON ROSTER
  hil_test.py restore -b BOARD --build-record REC (--under-hold REASON | --recover-holder PID) ROSTER
  hil_test.py fat-read --dev DEV --file NAME [--offset N] [--length N]   (child: one bounded FAT read)
  hil_test.py flush --dev DEV                                             (child: fsync of the drive)

Exit: 0 pass (and cleaned up), 1 a step failed or the board did not run, 2 refused before any hardware access,
4 cleanup failed (run) or a command failed after touching the board (install, restore): a run's failure is logged in
the board's cleanup log and its lock released (SKILL.md Recovery). One board per process (SKILL.md Reservation).
"""
import argparse
import contextlib
import errno
import hashlib
import json
import os
import re
import secrets
import shutil
import socket
import stat
import struct
import subprocess
import sys
import time
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
import agentrc  # noqa: E402
import hil_run_report as run_report  # noqa: E402

REASON = 'hil_test.py'
PROFILE_STEPS = run_report.PROFILE_STEPS
REENTRY = ('gpregret',)
# nRF52 NVMC (PS v1.7 4.3.9) and POWER.GPREGRET (5.3.7.16); the same offsets on nRF52832/833
NVMC_READY, NVMC_CONFIG, NVMC_ERASEALL = 0x4001E400, 0x4001E504, 0x4001E50C
CONFIG_REN, CONFIG_EEN = 0, 2
GPREGRET = 0x4000051C
DFU_MAGIC_UF2_RESET, DFU_MAGIC_SERIAL_ONLY_RESET = 0x57, 0x4E   # src/main.c
ADAFRUIT_DEVICE_TYPE = 0x0052   # src/dfu_init.c: every init packet's device type
MBR_SIZE = 0x1000            # nrf_mbr.h
PAGE = 0x1000                # nRF52840 flash: 256 pages of 4 kB (PS v1.7 4.2.2); src/flash_nrf5x.c buffers one page
BOOTLOADER_SETTINGS = 0x000FF000   # dfu_types.h, nRF52840: bootloader_settings_t, bank_0 and bank_0_crc its first word
BANK_VALID_APP, BANK_INVALID_APP = 0x01, 0xFF   # bootloader_types.h
DHCSR, DHCSR_S_HALT = 0xE000EDF0, 1 << 17   # ARMv7-M ARM DDI 0403D C1.6.2: the core in Debug state
RESETREAS, RESETREAS_SREQ = 0x40000400, 1 << 2   # nRF52840 PS v1.7 5.3.7.11: cumulative, write 1 to clear
APP_FAMILY = {'nrf52840': 0xADA52840, 'nrf52833': 0x621E937A}   # src/usb/uf2/uf2cfg.h CFG_UF2_FAMILY_APP_ID
UF2_MAGIC = (0x0A324655, 0x9E5D5157, 0x0AB16F30)
UF2_FLAG_FAMILY = 0x2000
JLINK_SPEED = 4000
ERASEALL_WAIT_MS = 1000      # tERASEALL is at most 169 ms (PS v1.7 4.3.10); READY is read after it
PROBE_TIMEOUT, INSTALL_TIMEOUT, NRFUTIL_TIMEOUT = 60, 180, 120
# a dual-bank serial transfer cut short: about 240 KiB, which adafruit-nrfutil needs ~6 s of erase wait (dfu
# serial get_erase_wait_time) and ~16 s at the DK's ~15 KB/s to send; its bound ends it about a third in
CUT_PAGES, CUT_AFTER = 60, 12
USB_TIMEOUT, UDEV_TIMEOUT, CHILD_TIMEOUT, LEAVE_TIMEOUT, QUIET = 30, 10, 30, 15, 3
APP_SAMPLE_MS = 200
# nRESET pulses through the DK's interface MCU (J-Link r0/r1); src/main.c enters DFU on a pin reset within
# DFU_DBL_RESET_DELAY (500 ms) of the previous one
RESET_PULSE_MS, DBL_IN_WINDOW_MS, DBL_OUT_OF_WINDOW_MS = 20, 150, 1500
FLUSH_GONE = 3
SETTLE = 1.0                 # s after a tool exits for its descendants to be proven gone (hil_process.run_tracked)
SYS_DEV = '/sys/dev'          # {block,char}/MAJ:MIN: a device node's sysfs device
# the DUT-side tools a run or manual command starts, each with the options naming its drive or tty
DUT_TOOLS = {'mcopy': ('-i',), 'adafruit-nrfutil': ('-p', '--port'), 'hil_test.py': ('--dev',)}
UICR_NRFFW = (0x10001014, 0x1000101C)   # NRFFW[0] bootloader address, NRFFW[1] MBR parameter page
ARTIFACTS = {'elf', 'hex', 'hexMbr', 'map', 'merged', 'uf2Mbr', 'update', 'zip'}   # check_build.expected_artifacts
DEFAULT_CONFIG = {'CMAKE_BUILD_TYPE': 'MinSizeRel', 'DUALBANK_FW': 'OFF', 'DEFAULT_TO_OTA_DFU': 'OFF', 'SIGNED_FW': 'OFF',
                  'SIGNED_FW_QX': '', 'SIGNED_FW_QY': '', 'FORCE_UF2': 'OFF'}
UNCOVERED = ('nRF52832 (serial DFU, no USB) and nRF52833: no rig board', 'BLE OTA DFU',
             'another SoftDevice version than the build\'s',
             'copying a different SoftDevice or bootloader through serial DFU (the softdevice step sends the '
             'installed images, which the bootloader finds equal and does not copy)',
             'a SoftDevice or bootloader update on a dual-bank or SIGNED_FW build')


class Refused(Exception):
  pass


class StepFail(Exception):
  def __init__(self, msg, evidence=None):
    super().__init__(msg)
    self.evidence = evidence or {}


class NotEnded(StepFail):
  """A tool whose process tree is not proven ended: the board must not be touched again until recovery."""


# ------------------------------------------------------------------ agentrc

def provider():
  return agentrc.load_file('skills/hil/scripts/hil_flash.py')


def lock_helper():
  return agentrc.load_file('skills/hil/scripts/hil_lock.py')


def registry():
  return agentrc.load_file('skills/hil/scripts/hil_boards.py')


def report_core():
  return agentrc.load_file('skills/hil/scripts/hil_report_core.py')


def process():
  return agentrc.load_file('skills/hil/scripts/hil_process.py')


def usb():
  return agentrc.load_file('skills/hil/scripts/usb_observe.py')


# ------------------------------------------------------------------ roster, record, facts

def load_board(roster_path, name):
  """The roster entry `name`, resolved through agentrc's board registry: {entry, flasher, lock, device}."""
  try:
    roster = json.loads(Path(roster_path).read_text())
    entry = next((b for b in roster['boards'] if b.get('name') == name), None)
  except (OSError, ValueError, KeyError, TypeError) as e:
    raise Refused(f'{roster_path}: not a roster ({e})')
  if entry is None:
    raise Refused(f'{name} is not in {roster_path}')
  try:
    resolved = registry().resolve(entry)
    lock = registry().lock_name(entry)
  except (ValueError, ImportError) as e:
    raise Refused(f'{name}: {e}')
  flasher = resolved.get('flasher') or {}
  m = re.fullmatch(r'-device (\S+)', flasher.get('args') or '')
  if flasher.get('name') != 'jlink' or not m or not flasher.get('uid'):
    raise Refused(f'{name}: the battery needs a J-Link flasher with args "-device <name>" ({flasher})')
  extra = entry.get('nrf52') or {}
  if extra.get('reentry') not in (None, *REENTRY):
    raise Refused(f'{name}: nrf52.reentry {extra.get("reentry")!r} is none of {REENTRY}')
  base = extra.get('selfUpdateBase')
  if base is not None and (not isinstance(base, dict) or set(base) != {'version', 'file', 'sha256'}
                           or not all(isinstance(v, str) and v for v in base.values()) or '/' in base['file']):
    raise Refused(f'{name}: nrf52.selfUpdateBase is not {{version, file, sha256}} of a release asset ({base!r})')
  return {'name': name, 'entry': resolved, 'flasher': flasher, 'device': m[1], 'lock': lock,
          'uid': entry.get('uid'), 'reentry': extra.get('reentry'), 'base': base}


def admit_base(board, base_dir, workdir):
  """The roster's pinned release update UF2 (nrf52.selfUpdateBase), from `base_dir`, copied into `workdir` from bytes
  that hash as pinned."""
  base = board['base']
  if not base:
    raise Refused(f'{board["name"]} pins no nrf52.selfUpdateBase in the roster: commissioning.md')
  src, copy = Path(base_dir) / base['file'], Path(workdir) / base['file']
  copy_pinned(src, copy, base['sha256'], f'{src} is not the pinned {base["file"]} (sha256 {base["sha256"]}): '
                                         f'download it from release {base["version"]}')
  return {**base, 'path': str(copy)}


def copy_pinned(src, dst, digest, refusal):
  """`src`'s bytes into `dst` once they hash as `digest`, read once so what is hashed is what is written; Refused
  with `refusal` for anything else, a symlink included."""
  try:
    if not src.is_file() or src.is_symlink():
      raise Refused(refusal)
    data = src.read_bytes()
    if hashlib.sha256(data).hexdigest() != digest:
      raise Refused(refusal)
    dst.write_bytes(data)
  except OSError as e:
    raise Refused(f'{refusal} ({e})')


def build():
  """The build contract's check_build module: its git, sha256 and board helpers."""
  scripts = str(ROOT / '.claude/skills/build/scripts')
  if scripts not in sys.path:
    sys.path.insert(0, scripts)
  import check_build
  return check_build


def git(*args):
  try:
    return build().git(*args)
  except build().Refusal as e:
    raise Refused(str(e))


def admit(record_path, board, roster_path, workdir, profile='default'):
  """The build record's row for `board` in `profile` (check_build.hil_profiles), checked against this checkout, the
  roster and the agentrc loaded here, its artifacts copied into `workdir`/`profile` from bytes that hash as recorded.
  Refused otherwise."""
  try:
    rec = json.loads(Path(record_path).read_text())
  except (OSError, ValueError) as e:
    raise Refused(f'build record {record_path}: {e}')
  rows = [r for r in rec.get('boards', []) if r.get('board') == board and r.get('profile') == profile
          and r.get('status') == 'ok']
  if not rec.get('pass') or len(rows) != 1 or 'hilRecord' not in rec or 'sha256' not in rows[0]:
    raise Refused(f'{record_path}: not a passing HIL record of {board} in profile {profile} (check_build.py --board '
                  f'{board} --shared --hil-config {roster_path})')
  row, inp = rows[0], rec.get('input')
  if not isinstance(inp, dict):
    raise Refused(f'{record_path}: no input record')
  try:
    want, key = build().hil_profiles()[profile], build().SIGNED_FW_TEST_KEY
  except build().Refusal as e:
    raise Refused(str(e))
  bound = rec['hilRecord']
  if (bound.get('profiles') or {}).get(profile) != want or row.get('defines') != want or \
     (bound.get('signedFwKey') or {}).get('sha256') != build().sha256(key):
    raise Refused(f'{record_path}: profile {profile} is not built as this checkout defines it ({want}): rebuild')
  config = row.get('config') or {}
  expected = {**DEFAULT_CONFIG, 'BOARD': board, **dict(d.split('=', 1) for d in want)}
  wrong = {k: config.get(k) for k, v in expected.items() if config.get(k) != v}
  if rec.get('defines') or wrong:
    raise Refused(f'{record_path}: not profile {profile}\'s configuration (defines {rec.get("defines")}, {wrong})')
  if set(row.get('artifacts') or {}) != ARTIFACTS or set(row.get('sha256') or {}) != ARTIFACTS:
    raise Refused(f'{record_path}: artifacts {sorted(row.get("artifacts") or {})} are not {sorted(ARTIFACTS)}')
  head, dirty = git('rev-parse', 'HEAD').strip(), bool(git('status', '--porcelain').strip())
  if inp.get('dirty') or inp.get('head') != head or dirty:
    raise Refused(f'{record_path} was built from {inp.get("head")} (dirty {inp.get("dirty")}); this checkout is '
                  f'{head} (dirty {dirty}): rebuild on a clean HEAD')
  subs = {}
  for line in git('submodule', 'status').splitlines():
    sha, path = line[1:].split()[:2]
    subs[path] = sha
  if {p: s['commit'] for p, s in inp.get('submodules', {}).items()} != subs:
    raise Refused(f'{record_path}: submodules differ from this checkout ({subs})')
  if rec['hilRecord']['roster']['sha256'] != build().sha256(roster_path):
    raise Refused(f'{record_path} binds another roster than {roster_path}: rebuild')
  try:
    here = agentrc.identity()
  except ImportError as e:
    raise Refused(f'agentrc: {e}')
  bound = rec['hilRecord'].get('agentrc') or {}
  if (bound.get('commit'), bound.get('sha256')) != (here.get('commit'), here.get('sha256')):
    raise Refused(f'{record_path} binds agentrc {bound.get("commit")}, this run loads {here.get("commit")}: rebuild')
  copies, into = {}, Path(workdir) / profile
  into.mkdir(exist_ok=True)
  for name, rel in row['artifacts'].items():
    src = ROOT / rel
    copies[name] = into / src.name
    copy_pinned(src, copies[name], row['sha256'][name], f'{rel} does not hash as {record_path} records: rebuild')
  return rec, row, copies


def parse_hex(path):
  """{address: byte} of an Intel HEX file (record types 00, 01, 02, 04, 05)."""
  data, base = {}, 0
  for n, line in enumerate(Path(path).read_text().splitlines(), 1):
    if not line.startswith(':'):
      continue
    raw = bytes.fromhex(line[1:])
    if sum(raw) & 0xFF or len(raw) != raw[0] + 5:
      raise Refused(f'{path}:{n}: bad record')
    addr, kind, body = int.from_bytes(raw[1:3], 'big'), raw[3], raw[4:4 + raw[0]]
    if kind == 0:
      for i, b in enumerate(body):
        data[base + addr + i] = b
    elif kind == 2:
      base = int.from_bytes(body, 'big') << 4
    elif kind == 4:
      base = int.from_bytes(body, 'big') << 16
    elif kind not in (1, 3, 5):
      raise Refused(f'{path}:{n}: record type {kind}')
  return data


def runs(data):
  """[(start, bytes)] of the contiguous runs of a parse_hex dict."""
  out, start, buf, prev = [], None, bytearray(), None
  for a in sorted(data):
    if prev is not None and a != prev + 1:
      out.append((start, bytes(buf)))
      start, buf = None, bytearray()
    if start is None:
      start = a
    buf.append(data[a])
    prev = a
  if start is not None:
    out.append((start, bytes(buf)))
  return out


def check_spans(data, flash_end, app_start, boot_start, name):
  """Refused unless the merged image writes only flash below `flash_end` and UICR NRFFW, holds MBR, SoftDevice and
  bootloader, leaves the app region [app_start, boot_start) empty, and points NRFFW[0] at the bootloader."""
  stray = sorted(a for a in data if not (a < flash_end or UICR_NRFFW[0] <= a < UICR_NRFFW[1]))
  if stray:
    raise Refused(f'{name} writes outside flash and UICR NRFFW, from {stray[0]:#x}')
  in_app = [a for a in data if app_start <= a < boot_start]
  if in_app:
    raise Refused(f'{name} writes the app region at {min(in_app):#x}')
  starts = {a for a, _ in runs(data)}
  nrffw0 = bytes(data.get(UICR_NRFFW[0] + i, 0xFF) for i in range(4))
  if not {0, MBR_SIZE, boot_start} <= starts or int.from_bytes(nrffw0, 'little') != boot_start:
    raise Refused(f'{name}: want runs at 0 (MBR), {MBR_SIZE:#x} (SoftDevice) and {boot_start:#x} (bootloader) and '
                  f'NRFFW[0] = {boot_start:#x}; have runs at {sorted(map(hex, starts))}, NRFFW[0] {nrffw0.hex()}')


def define(text, name):
  m = re.search(rf'^\s*#define\s+{name}\s+(.+?)\s*(//.*)?$', text, re.M)
  return m[1] if m else None


def facts(board, row, copies, workdir):
  """What the battery checks the board against, all from the record's build and this checkout's sources."""
  mcu, sd = build().board_config(board)
  if mcu not in APP_FAMILY:
    raise Refused(f'{board}: {mcu} has no USB, so no UF2 drive: the battery does not apply')
  if row['softdevice']['name'] + '_nrf52_' + row['softdevice']['version'] != sd:
    raise Refused(f'{board}: record SoftDevice {row["softdevice"]} is not the board\'s {sd}')
  sdm = (ROOT / f'lib/softdevice/{sd}/{sd}_API/include/nrf_sdm.h').read_text()
  sd_size = define(sdm, 'SD_FLASH_SIZE')
  ld = (ROOT / f'lib/nrfx/mdk/{mcu}_xxaa.ld').read_text()
  ram = re.search(r'RAM\s*\(rwx\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+),\s*LENGTH\s*=\s*(0x[0-9A-Fa-f]+)', ld)
  bh = (ROOT / f'src/boards/{board}/board.h').read_text()
  vid, pid, board_id = define(bh, 'USB_DESC_VID'), define(bh, 'USB_DESC_UF2_PID'), define(bh, 'UF2_BOARD_ID')
  cdc_pid = define(bh, 'USB_DESC_CDC_ONLY_PID')
  if not (sd_size and ram and vid and pid and cdc_pid and board_id):
    raise Refused(f'{board}: cannot establish SD_FLASH_SIZE, RAM, USB_DESC_VID/UF2_PID/CDC_ONLY_PID or UF2_BOARD_ID')
  merged = parse_hex(copies['merged'])
  flash = re.search(r'FLASH\s*\(rx\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+),\s*LENGTH\s*=\s*(0x[0-9A-Fa-f]+)', ld)
  boot = re.search(r'FLASH\s*\(rx\)\s*:\s*ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)', (ROOT / f'linker/{mcu}.ld').read_text())
  if not flash or not boot:
    raise Refused(f'{mcu}: cannot establish the flash size or the bootloader start')
  check_spans(merged, int(flash[2], 16), MBR_SIZE + int(sd_size, 0), int(boot[1], 16), copies['merged'].name)
  bank1 = bank_1_start(mcu, MBR_SIZE + int(sd_size, 0), int(boot[1], 16))
  hex_runs = runs(merged)
  versions = {v for _, data in hex_runs for v in re.findall(rb'UF2 Bootloader ([^\r\n\x00]+)\r\n', data)}
  drive = has_drive(row)
  if len(versions) != int(drive):
    raise Refused(f'{copies["merged"].name}: expected {int(drive)} "UF2 Bootloader <version>" string, found '
                  f'{len(versions)}')
  for a, b in hex_runs:
    (Path(workdir) / f'run-{a:08x}.bin').write_bytes(b)
  return {'mcu': mcu, 'family': APP_FAMILY[mcu], 'appStart': MBR_SIZE + int(sd_size, 0),
          'bank1': bank1, 'ramEnd': int(ram[1], 16) + int(ram[2], 16), 'vid': int(vid, 0), 'pid': int(pid, 0),
          'cdcPid': int(cdc_pid, 0),
          'boardId': board_id.strip('"'), 'version': versions.pop().decode() if drive else None, 'drive': drive,
          'softdevice': f'S{row["softdevice"]["name"][1:]} {row["softdevice"]["version"]}',
          'hex': str(copies['merged']), 'update': str(copies['update']), 'zip': str(copies['zip']),
          'runs': [(a, str(Path(workdir) / f'run-{a:08x}.bin')) for a, _ in hex_runs]}


def has_drive(row):
  """Whether the record row's build has the UF2 drive: SIGNED_FW drops it unless FORCE_UF2 (src/usb/tusb_config.h)."""
  defines = row.get('defines') or []
  return 'SIGNED_FW=ON' not in defines or 'FORCE_UF2=ON' in defines


def bank_1_start(mcu, app_start, boot_start):
  """DFU_BANK_1_REGION_START (dfu_types.h): each bank half the app region less the MCU's DFU_APP_DATA_RESERVED
  (CMakeLists.txt), rounded down to two pages, bank 0 at the app start."""
  m = re.search(rf'MCU_VARIANT STREQUAL "{mcu}"\)(?:(?!elseif|endif).)*?set\(DFU_APP_DATA_RESERVED (\d+)\*4096\)',
                (ROOT / 'CMakeLists.txt').read_text(), re.S)
  if not m:
    raise Refused(f'{mcu}: cannot establish DFU_APP_DATA_RESERVED')
  full = boot_start - app_start - int(m[1]) * 4096
  return app_start + (full - full % (2 * PAGE)) // 2


def serial_argv(pkg, tty, singlebank):
  """adafruit-nrfutil's arguments sending genpkg package `pkg` over CDC `tty`; `singlebank` for a single-bank build,
  where it skips the bank 1 to bank 0 copy's wait."""
  return ['--verbose', 'dfu', 'serial', '--package', pkg, '-p', tty, '-b', '115200',
          *(['--singlebank'] if singlebank else [])]


def repackage_image(pkg):
  """The application image a genpkg package `pkg` carries."""
  with zipfile.ZipFile(pkg) as z:
    app = json.loads(z.read('manifest.json'))['manifest']['application']
    return z.read(app['bin_file'])


def repackage(src, dst, edit):
  """genpkg package `src` written again to `dst`, each init packet and image replaced by edit(dat, image)."""
  with zipfile.ZipFile(src) as z:
    files = {n: z.read(n) for n in z.namelist()}
  for part in json.loads(files['manifest.json'])['manifest'].values():
    if isinstance(part, dict) and 'dat_file' in part:
      files[part['dat_file']], files[part['bin_file']] = edit(files[part['dat_file']], files[part['bin_file']])
  with zipfile.ZipFile(dst, 'w') as z:
    for n, data in files.items():
      z.writestr(n, data)
  return dst


def flip_crc(dat, image):
  """An unsigned init packet whose trailing CRC-16 (dfu_init.c postvalidate) no longer matches `image`."""
  return dat[:-2] + bytes(b ^ 0xFF for b in dat[-2:]), image


# ------------------------------------------------------------------ the fixture app

def fixture(f, nonce, pattern=0x1000):
  """An app the probe can prove ran, and which one: SP and reset vectors, every other vector a fault loop, a reset
  handler that loads a word of `nonce` into r1 and counts in r0 forever, then a `pattern`-byte pattern from `nonce`
  (a multiple of 32), so an earlier copy's image reads back different."""
  start, table = f['appStart'], 64
  handler = start + table
  loop, fault = handler + 2, handler + 6
  marker = int.from_bytes(hashlib.sha256(b'marker' + nonce).digest()[:4], 'little')
  vectors = [f['ramEnd'], handler | 1] + [fault | 1] * (table // 4 - 2)
  # ldr r1, [pc, #4] (the marker) ; loop: adds r0, #1 ; b loop ; fault: b . ; marker
  code = struct.pack('<HHHHI', 0x4901, 0x3001, 0xE7FD, 0xE7FE, marker)
  pattern = b''.join(hashlib.sha256(nonce + i.to_bytes(4, 'little')).digest() for i in range(pattern // 32))
  img = struct.pack(f'<{len(vectors)}I', *vectors) + code + pattern
  img += b'\xff' * (-len(img) % 256)
  return {'image': img, 'handler': handler, 'loop': loop, 'fault': fault, 'marker': marker, 'start': start}


def shown(fx, nonce):
  """A step's evidence naming fixture `fx`: its span and the nonce it came from."""
  return {'image': f'{fx["start"]:#010x}+{len(fx["image"])}', 'nonce': nonce.hex()}


def make_uf2(img, base, family):
  n = len(img) // 256
  out = bytearray()
  for i in range(n):
    chunk = img[i * 256:(i + 1) * 256]
    out += struct.pack('<8I', UF2_MAGIC[0], UF2_MAGIC[1], UF2_FLAG_FAMILY, base + 256 * i, 256, i, n, family)
    out += chunk + b'\0' * (476 - 256) + struct.pack('<I', UF2_MAGIC[2])
  return bytes(out)


def uf2_payload(blob, base, length):
  """The bytes `blob` (a UF2 file read from the drive) carries for [base, base + length)."""
  got = {}
  for off in range(0, len(blob) - 511, 512):
    m0, m1, _, addr, size, _, _, _ = struct.unpack_from('<8I', blob, off)
    if (m0, m1) == UF2_MAGIC[:2] and base <= addr < base + length:
      got[addr] = blob[off + 32:off + 32 + size]
  return b''.join(got[a] for a in sorted(got))


# ------------------------------------------------------------------ signed packages (SIGNED_FW profile)

# NIST P-256 (FIPS 186-4 D.1.2.3), the curve src/dfu_init.c verifies with (tinycrypt uECC_secp256r1)
P256_P = 0xFFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF
P256_N = 0xFFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551
P256_G = (0x6B17D1F2E12C4247F8BCE6E563A440F277037D812DEB33A0F4A13945D898C296,
          0x4FE342E2FE1A7F9B8EE7EB4A7C0F9E162BCE33576B315ECECBB6406837BF51F5)
DFU_INIT_USES_ECDS = 2   # src/dfu_init.c DFU_INIT_PACKET_USES_ECDS


def ec_add(a, b):
  """a + b on P-256 in affine coordinates, None the point at infinity."""
  if a is None or b is None:
    return a or b
  if a[0] == b[0] and (a[1] + b[1]) % P256_P == 0:
    return None
  if a == b:
    m = 3 * (a[0] * a[0] - 1) * pow(2 * a[1], -1, P256_P)   # curve a = -3
  else:
    m = (b[1] - a[1]) * pow(b[0] - a[0], -1, P256_P)
  x = (m * m - a[0] - b[0]) % P256_P
  return x, (m * (a[0] - x) - a[1]) % P256_P


def ec_mul(k, point=P256_G):
  out = None
  while k:
    if k & 1:
      out = ec_add(out, point)
    point, k = ec_add(point, point), k >> 1
  return out


def public_key(d):
  """The 64-byte X || Y (big endian) of private key `d`, as SIGNED_FW_QX/QY take it."""
  x, y = ec_mul(d)
  return x.to_bytes(32, 'big') + y.to_bytes(32, 'big')


def ecdsa_sign(d, message):
  """ECDSA P-256 with SHA-256 over `message`: r || s, 32 bytes each big endian, as uECC_verify takes it."""
  z = int.from_bytes(hashlib.sha256(message).digest(), 'big')
  while True:
    k = secrets.randbelow(P256_N - 1) + 1
    r = ec_mul(k)[0] % P256_N
    s = pow(k, -1, P256_N) * (z + r * d) % P256_N
    if r and s:
      return r.to_bytes(32, 'big') + s.to_bytes(32, 'big')


def signed_init_packet(dat, image, d):
  """An init packet `src/dfu_init.c` verifies under SIGNED_FW from adafruit-nrfutil's CRC16 one (`dat`): its fields
  without the CRC, then the ECDS identifier, the image length and SHA-256, signed with `d` over all of that."""
  signed = dat[:-2] + struct.pack('<II', DFU_INIT_USES_ECDS, len(image)) + hashlib.sha256(image).digest()
  return signed + ecdsa_sign(d, signed)


def signed_by(d):
  """A repackage edit signing each init packet with private key `d`."""
  return lambda dat, image: (signed_init_packet(dat, image, d), image)


def tampered_after_signing(d):
  """A repackage edit signing each image with `d`, then flipping the image's last byte."""
  return lambda dat, image: (signed_init_packet(dat, image, d), image[:-1] + bytes([image[-1] ^ 0xFF]))


def signing_key():
  """The private key of the signed profile's test key (check_build.signed_fw_test_key), checked against its public
  key, which the build compiles in."""
  try:
    key = build().signed_fw_test_key()
  except build().Refusal as e:
    raise Refused(str(e)) from None
  d = int.from_bytes(key['d'], 'big')
  if public_key(d) != key['qx'] + key['qy']:
    raise Refused(f'{build().SIGNED_FW_TEST_KEY.name}: d does not give qx, qy')
  return d


# ------------------------------------------------------------------ processes

class Tools:
  """Every child runs under hil_process.run_tracked, which proves its process tree ended (`cleanup` 'done') or names
  a receipt; one that is not proven ended stops the battery (NotEnded) and its receipt goes to the board's cleanup log.
  run_tracked keeps a child's files in `dir` until it is proven ended, for resolve_restored to recheck its receipt."""
  def __init__(self, workdir, log):
    self.dir, self.log, self.receipts = process().state_dir(str(workdir), 'ops'), log, []

  def run(self, cmd, cwd=None, *, env=None, timeout):
    """hil_flash's runner contract: a CompletedProcess carrying `cleanup` and `receipt`."""
    if env:
      raise ValueError('no tool here takes an environment')
    argv = [str(a) for a in cmd]
    try:
      r = process().run_tracked(argv, cwd=cwd, timeout=timeout, settle=SETTLE, state_dir=self.dir, quiet=True)
    except BaseException as e:
      self.note(argv, getattr(e, 'cleanup', 'unknown'), getattr(e, 'receipt', None))
      raise
    self.note(argv, r.cleanup, r.receipt)   # before anything else can fail
    with open(self.log, 'a') as fh:
      fh.write(f'$ {" ".join(argv)}\n{process().cmd_stdout_text(r.stdout)}{process().cmd_stdout_text(r.stderr or "")}'
               f'\n# exit {r.returncode}, cleanup {r.cleanup}\n')
    return r

  def note(self, argv, cleanup, receipt):
    if cleanup != 'done' or receipt is not None:
      self.receipts.append({'cmd': argv[0], 'cleanup': cleanup, 'receipt': receipt})

  def check(self, what):
    if self.receipts:
      raise NotEnded(f'{what}: a tool is not proven ended', {'receipts': self.receipts})

  def logged(self):
    """What the board's cleanup log keeps of each tool not proven ended: its receipt, or for one run_tracked named
    none, an entry nothing rechecks done, so its failure stays open."""
    return [r['receipt'] if r['receipt'] is not None else {'unrecheckable': f'{r["cmd"]}: cleanup {r["cleanup"]}'}
            for r in self.receipts]

  def spawn(self, argv, timeout):
    """(exit, output) of a bounded child; exit 124 on the bound."""
    r = self.run(argv, timeout=timeout)
    self.check(Path(str(argv[0])).name)
    return r.returncode, process().cmd_stdout_text(r.stdout) + process().cmd_stdout_text(r.stderr or '')

  def child_json(self, argv, timeout, what):
    code, out = self.spawn([sys.executable, '-B', __file__, *argv], timeout)
    if code == 124:
      raise StepFail(f'{what}: timed out after {timeout}s')
    try:
      return code, json.loads(out.strip().splitlines()[-1])
    except (IndexError, ValueError):
      return code, None


# ------------------------------------------------------------------ probe

def verify_runs(f):
  """(J-Link verifybin commands, evidence) for every run of the merged HEX `f` describes."""
  return ([f'verifybin {p} {a:08X}' for a, p in f['runs']],
          [f'{a:#010x}+{Path(p).stat().st_size}' for a, p in f['runs']])


class Probe:
  """J-Link sessions through agentrc's provider, each command judged from its own output."""
  def __init__(self, board, workdir, tools):
    self.b, self.tools = board, tools
    self.profile = {'interface': 'swd', 'speed': JLINK_SPEED, 'autoconnect': True, 'device': board['device'],
                    'workdir': str(workdir)}

  def session(self, commands, timeout=PROBE_TIMEOUT, judge=True, connect=True):
    def crashed(a):   # the OB's libjlinkarm crash before any target access
      out = process().cmd_stdout_text(a.result.stdout)
      return a.result.returncode in provider().JLINK_CRASH_RCS and 'Found Cortex' not in out
    op = provider().run_jlink_commands(self.b['flasher'], list(commands), **{**self.profile, 'autoconnect': connect},
                                       timeout=timeout, run=self.tools.run, retry_if=crashed, retry_delay=2)
    self.tools.check('JLinkExe')
    out, last = process().cmd_stdout_text(op.stdout), op.attempts[-1]
    tail = {'tail': out.strip().splitlines()[-6:]}
    if last.timed_out:
      raise StepFail(f'JLinkExe timed out after {timeout}s', tail)
    if judge and (problem := provider().jlink_problem(op)):
      raise StepFail(f'J-Link session failed: {problem}', tail)
    return out

  def install(self, f):
    """Erase all (NVMC, CPU halted: nRF52840 rev3 errata [184]), load the merged HEX, then verify every run of it.
    Reset and halt first: the bootloader write-protects its region through the ACL before it starts an app, which
    blocks ERASEALL, and only a reset clears the ACL (PS v1.7 6.3, 5.3.6.8)."""
    out = self.session(['r', f'w4 {NVMC_CONFIG:08X} {CONFIG_EEN:08X}', f'w4 {NVMC_ERASEALL:08X} 00000001',
                        f'sleep {ERASEALL_WAIT_MS}', f'mem32 {NVMC_READY:08X} 1', f'w4 {NVMC_CONFIG:08X} {CONFIG_REN:08X}',
                        f'loadfile {f["hex"]}', 'h', *verify_runs(f)[0], 'r', 'g'],
                       timeout=INSTALL_TIMEOUT)
    ready = word_at(out, NVMC_READY)
    if ready is None or ready & 1 != 1:
      raise StepFail(f'NVMC not ready after ERASEALL ({"no read" if ready is None else f"{ready:08X}"})')
    return {'erased': True, 'verified': verify_runs(f)[1]}

  def samples(self):
    """Two register samples a commanded APP_SAMPLE_MS apart, the CPU running between them, in one session."""
    out = self.session(['h', 'regs', 'g', f'sleep {APP_SAMPLE_MS}', 'h', 'regs', 'g'])
    chunks = provider().jlink_after(out, 'regs')
    if len(chunks) != 2:
      raise StepFail(f'{len(chunks)} J-Link regs outputs, not 2')

    def one(chunk, pat):
      m = re.search(pat, chunk)
      if not m:
        raise StepFail(f'no {pat!r} in J-Link regs output')
      return int(m[1], 16)
    return [{'pc': one(c, r'\bPC = ([0-9A-F]{8})'), 'r0': one(c, r'\bR0 = ([0-9A-F]{8})'),
             'r1': one(c, r'\bR1 = ([0-9A-F]{8})'), 'ipsr': one(c, r'\bIPSR = ([0-9A-F]{3})')} for c in chunks]

  def reenter(self, magic=DFU_MAGIC_UF2_RESET):
    """GPREGRET = `magic`, read back, then SYSRESETREQ: a soft reset keeps the retained register (PS v1.7 5.3.6.8), and
    the bootloader enters UF2 DFU (0x57) or CDC-only serial DFU (0x4E) on it (src/main.c)."""
    value = word_at(self.session(['h', f'w4 {GPREGRET:08X} {magic:08X}', f'mem32 {GPREGRET:08X} 1', 'g']), GPREGRET)
    if value is None or value & 0xFF != magic:
      raise StepFail(f'GPREGRET reads back {"nothing" if value is None else f"{value:08X}"}')
    op = provider().sysresetreq_jlink(self.b['flasher'], **self.profile, timeout=PROBE_TIMEOUT,
                                      run=self.tools.run)
    self.tools.check('JLinkExe')
    # the reset may drop the connection mid-write, so the write is judged from its echo, not jlink_problem
    issued = f'Writing {provider().AIRCR_SYSRESETREQ:08X} -> {provider().SCB_AIRCR:08X}'
    if op.attempts[-1].timed_out or issued not in process().cmd_stdout_text(op.stdout):
      raise StepFail('the SYSRESETREQ write was not issued')
    return {'gpregret': f'{value:08X}'}

  def pin_resets(self, gaps):
    """nRESET pulses `gaps` ms apart, each RESET_PULSE_MS low, from J-Link r0/r1 in a session that never connects to
    the target: on the DK the interface MCU drives the nRF52840's nRESET (nRF52840 DK guide, book 1602 pp14, 25;
    qualified on the rig: RESETREAS.RESETPIN set). The pulses are judged by their echoes, which no completion rule
    covers."""
    pulse = ['r0', f'sleep {RESET_PULSE_MS}', 'r1']
    commands = [*pulse, *(c for g in gaps for c in (f'sleep {g}', *pulse))]
    out = self.session(commands, judge=False, connect=False)
    echoed = provider().jlink_echoed(out)
    failed = [c for c, chunk in echoed if re.search(provider().JLINK_ERRORS, chunk)]
    if [c for c, _ in echoed] != [*commands, 'exit'] or failed:
      raise StepFail(f'the nRESET pulses were not all issued (failed: {failed})',
                     {'tail': out.strip().splitlines()[-8:]})
    return {'pulses': len(gaps) + 1, 'gapsMs': list(gaps)}


def word_at(out, addr):
  """The word a J-Link `mem32 <addr> 1` printed in `out`, None when it printed none."""
  words = words_at(out, addr, 1)
  return words[0] if words else None


def words_at(out, addr, n):
  """The `n` words a J-Link `mem32 <addr> <n>` printed in `out` (n <= 4, one line), None when it printed fewer."""
  m = re.search(rf'^{addr:08X} = ((?:[0-9A-F]{{8}} ?){{{n}}})', out, re.M)
  return [int(w, 16) for w in m[1].split()] if m else None


# ------------------------------------------------------------------ USB drive

def usb_device(vid, pid, serial):
  """The one enumerated USB device vid:pid whose serial is exactly `serial`, as agentrc usb_observe.usb_scan gives it
  ({busport, ino, ...}; ino, the kernfs enumeration_id, changes on every re-enumeration), None when there is none.
  A device whose serial cannot be read is not returned (usb_absent counts it as maybe present)."""
  hits = [d for d in usb().usb_scan(vid_pid=(f'{vid:04x}', f'{pid:04x}'), serial=serial) if d['serial'] == serial]
  if len(hits) > 1:
    raise StepFail(f'{len(hits)} USB devices {vid:04x}:{pid:04x} with serial {serial}: '
                   f'{[d["busport"] for d in hits]}')
  return hits[0] if hits else None


def cdc_tty(usb_path):
  ttys = sorted(p.name for p in Path('/sys/bus/usb/devices').glob(f'{usb_path}:*/tty/*'))
  if len(ttys) > 1:
    raise StepFail(f'{usb_path} has {len(ttys)} ttys: {ttys}')
  return f'/dev/{ttys[0]}' if ttys else None


def block_device(usb_path):
  for blk in Path('/sys/block').iterdir():
    if f'/{usb_path}:' in os.path.realpath(blk):
      return f'/dev/{blk.name}'
  return None


def wait_for(fn, timeout, what):
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    v = fn()
    if v:
      return v
    time.sleep(0.2)
  raise StepFail(f'{what} not seen within {timeout}s')


def stays_away(fn, seconds, what):
  end = time.monotonic() + seconds
  while True:
    if fn():
      raise StepFail(what)
    if time.monotonic() >= end:
      return
    time.sleep(0.2)


def automounter():
  return any(Path(p).exists() for p in ('/usr/libexec/udisks2/udisksd', '/usr/lib/udisks2/udisksd'))


def exclusive(dev):
  if automounter():
    props = subprocess.run(['udevadm', 'info', '--query=property', f'--name={dev}'], capture_output=True, text=True)
    if 'UDISKS_IGNORE=1' not in props.stdout.split():
      raise StepFail(f'{dev} has no UDISKS_IGNORE=1 and udisks2 is installed: automount could race the raw I/O '
                     '(commissioning.md)')
  if any(os.path.realpath(l.split()[0]) == dev for l in Path('/proc/mounts').read_text().splitlines()):
    raise StepFail(f'{dev} is mounted; refusing raw I/O')


class Drive:
  def __init__(self, board, f, tools):
    self.b, self.f, self.tools = board, f, tools

  def present(self):
    usb = usb_device(self.f['vid'], self.f['pid'], self.b['uid'])
    return block_device(usb['busport']) if usb else None

  def absent(self):
    """No bootloader device with the board's serial enumerated: one whose serial cannot be read may be it."""
    return usb().usb_absent(vid_pid=(f'{self.f["vid"]:04x}', f'{self.f["pid"]:04x}'), serial=self.b['uid'])

  def wait(self):
    dev = wait_for(self.present, USB_TIMEOUT, f'UF2 drive {self.f["vid"]:04x}:{self.f["pid"]:04x} {self.b["uid"]}')
    # the node appears before udev's rule gives it the rig's group
    wait_for(lambda: os.access(dev, os.R_OK | os.W_OK), UDEV_TIMEOUT, f'read-write access to {dev}')
    exclusive(dev)
    return dev

  def read(self, dev, name, offset=0, length=None):
    argv = ['fat-read', '--dev', dev, '--file', name, '--offset', str(offset)]
    code, res = self.tools.child_json(argv + (['--length', str(length)] if length else []), CHILD_TIMEOUT, name)
    if code or not res or not res.get('ok'):
      raise StepFail(f'reading {name} failed', {'result': res, 'exit': code})
    return bytes.fromhex(res['data']), res

  def info(self, dev, version=None):
    text = self.read(dev, 'INFO_UF2.TXT')[0].decode(errors='replace')
    want = {'UF2 Bootloader': version or self.f['version'], 'Board-ID:': self.f['boardId'],
            'SoftDevice:': self.f['softdevice']}
    lines = text.splitlines()
    bad = {k: v for k, v in want.items() if not any(l.strip() == f'{k} {v}' for l in lines)}
    if bad:
      raise StepFail(f'INFO_UF2.TXT does not name {bad}', {'info': lines})
    return lines

  def copy(self, dev, uf2, workdir, leaves=True):
    """`uf2` copied to the drive and flushed; the drive then leaves, or with `leaves` False (a transfer the
    bootloader cannot complete) stays."""
    path = Path(workdir) / 'copy.uf2'
    path.write_bytes(uf2)
    code, out = self.tools.spawn(['mcopy', '-o', '-i', dev, path, '::APP.UF2'], 60)
    if code:
      raise StepFail('mcopy failed', {'tail': out.strip().splitlines()[-5:]})
    code, res = self.tools.child_json(['flush', '--dev', dev], CHILD_TIMEOUT, 'flush')
    if code not in (0, FLUSH_GONE):
      raise StepFail('flush failed', {'result': res})
    if not leaves:
      stays_away(lambda: not self.present(), QUIET, 'the drive left after a copy it cannot complete')
      return {'flush': 'flushed' if code == 0 else 'device gone before flush', 'stayed': QUIET}
    wait_for(self.absent, LEAVE_TIMEOUT, 'the drive leaving after the copy')
    return {'flush': 'flushed' if code == 0 else 'device gone before flush'}


# ------------------------------------------------------------------ the battery

class Battery:
  """One profile's battery (hil_run_report.PROFILE_STEPS) on the image `f` describes; cleanup restores `home`, the
  default profile's image, `f` itself when that is the default."""
  def __init__(self, board, f, workdir, log, fail_at=None, profile='default', home=None):
    self.b, self.f, self.dir, self.fail_at, self.profile = board, f, Path(workdir), fail_at, profile
    self.home = home or f
    self.tools = Tools(workdir, log)
    self.probe, self.drive = Probe(board, workdir, self.tools), Drive(board, f, self.tools)
    self.steps, self.touched = [], False

  def step(self, name, fn):
    t = time.monotonic()
    try:
      if name == self.fail_at:
        raise StepFail(f'--fail-at {name}')
      ev = fn() or {}
      self.steps.append({'step': name, 'pass': True, 'evidence': ev, 'seconds': round(time.monotonic() - t, 1)})
      return ev
    except StepFail as e:
      self.steps.append({'step': name, 'pass': False, 'detail': str(e), 'evidence': e.evidence,
                         'seconds': round(time.monotonic() - t, 1)})
      raise
    except Exception as e:   # unexpected: the step failed, and cleanup decides what the board is left in
      self.steps.append({'step': name, 'pass': False, 'detail': f'{type(e).__name__}: {e}',
                         'seconds': round(time.monotonic() - t, 1)})
      raise StepFail(f'{type(e).__name__}: {e}') from e

  def install(self):
    self.touched = True
    return self.probe.install(self.f)

  def dfu(self):
    dev = self.drive.wait()
    return {'dev': dev, 'info': self.drive.info(dev)}

  def copy(self, nonce):
    fx = fixture(self.f, nonce)
    dev = self.drive.wait()
    ev = self.drive.copy(dev, make_uf2(fx['image'], fx['start'], self.f['family']), self.dir)
    return {**ev, **shown(fx, nonce)}

  def app_ran(self, nonce):
    """The app the copy wrote runs after the bootloader's own reset: no probe reset first. The drive stays away, then
    two samples: thread mode, PC in the counting loop, r1 this copy's marker, r0 advancing."""
    fx = fixture(self.f, nonce)
    stays_away(lambda: not self.drive.absent(), QUIET, 'the UF2 drive came back instead of the app')
    first, second = self.probe.samples()
    ev = {'samples': [{k: hex(v) for k, v in s.items()} for s in (first, second)]}
    loop = range(fx['loop'], fx['fault'])
    for s in (first, second):
      if s['ipsr'] or s['pc'] not in loop or s['r1'] != fx['marker']:
        raise StepFail('this copy\'s app is not counting in thread mode',
                       {**ev, 'loop': hex(fx['loop']), 'marker': hex(fx['marker'])})
    if (second['r0'] - first['r0']) & 0xFFFFFFFF == 0:
      raise StepFail('r0 did not advance', ev)
    return ev

  def double_reset(self, nonce):
    """src/main.c's double reset on the reset pin (SKILL.md Execution, double-reset): one pulse and two
    DBL_OUT_OF_WINDOW_MS apart leave app `nonce` running, two DBL_IN_WINDOW_MS apart bring the drive."""
    ev = {}
    for name, gaps in (('single', []), ('outOfWindow', [DBL_OUT_OF_WINDOW_MS])):
      ev[name] = {**self.probe.pin_resets(gaps), 'appRan': self.app_ran(nonce)}
    ev['inWindow'] = self.probe.pin_resets([DBL_IN_WINDOW_MS])
    ev['inWindow']['info'] = self.drive.info(self.drive.wait())
    return ev

  def reenter(self, nonce):
    fx = fixture(self.f, nonce)
    ev = self.probe.reenter()
    dev = self.drive.wait()
    blob, res = self.drive.read(dev, 'CURRENT.UF2', *self.current_span(len(fx['image'])))
    got = uf2_payload(blob, fx['start'], len(fx['image']))
    if got != fx['image']:
      raise StepFail('CURRENT.UF2 does not read back the copied app', {'read': len(got)})
    return {**ev, 'method': 'gpregret', 'readBack': len(got)}

  def serial(self, nonce, singlebank=True, edit=None):
    """App `nonce` through CDC-only serial DFU (SKILL.md Execution, serial), its package rewritten by `edit`."""
    fx = fixture(self.f, nonce)
    ev, tty = self.enter_cdc()
    self.dfu_serial(self.package(fx['image'], f'serial-{nonce.hex()[:8]}', edit), tty, singlebank)
    return {**ev, 'tty': tty, **shown(fx, nonce)}

  def package(self, image, name, edit=None):
    """An adafruit-nrfutil genpkg app package of `image`, rewritten by repackage's edit(dat, image) when given."""
    app, pkg = self.dir / f'{name}.bin', self.dir / f'{name}.zip'
    app.write_bytes(image)
    self.nrfutil(['dfu', 'genpkg', '--dev-type', f'{ADAFRUIT_DEVICE_TYPE:#06x}', '--application', app, pkg], 'genpkg')
    return repackage(pkg, pkg, edit) if edit else pkg

  def cdc_only(self, pid=None, stale=()):
    """The tty of the bootloader in CDC-only serial DFU (`pid` default its CDC PID) and the roster serial, one tty, no
    drive. The enumeration still up from before the reset can carry the same PID (pca10056): one of `stale`
    (enumerations()) is skipped, and a drive must drop first."""
    pid = pid or self.f['cdcPid']
    want = f'serial DFU {self.f["vid"]:04x}:{pid:04x} {self.b["uid"]}'
    seen = {}

    def cdc():
      usb = usb_device(self.f['vid'], pid, self.b['uid'])
      seen['drive'] = None
      if not usb or (usb['busport'], usb['ino']) in stale:
        return None
      seen['drive'] = block_device(usb['busport'])
      return usb['busport'] if not seen['drive'] and cdc_tty(usb['busport']) else None
    try:
      usb = wait_for(cdc, USB_TIMEOUT, want)
    except StepFail:
      if seen.get('drive'):
        raise StepFail(f'{want} enumerated a drive in CDC-only serial DFU', {'drive': seen['drive']}) from None
      raise
    tty = cdc_tty(usb)
    wait_for(lambda: os.access(tty, os.R_OK | os.W_OK), UDEV_TIMEOUT, f'read-write access to {tty}')
    if block_device(usb):
      raise StepFail(f'{usb} enumerated a drive in CDC-only serial DFU')
    return tty

  def enumerations(self):
    """The board's USB enumerations now, on its UF2 and CDC PIDs: (busport, ino), which a new one changes."""
    found = (usb_device(self.f['vid'], pid, self.b['uid']) for pid in {self.f['pid'], self.f['cdcPid']})
    return {(usb['busport'], usb['ino']) for usb in found if usb}

  def enter_cdc(self):
    """GPREGRET 0x4E and a reset, then the tty of the new CDC-only enumeration."""
    stale = self.enumerations()
    ev = self.probe.reenter(DFU_MAGIC_SERIAL_ONLY_RESET)
    return ev, self.cdc_only(stale=stale)

  def dfu_serial(self, pkg, tty, singlebank=True):
    out = self.nrfutil(serial_argv(pkg, tty, singlebank), 'dfu serial')
    if 'Device programmed.' not in out:   # it exits 0 on a failed upgrade too
      raise StepFail('adafruit-nrfutil did not program the device', {'tail': out.strip().splitlines()[-8:]})
    return out

  def nrfutil(self, argv, what):
    code, out = self.tools.spawn(['adafruit-nrfutil', *argv], NRFUTIL_TIMEOUT)
    if code:
      raise StepFail(f'adafruit-nrfutil {what} exited {code}', {'tail': out.strip().splitlines()[-8:]})
    return out

  def update(self, uf2, version, nonce):
    """A bootloader update UF2 copied to the drive (SKILL.md Execution, update-base): app `nonce` still runs, and
    re-entered, the drive names `version`."""
    ev = {}
    if not self.drive.present():
      ev['enter'] = self.probe.reenter()
    dev = self.drive.wait()
    ev['before'] = next((l for l in self.drive.read(dev, 'INFO_UF2.TXT')[0].decode(errors='replace').splitlines()
                         if l.startswith('UF2 Bootloader')), None)
    ev['copy'] = self.drive.copy(dev, Path(uf2).read_bytes(), self.dir)
    ev['appRan'] = self.app_ran(nonce)
    ev['reenter'] = self.probe.reenter()
    ev['info'] = self.drive.info(self.drive.wait(), version)
    return ev

  def interrupted(self, nonce):
    """An interrupted UF2 transfer (SKILL.md Execution, interrupted): app D copied up to the first block of its
    third page (src/flash_nrf5x.c writes a page once the transfer moves past it), J-Link reads two pages written and
    the third not, resets, and the mixed image runs, bank 0 being valid with CRC 0; then the whole of D runs."""
    fx = fixture(self.f, nonce, pattern=3 * PAGE)
    uf2 = make_uf2(fx['image'], fx['start'], self.f['family'])
    written, whole = Path(self.dir) / 'interrupted-written.bin', Path(self.dir) / 'interrupted-whole.bin'
    written.write_bytes(fx['image'][:2 * PAGE])
    whole.write_bytes(fx['image'])
    ev = {'enter': self.probe.reenter()}
    ev['partial'] = self.drive.copy(self.drive.wait(), uf2[:(2 * PAGE // 256 + 1) * 512], self.dir, leaves=False)
    third = fx['start'] + 2 * PAGE
    out = self.probe.session(['h', f'mem32 {BOOTLOADER_SETTINGS:08X} 1', f'verifybin {written} {fx["start"]:08X}',
                              f'mem32 {third:08X} 4', 'r', 'g'])
    settings, after = word_at(out, BOOTLOADER_SETTINGS), words_at(out, third, 4)
    if settings is None or after is None:
      raise StepFail('the bootloader settings or the third page did not read back', {'tail': out.splitlines()[-8:]})
    ev['bank0'] = {'state': settings & 0xFFFF, 'crc': settings >> 16}
    if ev['bank0'] != {'state': BANK_VALID_APP, 'crc': 0}:
      raise StepFail('bank 0 is not a valid app without a CRC: this outcome does not apply', ev)
    if struct.pack('<4I', *after) == fx['image'][2 * PAGE:2 * PAGE + 16]:
      raise StepFail("D's third page is in flash: the transfer was not cut short", ev)
    ev['readBack'] = {'written': f'{fx["start"]:#010x}+{2 * PAGE}', 'notWritten': f'{third:#010x}'}
    wait_for(self.drive.absent, USB_TIMEOUT, 'the drive leaving on the reset')
    ev['mixedRan'] = self.app_ran(nonce)
    ev['reenter'] = self.probe.reenter()
    ev['copy'] = self.drive.copy(self.drive.wait(), uf2, self.dir)
    self.probe.session(['h', f'verifybin {whole} {fx["start"]:08X}', 'g'])
    ev['appRan'] = self.app_ran(nonce)
    return {**ev, **shown(fx, nonce)}

  def softdevice(self, nonce):
    """A same-image SoftDevice and bootloader transfer and validation through the release package (SKILL.md
    Execution, softdevice): back in DFU by itself, every run of the merged HEX verifies and app `nonce` runs."""
    ev, tty = self.enter_cdc()
    ev = {'enter': ev, 'tty': tty}
    self.dfu_serial(self.f['zip'], tty)
    dev = self.drive.wait()
    commands, ev['verified'] = verify_runs(self.f)
    self.probe.session(['h', *commands, 'g'])
    ev['info'] = self.drive.info(dev)
    ev['copy'] = self.copy(nonce)
    ev['appRan'] = self.app_ran(nonce)
    return {**ev, 'package': Path(self.f['zip']).name}

  def banked(self, nonce):
    """App `nonce` through dual-bank serial DFU (SKILL.md Execution, dualbank): once the bootloader left on
    activation, it reads back in both banks with bank 0 valid."""
    fx = fixture(self.f, nonce)
    ev = self.serial(nonce, singlebank=False)
    self.cdc_gone('the bootloader leaving on activation')
    ev['bank0'] = self.bank_check(fx['image'], fx['image'], ['g'])
    return ev

  def refused(self, nonce, running):
    """App `nonce` with a flipped init packet CRC-16 (SKILL.md Execution, dualbank refused): the bootloader stays in
    DFU, bank 1 holds it, bank 0 still `running`, which runs after a J-Link reset."""
    fx = fixture(self.f, nonce)
    ev, tty = self.enter_cdc()
    pkg = self.package(fx['image'], f'refused-{nonce.hex()[:8]}', flip_crc)
    code, out = self.tools.spawn(['adafruit-nrfutil', *serial_argv(pkg, tty, False)], NRFUTIL_TIMEOUT)
    ev['nrfutil'] = {'exit': code, 'tail': out.strip().splitlines()[-2:]}   # it reports success either way
    if code or 'Activating new firmware' not in out:   # the bootloader checks the CRC on the stop packet
      raise StepFail('adafruit-nrfutil did not send the whole image and its stop packet', ev)
    stays_away(self.cdc_absent, QUIET,
               'the bootloader reset as if it activated the refused package')
    ev['bank0'] = self.bank_check(fx['image'], fixture(self.f, running)['image'], ['r', 'g'])
    self.cdc_gone('the bootloader leaving on the reset')
    ev['appRan'] = self.app_ran(running)
    return {**ev, **shown(fx, nonce)}

  def cut_short(self, nonce, running):
    """A dual-bank serial transfer of app `nonce` (CUT_PAGES) ended by adafruit-nrfutil's CUT_AFTER bound (SKILL.md
    Execution, dualbank interrupted): its first page in bank 1, not its last; `running` runs after a J-Link reset."""
    fx = fixture(self.f, nonce, pattern=CUT_PAGES * PAGE)
    enter, tty = self.enter_cdc()
    ev = {'enter': enter}
    pkg = self.package(fx['image'], f'cut-{nonce.hex()[:8]}')
    code, out = self.tools.spawn(['adafruit-nrfutil', *serial_argv(pkg, tty, False)], CUT_AFTER)
    if code != 124:
      raise StepFail(f'adafruit-nrfutil was not cut short: exit {code}', {'tail': out.strip().splitlines()[-4:]})
    ev['bank0'] = self.bank_check(fx['image'][:PAGE], fixture(self.f, running)['image'], ['r', 'g'],
                                  whole=fx['image'])
    self.cdc_gone('the bootloader leaving on the reset')
    ev['appRan'] = self.app_ran(running)
    return {**ev, **shown(fx, nonce)}

  def cdc_gone(self, what):
    wait_for(self.cdc_absent, USB_TIMEOUT, what)

  def cdc_absent(self):
    """No CDC-only bootloader with the board's serial enumerated: one whose serial cannot be read may be it."""
    return usb().usb_absent(vid_pid=(f'{self.f["vid"]:04x}', f'{self.f["cdcPid"]:04x}'), serial=self.b['uid'])

  def bank_check(self, bank1, bank0, then, whole=None):
    """Halted, bank 1 verifies `bank1`, bank 0 `bank0` and the settings mark bank 0 valid; with `whole`, the first 16
    bytes of its last page in bank 1 are not its own (its tail is 0xFF padding). Then the J-Link commands `then`."""
    files = []
    for name, data in (('bank1', bank1), ('bank0', bank0)):
      files.append(Path(self.dir) / f'{name}-check.bin')
      files[-1].write_bytes(data)
    last = (len(whole or b'') - 1) // PAGE * PAGE
    end = self.f['bank1'] + last
    out = self.probe.session(['h', f'mem32 {BOOTLOADER_SETTINGS:08X} 1', f'verifybin {files[0]} {self.f["bank1"]:08X}',
                              f'verifybin {files[1]} {self.f["appStart"]:08X}',
                              *([f'mem32 {end:08X} 4'] if whole else []), *then])
    settings = word_at(out, BOOTLOADER_SETTINGS)
    if settings is None or settings & 0xFFFF != BANK_VALID_APP:
      raise StepFail('the bootloader settings do not mark bank 0 valid', {'tail': out.splitlines()[-8:]})
    ev = {'state': settings & 0xFFFF, 'crc': settings >> 16, 'bank1': f'{self.f["bank1"]:#010x}+{len(bank1)}',
          'verified': len(bank0)}
    if whole:
      got = words_at(out, end, 4)
      if got is None or struct.pack('<4I', *got) == whole[last:last + 16]:
        raise StepFail("bank 1's last page did not read back or holds the whole image: not cut short",
                       {**ev, 'tail': out.splitlines()[-8:]})
      ev['notWritten'] = f'{end:#010x}'
    return ev

  def cdc_dfu(self, stale=()):
    """SIGNED_FW without FORCE_UF2 has no drive (src/usb/tusb_config.h, usb_desc.c): with no app the bootloader stays
    in DFU as one CDC port on its UF2 PID; `stale` as for cdc_only."""
    return {'tty': self.cdc_only(self.f['pid'], stale)}

  def refused_init(self, nonce, edit, key):
    """An init packet SIGNED_FW refuses (unsigned, or signed by another key): the fresh app erased, bank 0 invalid,
    and the bootloader reset (RESETREAS SREQ) or halted at BKPT (DHCSR S_HALT)."""
    ev, rb = self.signed_transfer(nonce, edit, key, False)
    if rb['app'] != (0xFFFFFFFF,) * 4 or not (ev['halted'] or rb['reset']):
      raise StepFail('no init packet refusal: the app is not erased, or the bootloader neither reset nor halted', ev)
    return self.back_in_dfu(ev, rb)

  def refused_image(self, nonce, key):
    """An image changed after signing with `key`: received whole, its validation requested, then refused: no reset, no
    halt, bank 0 invalid and holding it."""
    ev, rb = self.signed_transfer(nonce, tampered_after_signing(key), key, True)
    if ev['halted'] or rb['reset']:
      raise StepFail('the bootloader halted or reset on a received image instead of refusing its activation', ev)
    ev['verified'] = f'{self.f["appStart"]:#010x}+{rb["image"].stat().st_size}'
    return self.back_in_dfu(ev, rb)

  def signed_transfer(self, nonce, edit, key, received):
    """A fresh app signed with `key` running, RESETREAS cleared, app `nonce`'s package rewritten by `edit` sent
    through CDC-only serial DFU, then DHCSR read before a halt, RESETREAS, the settings and the app start read back
    (with `received`, the whole image verified) and a reset (SKILL.md Execution, signed). (evidence, read-back) once
    bank 0 reads invalid; with `received`, adafruit-nrfutil must have sent the stop packet."""
    fresh = os.urandom(16)
    ev = {'before': {'serial': self.serial(fresh, edit=signed_by(key)), 'appRan': self.app_ran(fresh)}}
    fx = fixture(self.f, nonce)
    ev['enter'], tty = self.enter_cdc()
    pkg = self.package(fx['image'], f'refused-{nonce.hex()[:8]}', edit)
    image = Path(self.dir) / f'refused-{nonce.hex()[:8]}-image.bin'
    image.write_bytes(repackage_image(pkg))
    out = self.probe.session([f'w4 {RESETREAS:08X} FFFFFFFF', f'mem32 {RESETREAS:08X} 1'])
    if word_at(out, RESETREAS) != 0:
      raise StepFail('RESETREAS did not clear', {'tail': out.splitlines()[-4:]})
    code, out = self.tools.spawn(['adafruit-nrfutil', *serial_argv(pkg, tty, True)], NRFUTIL_TIMEOUT)
    ev['nrfutil'] = {'exit': code, 'tail': out.strip().splitlines()[-2:]}
    if received and (code or 'Activating new firmware' not in out):
      raise StepFail('adafruit-nrfutil did not send the whole image and its stop packet', ev)
    stale = self.enumerations()
    out = self.probe.session([f'mem32 {DHCSR:08X} 1', 'h', f'mem32 {RESETREAS:08X} 1',
                              f'mem32 {BOOTLOADER_SETTINGS:08X} 1', f'mem32 {self.f["appStart"]:08X} 4',
                              *([f'verifybin {image} {self.f["appStart"]:08X}'] if received else []), 'r', 'g'])
    dhcsr, reas, settings = word_at(out, DHCSR), word_at(out, RESETREAS), word_at(out, BOOTLOADER_SETTINGS)
    app = words_at(out, self.f['appStart'], 4)
    if dhcsr is None or reas is None or settings is None or app is None:
      raise StepFail('DHCSR, RESETREAS, the settings or the app start did not read back',
                     {'tail': out.splitlines()[-8:]})
    ev.update(halted=bool(dhcsr & DHCSR_S_HALT), resetreas=f'{reas:08X}',
              bank0={'state': settings & 0xFFFF, 'appStart': [f'{w:08X}' for w in app]}, package=pkg.name,
              nonce=nonce.hex())
    if settings & 0xFFFF != BANK_INVALID_APP:
      raise StepFail('bank 0 is not marked invalid', ev)
    return ev, {'app': tuple(app), 'reset': bool(reas & RESETREAS_SREQ), 'image': image, 'stale': stale}

  def back_in_dfu(self, ev, rb):
    ev['dfu'] = self.cdc_dfu(rb['stale'])
    return ev

  def current_span(self, length):
    """(offset, length) of CURRENT.UF2 holding [appStart, appStart + length): one 512-byte block per 256 bytes of flash
    from USER_FLASH_START = MBR_SIZE (src/usb/uf2/ghostfat.c, uf2cfg.h)."""
    return (self.f['appStart'] - MBR_SIZE) // 256 * 512, length // 256 * 512

  def battery(self, base):
    {'default': self.default, 'dualbank': self.dualbank, 'signed': self.signed}[self.profile](base)

  def default(self, base):
    a, b, c, d, e = (os.urandom(16) for _ in range(5))
    self.step('install', self.install)
    self.step('dfu', self.dfu)
    self.step('copy', lambda: self.copy(a))
    self.step('app-ran', lambda: self.app_ran(a))
    self.step('reenter', lambda: self.reenter(a))
    self.step('copy-b', lambda: self.copy(b))
    self.step('app-ran-b', lambda: self.app_ran(b))
    self.step('double-reset', lambda: self.double_reset(b))
    self.step('serial', lambda: self.serial(c))
    self.step('app-ran-c', lambda: self.app_ran(c))
    self.step('update-base', lambda: self.update(base['path'], base['version'], c))
    self.step('update', lambda: self.update(self.f['update'], self.f['version'], c))
    self.step('interrupted', lambda: self.interrupted(d))
    self.step('softdevice', lambda: self.softdevice(e))

  def dualbank(self, base):
    a, b, c, d = (os.urandom(16) for _ in range(4))
    self.step('install', self.install)
    self.step('dfu', self.dfu)
    self.step('serial', lambda: self.banked(a))
    self.step('app-ran', lambda: self.app_ran(a))
    self.step('serial-b', lambda: self.banked(b))
    self.step('app-ran-b', lambda: self.app_ran(b))
    self.step('refused', lambda: self.refused(c, b))
    self.step('interrupted', lambda: self.cut_short(d, b))

  def signed(self, base):
    a, b, c, d, e = (os.urandom(16) for _ in range(5))
    key, other = signing_key(), secrets.randbelow(P256_N - 1) + 1
    self.step('install', self.install)
    self.step('dfu', self.cdc_dfu)
    self.step('serial', lambda: self.serial(a, edit=signed_by(key)))
    self.step('app-ran', lambda: self.app_ran(a))
    self.step('unsigned', lambda: self.refused_init(c, None, key))
    self.step('wrong-key', lambda: self.refused_init(d, signed_by(other), key))
    self.step('tampered', lambda: self.refused_image(e, key))
    self.step('serial-b', lambda: self.serial(b, edit=signed_by(key)))
    self.step('app-ran-b', lambda: self.app_ran(b))

  def restore(self):
    """The candidate installed again and the benign fixture copied through it and running; never while a tool of the
    run is not proven ended."""
    if self.tools.receipts:
      return {'pass': False, 'state': 'unknown', 'detail': 'a tool is not proven ended; not restored',
              'receipts': self.tools.receipts}
    if not self.touched:
      return {'pass': True, 'state': 'untouched', 'evidence': {}}
    ev, nonce = {}, b'restore'.ljust(16, b'\0')
    self.f, self.drive = self.home, Drive(self.b, self.home, self.tools)
    try:
      ev['install'] = self.probe.install(self.f)
      ev['dfu'] = self.dfu()
      ev['copy'] = self.copy(nonce)
      ev['appRan'] = self.app_ran(nonce)
      return {'pass': True, 'state': 'fixture running', 'evidence': ev}
    except Exception as e:
      return {'pass': False, 'state': 'unknown', 'detail': f'{type(e).__name__}: {e}',
              'evidence': {**ev, 'failed': getattr(e, 'evidence', {})}, 'receipts': self.tools.receipts}


# ------------------------------------------------------------------ locks

def log_failure(board, detail, receipts):
  """A cleanup this command under the caller's hold could not prove, appended to the board's cleanup log without
  touching its lock; '' or why it was not logged."""
  try:
    lock_helper().log_cleanup_failure(board['lock'], REASON, detail=detail, receipts=receipts)
  except Exception as e:
    return f'cleanup log not written: {type(e).__name__}: {e}'
  return ''


def note(fields, key, why):
  if why:
    fields[key] = '; '.join(filter(None, (fields.get(key), why)))


def resolve(board, found, state):
  """The board was restored: each of `found` (open failed cleanups) whose own receipts recheck done is marked restored,
  every other kept open. '' or why some stay."""
  try:
    return '; '.join(lock_helper().resolve_restored(board['lock'], found, evidence=f'{board["name"]}: {state}'))
  except Exception as e:
    return f'restorations not logged: {type(e).__name__}: {e}'


def prove_hold(board, reason):
  """A live manual hold of the board with `reason`; a run's record (protected) is never one."""
  helper = lock_helper()
  rec = helper.read_record(board['lock'])
  if not rec or rec.get('reason') != reason or not helper.holder_alive(rec) or helper.is_protected(rec):
    raise Refused(f'{board["lock"]} is not held with reason {reason!r}: take it first (hil_lock.py hold)')


def prove_cleanup_holder(board, pid):
  """The board's lock record: an older helper's live cleanup holder `pid` of this board alone, every receipt it was
  handed rechecking `done`, and no tool of a run left on the host."""
  helper = lock_helper()
  rec = helper.read_record(board['lock'])
  if not helper.is_cleanup_holder(rec) or rec.get('pid') != pid or not helper.holder_alive(rec) \
     or rec.get('locks') != [board['lock']] or not rec.get('token'):
    raise Refused(f'{board["lock"]} is not held by cleanup holder {pid} alone: {rec}')
  open_receipts = [r for r in rec.get('receipts') or [] if process().recheck(r) != 'done']
  if open_receipts:
    raise Refused(f'cleanup holder {pid} names receipts not yet done: {open_receipts}')
  if left := tools_left():
    raise Refused(f'tools of an earlier run are still on the host: {left}')
  return rec


def tools_left():
  """Every JLinkExe, mcopy, adafruit-nrfutil and hil_test.py child on the host (recovery's host-wide check); Refused
  when pgrep cannot tell."""
  p = subprocess.run(['pgrep', '-af', r'JLinkExe|mcopy|adafruit-nrfutil|hil_test\.py (fat-read|flush)'],
                     capture_output=True, text=True)
  if p.returncode not in (0, 1):   # pgrep(1): 1 none matched, 2 syntax error, 3 fatal error
    raise Refused(f'tools of an earlier run: pgrep exited {p.returncode}: {p.stderr.strip()[:200]}')
  return [l for l in p.stdout.splitlines() if l.split(maxsplit=1)[0] != str(os.getpid())]


def probe_busy(board):
  """The busy check before any hardware access (SKILL.md Reservation): agentrc's hil_flash.probe_users on the board's
  J-Link, [{'pid', 'cmd'}], empty when idle; ProbeCheckInconclusive when it cannot tell."""
  return provider().probe_users(board['flasher'])


def dut_tools(board):
  """The busy check's view of DUT-side tools, which needs no lock record or cleanup log naming them (a killed
  command's, or a run's whose failure could not be logged): [{'pid', 'cmd'}] for each process tools_left shows whose
  command line runs mcopy, adafruit-nrfutil or hil_test.py fat-read or flush on a device of a USB device with the
  board's serial, whoever started it. Refused when it cannot tell: tools_left cannot, or a command line cannot be
  read, or such a tool's device cannot be resolved (dut_device)."""
  users = []
  for line in tools_left():
    pid = int(line.split(maxsplit=1)[0])
    try:
      argv = [a.decode(errors='replace') for a in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')[:-1]]
    except (FileNotFoundError, ProcessLookupError):
      continue   # it ended
    except OSError as e:
      raise Refused(f'DUT-side tools: pid {pid}: {e}') from e
    if (serial := dut_device(argv)) and serial.lower() == board['uid'].lower():
      users.append({'pid': pid, 'cmd': ' '.join(argv)})
  return users


def dut_device(argv):
  """The USB serial of the device a DUT-side tool's `argv` names (DUT_TOOLS, run directly or as a Python script); ''
  when argv is no such tool, adafruit-nrfutil without `serial` (genpkg), or its device is a file or not on USB.
  Refused when it cannot tell: a tool naming no device, a device gone, or a serial that cannot be read."""
  prog = argv
  if argv and os.path.basename(argv[0]).startswith('python'):
    prog = provider()._python_program(argv) or []   # the script or module a Python interpreter runs, and its args
  name, args = (os.path.basename(prog[0]), prog[1:]) if prog else ('', [])
  if name not in DUT_TOOLS or name == 'hil_test.py' and args[:1] not in (['fat-read'], ['flush']) \
     or name == 'adafruit-nrfutil' and 'serial' not in args:
    return ''
  opts = DUT_TOOLS[name]
  dev = next((v for a, v in zip(args, args[1:]) if a in opts), None) or \
    next((a.partition('=')[2] for a in args if a.partition('=')[0] in opts), None)
  if not dev:
    raise Refused(f'DUT-side tool names no device: {" ".join(argv)}')
  try:
    st = os.stat(dev)
  except OSError as e:
    raise Refused(f'DUT-side tool on a device gone: {" ".join(argv)}: {e}') from e
  kind = 'block' if stat.S_ISBLK(st.st_mode) else 'char' if stat.S_ISCHR(st.st_mode) else None
  if kind is None:
    return ''   # an image file (mcopy -i)
  sysdev = f'{SYS_DEV}/{kind}/{os.major(st.st_rdev)}:{os.minor(st.st_rdev)}'
  if not os.path.exists(sysdev):
    raise Refused(f'DUT-side tool on {dev}: no {sysdev}')
  d = os.path.realpath(sysdev)
  while d != os.path.dirname(d) and not usb().BUSPORT_RE.match(os.path.basename(d)):
    d = os.path.dirname(d)
  if d == os.path.dirname(d):
    return ''   # not a USB device's
  if (serial := usb().read_sysfs(os.path.join(d, 'serial'))) is None:
    raise Refused(f'DUT-side tool on {dev}: the serial of USB device {os.path.basename(d)} cannot be read')
  return serial


def earlier_runs(failures):
  """(judged, running) for the board's open failed cleanups `failures`. judged: each as resolve_restored is to judge
  it, one a killed run left (found-left, its record's runDir) carrying the operations of its ops dir not proven ended
  (hil_process.recheck_dir, which keeps the files of every logged receipt) as its receipts, so it stays open until
  they are done; running: [{'pid', 'cmd'}] for each receipt still running ('pending'), which the busy check adds to
  the probe's users."""
  helper = lock_helper()
  keep, judged, running = helper.logged_tokens(failures), [], []
  for f in failures:
    task = f.get('task') if isinstance(f.get('task'), dict) else {}
    if f.get('how') == 'found-left' and not f.get('receipts') and not task.get('receipts') \
       and isinstance(task.get('runDir'), str):
      ops = os.path.join(task['runDir'], 'ops')
      try:
        verdict, entries = process().recheck_dir(ops, keep)
      except Exception as e:   # a recheck that cannot run proves nothing ended
        verdict, entries = 'unknown', [{'unrecheckable': f'{ops}: {type(e).__name__}: {e}'}]
      if verdict != 'done':
        f = {**f, 'receipts': entries}
    judged.append(f)
    receipts = next((r for r in (f.get('receipts'), task.get('receipts')) if isinstance(r, list) and r), [])
    states, _ = helper.settle(receipts, 0)   # one recheck pass, any error read as unknown
    pending = [r for r, s in zip(receipts, states) if s == 'pending']
    for r, text in zip(pending, helper.open_runs(pending, ['pending'] * len(pending))):
      sup = r.get('supervisor') if isinstance(r, dict) else None
      running.append({'pid': sup.get('pid') if isinstance(sup, dict) else None,
                      'cmd': f'failed cleanup {f.get("id")}: {text}'})
  return judged, running


def session_mutex(board, stack):
  """The board's session mutex (hil_lock.session_mutex), held for `stack`'s life: commands under one hold run one at a
  time. Refused when another command holds it or it cannot be used."""
  helper = lock_helper()
  try:
    stack.enter_context(helper.session_mutex(board['lock']))
  except (helper.SessionBusy, helper.LockUnavailable) as e:
    raise Refused(str(e)) from e


# ------------------------------------------------------------------ commands

def publish(result, run_dir, args, argv):
  try:
    core = report_core()
    schema = run_report.schema(core)
    attempt = core.start_attempt(run_dir, schema, False, run_dir.name)
    meta = core.make_meta(schema, attempt, termination='completed', invocation=['hil_test.py', *argv],
                          selection=[args.board], host=socket.gethostname(), roster=core.roster_identity(args.roster),
                          build={'buildRecord': args.build_record} if args.build_record else None)
    core.publish(run_dir, schema, result, meta)
  except Exception as e:   # the run's outcome stands whatever failed here
    result['reportPath'] = None
    result['reportError'] = f'report not published: {type(e).__name__}: {e}'


def cmd_run(args, argv):
  run_dir = Path(args.run_root or ROOT / 'cmake-build' / 'hil') / f'run-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}'
  run_dir.mkdir(parents=True)
  log = run_dir / 'hil.log'
  row = {'board': args.board, 'profile': args.profile, 'ran': False, 'pass': False, 'locked': False, 'steps': [],
         'cleanup': {'pass': True, 'state': 'untouched'}}
  result = {'pass': False, 'caveat': '', 'results': [row], 'reportPath': str(run_dir / 'hil_report.json'),
            'uncovered': list(UNCOVERED)}

  def finish(code):
    result['exitCode'] = code
    publish(result, run_dir, args, argv)
    print(json.dumps(result, sort_keys=True))
    return code

  try:
    board = load_board(args.roster, args.board)
    if not board['reentry']:
      raise Refused(f'{args.board} is pending commissioning (no nrf52.reentry): commissioning.md')
    if args.fail_at and args.fail_at not in PROFILE_STEPS[args.profile]:
      raise Refused(f'--fail-at {args.fail_at} is no step of profile {args.profile}: {PROFILE_STEPS[args.profile]}')
    rec, rrow, copies = admit(args.build_record, args.board, args.roster, run_dir, args.profile)
    f = facts(args.board, rrow, copies, run_dir / args.profile)
    home = None
    if args.profile != 'default':
      _, hrow, hcopies = admit(args.build_record, args.board, args.roster, run_dir)
      home = facts(args.board, hrow, hcopies, run_dir / 'default')
    row['firmware'] = {'version': f['version'], 'buildRecord': args.build_record, 'merged': rrow['sha256']['merged'],
                       'head': rec['input']['head'], 'profile': args.profile}
    base = admit_base(board, args.base_dir, run_dir)
    row['selfUpdateBase'] = {k: base[k] for k in ('version', 'file', 'sha256')}
    for tool, what in (('mcopy', 'mtools'), ('adafruit-nrfutil', 'the farm\'s hil-venv')):
      if not shutil.which(tool):
        raise Refused(f'{tool} ({what}) must be on PATH')
    try:
      bat = Battery(board, f, run_dir, log, args.fail_at, args.profile, home)
    except OSError as e:
      raise Refused(f'ops dir in {run_dir}: {type(e).__name__}: {e}')
  except Refused as e:
    row['detail'] = result['error'] = str(e)
    return finish(2)

  helper = lock_helper()
  found = []   # the take's failures as logged, for the report

  def busy():
    """probe_busy plus an earlier run's operation still running and dut_tools; the task's restored verdict resolves
    its failures as earlier_runs judged them."""
    found[:] = task.cleanup_failures
    task.cleanup_failures[:], running = earlier_runs(found)
    return [*probe_busy(board), *running, *dut_tools(board)]
  task = helper.board_task(board['lock'], REASON, busy=busy, wait=args.lock_wait, runDir=str(run_dir))
  code = 1
  try:
    with task:
      row['ran'] = True
      try:
        try:
          bat.battery(base)
          row['pass'] = True
        except StepFail as e:
          row['detail'] = f'{bat.steps[-1]["step"]}: {e}'
        row['cleanup'] = bat.restore()
        code = 0 if row['pass'] and row['cleanup']['pass'] else 1
      finally:
        row['steps'] = bat.steps
        # touched but never restored (an interrupt or an error outside a step): restore() never says untouched then
        if row['cleanup']['state'] == 'untouched' and (bat.touched or bat.tools.receipts):
          row['cleanup'] = {'pass': False, 'state': 'unknown', 'detail': 'interrupted before cleanup',
                            'receipts': bat.tools.receipts}
        give_verdict(task, board, row, bat.tools)
  except BaseException as e:   # a refused admission, or an interrupt or error after the verdict: the run aborted
    row['detail'] = f'{type(e).__name__}: {e}'
    if row['ran']:
      row['pass'], code = False, code or 1
    else:
      row.update(admission_refusal(helper, e))
  row.update(helper.failure_report(found or task.cleanup_failures))
  if end_task(task, row['cleanup']):
    code = 4
  result['pass'] = code == 0
  return finish(code)


def give_verdict(task, board, row, tools):
  """The board task's cleanup verdict, its last hardware action, by precedence failed, restored, clean: a failed step
  whose restore passed is restored."""
  cleanup = row['cleanup']
  if not cleanup['pass']:
    task.failed(detail=cleanup.get('detail') or row.get('detail') or 'cleanup failed', receipts=tools.logged())
  elif cleanup['state'] == 'untouched':
    task.clean()
  else:
    task.restored(evidence=f'{board["name"]}: {cleanup["state"]}')


def admission_refusal(helper, e):
  """A run row's fields for a board task that refused before its body (SKILL.md Results)."""
  if isinstance(e, helper.BoardBusy):
    return {'detail': str(e), 'occupiedBy': e.users}
  if isinstance(e, helper.BoardLocked):   # RecoveryBlocked among them
    return {'detail': str(e), 'locked': True, 'recoveryBlocker': isinstance(e, helper.RecoveryBlocked)}
  if isinstance(e, helper.LockUnavailable):
    return {'detail': str(e)}
  return {}   # an inconclusive busy check or an interrupt: its type names it


def end_task(task, cleanup):
  """`cleanup` (the row's) given how the board task ended; True when the run must exit 4: its cleanup failed, or the
  lock it took did not end as its verdict (or its refusal) said."""
  cleanup['released'], cleanup['lock'] = task.released, task.describe()
  if task.failure_logged is not None:
    cleanup['failureLogged'] = task.failure_logged
  note(cleanup, 'restoredNote', '; '.join(task.restore_notes))
  return not cleanup['pass'] or not task.ended_cleanly


def cmd_manual(args, argv):
  """install or restore under the caller's own hold, or restore under an older helper's cleanup holder, ended once
  it passed. Nothing here ends a hold: a restore that failed, or a tool not proven ended, is only logged."""
  workdir = Path(args.run_root or ROOT / 'cmake-build' / 'hil') / \
    f'{args.cmd}-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}'
  workdir.mkdir(parents=True)
  helper = lock_helper()
  with contextlib.ExitStack() as session:
    try:
      board = load_board(args.roster, args.board)
      session_mutex(board, session)
      holder = None
      if args.under_hold:
        prove_hold(board, args.under_hold)
      else:
        holder = prove_cleanup_holder(board, args.recover_holder)
      found = helper.open_cleanup_failures(board['lock'])
      _, rrow, copies = admit(args.build_record, args.board, args.roster, workdir)
      f = facts(args.board, rrow, copies, workdir / 'default')
      try:
        bat = Battery(board, f, workdir, workdir / 'hil.log')
      except OSError as e:
        raise Refused(f'ops dir in {workdir}: {type(e).__name__}: {e}')
      try:
        judged, running = earlier_runs(found)
        users = [*probe_busy(board), *running, *dut_tools(board)]
      except Exception as e:
        raise Refused(f'{type(e).__name__}: {e}') from e
    except Refused as e:
      print(json.dumps({'pass': False, 'error': str(e)}))
      return 2
    if users:
      print(json.dumps({'pass': False, 'error': str(helper.BoardBusy(board['lock'], users, found)),
                        'occupiedBy': users, **helper.failure_report(found)}))
      return 2
    return manual_session(args, board, holder, found, judged, bat)


def manual_session(args, board, holder, found, judged, bat):
  """The hardware part of cmd_manual, under the board's session mutex; a passing restore resolves `judged`
  (earlier_runs' view of `found`)."""
  helper = lock_helper()
  out, interrupt = {**helper.failure_report(found)}, None
  bat.touched = True
  try:
    if args.cmd == 'install':
      out.update({'pass': True, 'evidence': {'install': bat.install(), 'dfu': bat.dfu()}})
    else:
      out.update(bat.restore())
  except BaseException as e:
    out.update({'pass': False, 'error': f'{type(e).__name__}: {e}', 'evidence': getattr(e, 'evidence', {})})
    interrupt = None if isinstance(e, Exception) else e
  try:
    if not out['pass'] and (args.cmd == 'restore' or bat.tools.receipts):
      why = log_failure(board, f'{args.cmd} failed: {out.get("detail") or out.get("error")}', bat.tools.logged())
      out['failureLogged'] = not why
      note(out, 'logError', why)
    elif out['pass'] and args.cmd == 'restore':
      note(out, 'restoredNote', resolve(board, judged, out['state']))
      if holder:
        try:
          helper.release_token(holder['token'])
          out['recovered'] = {'holder': holder['pid'], 'released': True}
        except helper.ReleaseRefused as e:
          out.update({'pass': False, 'recovered': {'holder': holder['pid'], 'released': False, 'error': str(e)}})
  finally:
    print(json.dumps(out))
  if interrupt:
    raise interrupt
  return 0 if out['pass'] else 4


def fat_read_main(args):
  """One bounded read of a root-directory file of the ghostfat volume (FAT16, 512-byte sectors)."""
  try:
    fd = os.open(args.dev, os.O_RDONLY)
    try:
      bs = os.pread(fd, 512, 0)
      bps, spc, res, nfat = struct.unpack_from('<HBHB', bs, 11)
      roots, spf = struct.unpack_from('<H', bs, 17)[0], struct.unpack_from('<H', bs, 22)[0]
      if bps != 512 or not spc or not nfat or not roots or not spf:
        raise ValueError(f'implausible FAT geometry {bps, spc, res, nfat, roots, spf}')
      root = (res + nfat * spf) * bps
      data_start, csize = root + roots * 32, spc * bps
      raw = os.pread(fd, roots * 32, root)
      ents = {}
      for i in range(0, len(raw), 32):
        e = raw[i:i + 32]
        if e[0] in (0, 0xE5) or e[11] & 0x08:
          continue
        name = (e[:8].decode('ascii', 'replace').strip() + '.' + e[8:11].decode('ascii', 'replace').strip()).rstrip('.')
        ents[name] = (struct.unpack_from('<H', e, 26)[0], struct.unpack_from('<I', e, 28)[0])
      if args.file not in ents:
        raise ValueError(f'{args.file} not on the volume ({sorted(ents)})')
      cluster, size = ents[args.file]
      if args.offset % csize or args.offset > size:
        raise ValueError(f'offset {args.offset} is not a cluster boundary inside {size} bytes')
      # ghostfat lays every file out contiguously (src/usb/uf2/ghostfat.c)
      want = min(size - args.offset, args.length or size)
      data = os.pread(fd, want, data_start + (cluster - 2) * csize + args.offset)
      if len(data) != want:
        raise ValueError(f'short read: {len(data)} of {want}')
    finally:
      os.close(fd)
  except (OSError, ValueError) as e:
    print(json.dumps({'ok': False, 'error': str(e)}))
    return 1
  print(json.dumps({'ok': True, 'size': size, 'entries': sorted(ents), 'data': data.hex()}))
  return 0


def flush_main(args):
  try:
    fd = os.open(args.dev, os.O_RDONLY)
  except OSError as e:
    print(json.dumps({'ok': False, 'errno': errno.errorcode.get(e.errno)}))
    return FLUSH_GONE if e.errno in (errno.ENOENT, errno.ENXIO, errno.ENODEV) else 1
  try:
    os.fsync(fd)
  except OSError as e:
    print(json.dumps({'ok': False, 'errno': errno.errorcode.get(e.errno)}))
    return FLUSH_GONE if e.errno in (errno.ENODEV, errno.EIO) else 1
  finally:
    os.close(fd)
  print(json.dumps({'ok': True}))
  return 0


def main(argv=None):
  argv = sys.argv[1:] if argv is None else argv
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest='cmd', required=True)
  for name in ('run', 'install', 'restore'):
    p = sub.add_parser(name)
    p.add_argument('-b', '--board', required=True)
    p.add_argument('--build-record', required=True)
    p.add_argument('--run-root')
    p.add_argument('roster')
    if name == 'run':
      p.add_argument('--lock-wait', type=float, default=0)
      p.add_argument('--base-dir', required=True, help="holds the roster's nrf52.selfUpdateBase file")
      p.add_argument('--profile', choices=list(PROFILE_STEPS), default='default')
      p.add_argument('--fail-at', choices=sorted({s for steps in PROFILE_STEPS.values() for s in steps}))
    elif name == 'install':
      p.add_argument('--under-hold', required=True, metavar='REASON')
    else:
      hold = p.add_mutually_exclusive_group(required=True)
      hold.add_argument('--under-hold', metavar='REASON')
      hold.add_argument('--recover-holder', type=int, metavar='PID')
  p = sub.add_parser('fat-read')
  p.add_argument('--dev', required=True)
  p.add_argument('--file', required=True)
  p.add_argument('--offset', type=int, default=0)
  p.add_argument('--length', type=int)
  p = sub.add_parser('flush')
  p.add_argument('--dev', required=True)
  args = ap.parse_args(argv)
  if args.cmd == 'fat-read':
    return fat_read_main(args)
  if args.cmd == 'flush':
    return flush_main(args)
  if args.cmd == 'run':
    return cmd_run(args, argv)
  return cmd_manual(args, argv)


if __name__ == '__main__':
  sys.exit(main())
