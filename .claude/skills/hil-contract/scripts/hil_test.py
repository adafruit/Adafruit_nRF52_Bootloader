#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The bootloader's HIL battery on one rig board (.claude/skills/hil-contract/SKILL.md). The last stdout line is JSON.

  hil_test.py run -b BOARD --build-record REC --base-dir DIR [--run-root DIR] [--lock-wait S] [--fail-at STEP] ROSTER
  hil_test.py install -b BOARD --build-record REC --under-hold REASON ROSTER
  hil_test.py restore -b BOARD --build-record REC --under-hold REASON ROSTER
  hil_test.py fat-read --dev DEV --file NAME [--offset N] [--length N]   (child: one bounded FAT read)
  hil_test.py flush --dev DEV                                             (child: fsync of the drive)

Exit: 0 pass (and cleaned up), 1 a step failed or the board did not run, 2 refused before any hardware access,
4 cleanup failed: the board stays held by a cleanup holder (SKILL.md Recovery).
"""
import argparse
import contextlib
import errno
import hashlib
import json
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
import agentrc  # noqa: E402
import hil_run_report as run_report  # noqa: E402

REASON = 'hil_test.py'
PENDING_KIND = 'nrf52-bootloader'   # hil_lock.PendingSessions: <lock>.nrf52-bootloader-pending.json
STEPS = run_report.STEPS
REENTRY = ('gpregret',)
# nRF52 NVMC (PS v1.7 4.3.9) and POWER.GPREGRET (5.3.7.16); the same offsets on nRF52832/833
NVMC_READY, NVMC_CONFIG, NVMC_ERASEALL = 0x4001E400, 0x4001E504, 0x4001E50C
CONFIG_REN, CONFIG_EEN = 0, 2
GPREGRET = 0x4000051C
DFU_MAGIC_UF2_RESET, DFU_MAGIC_SERIAL_ONLY_RESET = 0x57, 0x4E   # src/main.c
ADAFRUIT_DEVICE_TYPE = 0x0052   # src/dfu_init.c: every init packet's device type
MBR_SIZE = 0x1000            # nrf_mbr.h
APP_FAMILY = {'nrf52840': 0xADA52840, 'nrf52833': 0x621E937A}   # src/usb/uf2/uf2cfg.h CFG_UF2_FAMILY_APP_ID
UF2_MAGIC = (0x0A324655, 0x9E5D5157, 0x0AB16F30)
UF2_FLAG_FAMILY = 0x2000
JLINK_SPEED = 4000
ERASEALL_WAIT_MS = 1000      # tERASEALL is at most 169 ms (PS v1.7 4.3.10); READY is read after it
PROBE_TIMEOUT, INSTALL_TIMEOUT, NRFUTIL_TIMEOUT = 60, 180, 120
USB_TIMEOUT, UDEV_TIMEOUT, CHILD_TIMEOUT, LEAVE_TIMEOUT, QUIET = 30, 10, 30, 15, 3
APP_SAMPLE_MS = 200
LOCK_RETRY = 15
FLUSH_GONE = 3
SETTLE = 1.0                 # s after a tool exits for its descendants to be proven gone (hil_process.run_tracked)
UICR_NRFFW = (0x10001014, 0x1000101C)   # NRFFW[0] bootloader address, NRFFW[1] MBR parameter page
ARTIFACTS = {'elf', 'hex', 'hexMbr', 'map', 'merged', 'uf2Mbr', 'update', 'zip'}   # check_build.expected_artifacts
DEFAULT_CONFIG = {'CMAKE_BUILD_TYPE': 'MinSizeRel', 'DUALBANK_FW': 'OFF', 'DEFAULT_TO_OTA_DFU': 'OFF', 'SIGNED_FW': 'OFF',
                  'SIGNED_FW_QX': '', 'SIGNED_FW_QY': '', 'FORCE_UF2': 'OFF'}
UNCOVERED = ('nRF52832 (serial DFU, no USB) and nRF52833: no rig board', 'BLE OTA DFU', 'SIGNED_FW and dual-bank builds',
             'a physical double reset', 'an interrupted update', 'a SoftDevice update')


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


def admit(record_path, board, roster_path, workdir):
  """The build record's row for `board`, checked against this checkout, the roster and the agentrc loaded here, its
  artifacts copied into `workdir` from bytes that hash as recorded. Refused otherwise."""
  try:
    rec = json.loads(Path(record_path).read_text())
  except (OSError, ValueError) as e:
    raise Refused(f'build record {record_path}: {e}')
  rows = [r for r in rec.get('boards', []) if r.get('board') == board and r.get('status') == 'ok']
  if not rec.get('pass') or len(rows) != 1 or 'hilRecord' not in rec or 'sha256' not in rows[0]:
    raise Refused(f'{record_path}: not a passing HIL record of {board} (check_build.py --board {board} --shared '
                  f'--hil-config {roster_path})')
  row, inp = rows[0], rec.get('input')
  if not isinstance(inp, dict):
    raise Refused(f'{record_path}: no input record')
  config = row.get('config') or {}
  wrong = {k: config.get(k) for k, v in {**DEFAULT_CONFIG, 'BOARD': board}.items() if config.get(k) != v}
  if rec.get('defines') or wrong:
    raise Refused(f'{record_path}: not the default configuration (defines {rec.get("defines")}, {wrong})')
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
  copies = {}
  for key, rel in row['artifacts'].items():
    src = ROOT / rel
    copies[key] = Path(workdir) / src.name
    copy_pinned(src, copies[key], row['sha256'][key], f'{rel} does not hash as {record_path} records: rebuild')
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
  hex_runs = runs(merged)
  versions = {v for _, data in hex_runs for v in re.findall(rb'UF2 Bootloader ([^\r\n\x00]+)\r\n', data)}
  if len(versions) != 1:
    raise Refused(f'{copies["merged"].name}: expected one "UF2 Bootloader <version>" string, found {len(versions)}')
  for a, b in hex_runs:
    (Path(workdir) / f'run-{a:08x}.bin').write_bytes(b)
  return {'mcu': mcu, 'family': APP_FAMILY[mcu], 'appStart': MBR_SIZE + int(sd_size, 0),
          'ramEnd': int(ram[1], 16) + int(ram[2], 16), 'vid': int(vid, 0), 'pid': int(pid, 0), 'cdcPid': int(cdc_pid, 0),
          'boardId': board_id.strip('"'), 'version': versions.pop().decode(),
          'softdevice': f'S{row["softdevice"]["name"][1:]} {row["softdevice"]["version"]}',
          'hex': str(copies['merged']), 'update': str(copies['update']), 'runs': [(a, str(Path(workdir) / f'run-{a:08x}.bin')) for a, _ in hex_runs]}


# ------------------------------------------------------------------ the fixture app

def fixture(f, nonce):
  """An app the probe can prove ran, and which one: SP and reset vectors, every other vector a fault loop, a reset
  handler that loads a word of `nonce` into r1 and counts in r0 forever, then a 4 KiB pattern from `nonce`, so an
  earlier copy's image reads back different."""
  start, table = f['appStart'], 64
  handler = start + table
  loop, fault = handler + 2, handler + 6
  marker = int.from_bytes(hashlib.sha256(b'marker' + nonce).digest()[:4], 'little')
  vectors = [f['ramEnd'], handler | 1] + [fault | 1] * (table // 4 - 2)
  # ldr r1, [pc, #4] (the marker) ; loop: adds r0, #1 ; b loop ; fault: b . ; marker
  code = struct.pack('<HHHHI', 0x4901, 0x3001, 0xE7FD, 0xE7FE, marker)
  pattern = b''.join(hashlib.sha256(nonce + i.to_bytes(4, 'little')).digest() for i in range(128))
  img = struct.pack(f'<{len(vectors)}I', *vectors) + code + pattern
  img += b'\xff' * (-len(img) % 256)
  return {'image': img, 'handler': handler, 'loop': loop, 'fault': fault, 'marker': marker, 'start': start}


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


# ------------------------------------------------------------------ processes

def private_dir(path):
  """`path` created as a directory nobody else can write, in a parent likewise (run_tracked's state dir), whatever
  the umask."""
  path = Path(path)
  path.mkdir(parents=True, exist_ok=True)
  os.chmod(path.parent, 0o755)
  os.chmod(path, 0o700)
  return str(path)


class Tools:
  """Every child runs under hil_process.run_tracked, which proves its process tree ended (`cleanup` 'done') or names
  a receipt; one that is not proven ended stops the battery (NotEnded) and its receipt goes to the cleanup holder.
  run_tracked keeps a child's files in `dir` until it is proven ended: the board's pending mark names `dir` (claim)."""
  def __init__(self, workdir, log):
    self.dir, self.log, self.receipts = private_dir(Path(workdir) / 'ops'), log, []

  def run(self, cmd, cwd=None, *, env=None, timeout):
    """hil_flash's runner contract: a CompletedProcess carrying `cleanup` and `receipt`."""
    if env:
      raise ValueError('no tool here takes an environment')
    argv = [str(a) for a in cmd]
    before = set(os.listdir(self.dir))
    try:
      r = process().run_tracked(argv, cwd=cwd, timeout=timeout, settle=SETTLE, state_dir=self.dir, quiet=True)
    except BaseException as e:
      cleanup, receipt = getattr(e, 'cleanup', 'unknown'), getattr(e, 'receipt', None)
      if cleanup == 'done' and receipt is None:   # proven ended; a supervisor that never started leaves its log
        for name in set(os.listdir(self.dir)) - before:
          with contextlib.suppress(OSError):
            os.unlink(os.path.join(self.dir, name))
      self.note(argv, cleanup, receipt)
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

class Probe:
  """J-Link sessions through agentrc's provider, each command judged from its own output."""
  def __init__(self, board, workdir, tools):
    self.b, self.tools = board, tools
    self.profile = {'interface': 'swd', 'speed': JLINK_SPEED, 'autoconnect': True, 'device': board['device'],
                    'workdir': str(workdir)}

  def session(self, commands, timeout=PROBE_TIMEOUT, judge=True):
    def crashed(a):   # the OB's libjlinkarm crash before any target access
      out = process().cmd_stdout_text(a.result.stdout)
      return a.result.returncode in provider().JLINK_CRASH_RCS and 'Found Cortex' not in out
    op = provider().run_jlink_commands(self.b['flasher'], list(commands), **self.profile, timeout=timeout,
                                       run=self.tools.run, retry_if=crashed, retry_delay=2)
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
                        f'loadfile {f["hex"]}', 'h', *[f'verifybin {p} {a:08X}' for a, p in f['runs']], 'r', 'g'],
                       timeout=INSTALL_TIMEOUT)
    ready = word_at(out, NVMC_READY)
    if ready is None or ready & 1 != 1:
      raise StepFail(f'NVMC not ready after ERASEALL ({"no read" if ready is None else f"{ready:08X}"})')
    return {'erased': True, 'verified': [f'{a:#010x}+{Path(p).stat().st_size}' for a, p in f['runs']]}

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


def word_at(out, addr):
  """The word a J-Link `mem32 <addr> 1` printed in `out`, None when it printed none."""
  m = re.search(rf'^{addr:08X} = ([0-9A-F]{{8}})', out, re.M)
  return int(m[1], 16) if m else None


# ------------------------------------------------------------------ USB drive

def usb_device(vid, pid, serial):
  hits = []
  for d in Path('/sys/bus/usb/devices').iterdir():
    try:
      if (int((d / 'idVendor').read_text(), 16), int((d / 'idProduct').read_text(), 16)) == (vid, pid) and \
         (d / 'serial').read_text().strip() == serial:
        hits.append(d.name)
    except (OSError, ValueError):
      continue
  if len(hits) > 1:
    raise StepFail(f'{len(hits)} USB devices {vid:04x}:{pid:04x} with serial {serial}: {hits}')
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
    return block_device(usb) if usb else None

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

  def copy(self, dev, uf2, workdir):
    path = Path(workdir) / 'copy.uf2'
    path.write_bytes(uf2)
    code, out = self.tools.spawn(['mcopy', '-o', '-i', dev, path, '::APP.UF2'], 60)
    if code:
      raise StepFail('mcopy failed', {'tail': out.strip().splitlines()[-5:]})
    code, res = self.tools.child_json(['flush', '--dev', dev], CHILD_TIMEOUT, 'flush')
    if code not in (0, FLUSH_GONE):
      raise StepFail('flush failed', {'result': res})
    wait_for(lambda: not self.present(), LEAVE_TIMEOUT, 'the drive leaving after the copy')
    return {'flush': 'flushed' if code == 0 else 'device gone before flush'}


# ------------------------------------------------------------------ the battery

class Battery:
  def __init__(self, board, f, workdir, log, fail_at=None):
    self.b, self.f, self.dir, self.fail_at = board, f, Path(workdir), fail_at
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
    return {**ev, 'image': f'{fx["start"]:#010x}+{len(fx["image"])}', 'nonce': nonce.hex()}

  def app_ran(self, nonce):
    """The app the copy wrote runs after the bootloader's own reset: no probe reset first. The drive stays away, then
    two samples: thread mode, PC in the counting loop, r1 this copy's marker, r0 advancing."""
    fx = fixture(self.f, nonce)
    stays_away(self.drive.present, QUIET, 'the UF2 drive came back instead of the app')
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

  def reenter(self, nonce):
    fx = fixture(self.f, nonce)
    ev = self.probe.reenter()
    dev = self.drive.wait()
    blob, res = self.drive.read(dev, 'CURRENT.UF2', *self.current_span(len(fx['image'])))
    got = uf2_payload(blob, fx['start'], len(fx['image']))
    if got != fx['image']:
      raise StepFail('CURRENT.UF2 does not read back the copied app', {'read': len(got)})
    return {**ev, 'method': 'gpregret', 'readBack': len(got)}

  def serial(self, nonce):
    """App C through CDC-only serial DFU: GPREGRET 0x4E enumerates the bootloader as one CDC port and no drive
    (src/main.c, src/usb/usb_desc.c); adafruit-nrfutil sends the package and the bootloader starts the app."""
    fx = fixture(self.f, nonce)
    ev = self.probe.reenter(DFU_MAGIC_SERIAL_ONLY_RESET)
    usb = wait_for(lambda: usb_device(self.f['vid'], self.f['cdcPid'], self.b['uid']), USB_TIMEOUT,
                   f'serial DFU {self.f["vid"]:04x}:{self.f["cdcPid"]:04x} {self.b["uid"]}')
    tty = wait_for(lambda: cdc_tty(usb), UDEV_TIMEOUT, f'the CDC tty of {usb}')
    wait_for(lambda: os.access(tty, os.R_OK | os.W_OK), UDEV_TIMEOUT, f'read-write access to {tty}')
    if block_device(usb):
      raise StepFail(f'{usb} enumerated a drive in CDC-only serial DFU')
    app, pkg = self.dir / 'app-serial.bin', self.dir / 'app-serial.zip'
    app.write_bytes(fx['image'])
    self.nrfutil(['dfu', 'genpkg', '--dev-type', f'{ADAFRUIT_DEVICE_TYPE:#06x}', '--application', app, pkg], 'genpkg')
    out = self.nrfutil(['--verbose', 'dfu', 'serial', '--package', pkg, '-p', tty, '-b', '115200', '--singlebank'],
                       'dfu serial')
    if 'Device programmed.' not in out:   # it exits 0 on a failed upgrade too
      raise StepFail('adafruit-nrfutil did not program the device', {'tail': out.strip().splitlines()[-8:]})
    return {**ev, 'tty': tty, 'image': f'{fx["start"]:#010x}+{len(fx["image"])}', 'nonce': nonce.hex()}

  def nrfutil(self, argv, what):
    code, out = self.tools.spawn(['adafruit-nrfutil', *argv], NRFUTIL_TIMEOUT)
    if code:
      raise StepFail(f'adafruit-nrfutil {what} exited {code}', {'tail': out.strip().splitlines()[-8:]})
    return out

  def update(self, uf2, version, nonce):
    """A bootloader update UF2 copied to the drive: the bootloader stages it in the app region's top and has the MBR
    copy it over itself (src/usb/uf2/ghostfat.c, src/usb/msc_uf2.c). The app `nonce` still runs after it, and
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

  def current_span(self, length):
    """(offset, length) of CURRENT.UF2 holding [appStart, appStart + length): one 512-byte block per 256 bytes of flash
    from USER_FLASH_START = MBR_SIZE (src/usb/uf2/ghostfat.c, uf2cfg.h)."""
    return (self.f['appStart'] - MBR_SIZE) // 256 * 512, length // 256 * 512

  def battery(self, base):
    a, b, c = os.urandom(16), os.urandom(16), os.urandom(16)
    self.step('install', self.install)
    self.step('dfu', self.dfu)
    self.step('copy', lambda: self.copy(a))
    self.step('app-ran', lambda: self.app_ran(a))
    self.step('reenter', lambda: self.reenter(a))
    self.step('copy-b', lambda: self.copy(b))
    self.step('app-ran-b', lambda: self.app_ran(b))
    self.step('serial', lambda: self.serial(c))
    self.step('app-ran-c', lambda: self.app_ran(c))
    self.step('update-base', lambda: self.update(base['path'], base['version'], c))
    self.step('update', lambda: self.update(self.f['update'], self.f['version'], c))

  def restore(self):
    """The candidate installed again and the benign fixture copied through it and running; never while a tool of the
    run is not proven ended."""
    if not self.touched:
      return {'pass': True, 'state': 'untouched', 'evidence': {}}
    if self.tools.receipts:
      return {'pass': False, 'state': 'unknown', 'detail': 'a tool is not proven ended; not restored',
              'receipts': self.tools.receipts}
    ev, nonce = {}, b'restore'.ljust(16, b'\0')
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

def take_lock(board, run_dir, wait):
  helper = lock_helper()
  end = time.monotonic() + wait
  while True:
    try:
      return helper.take_board_lock(board['lock'], REASON, protected=True, runDir=str(run_dir)), None
    except helper.RecoveryBlocked as e:
      return None, {'locked': True, 'recoveryBlocker': True, 'detail': str(e)}
    except helper.BoardLocked as e:
      if time.monotonic() >= end:
        return None, {'locked': True, 'recoveryBlocker': False, 'detail': str(e)}
      time.sleep(min(LOCK_RETRY, max(0, end - time.monotonic())))


def claim(board, ops):
  """The board's pending mark names this invocation and its ops dir before its first tool starts; refused while it
  names another invocation not ended (invocation_ended), so a later one, a recovery's included, refuses while this one
  lives or a tool it started is not proven ended. add() runs under the mark's mutex: of two invocations claiming at
  once, the later one lists the earlier."""
  helper = lock_helper()
  me = {'ops': os.path.realpath(ops), 'pid': os.getpid(), 'start': helper.proc_start(os.getpid())}
  try:
    if me['start'] is None:
      raise OSError('own start time unreadable')
    listed = helper.PendingSessions(board['lock'], PENDING_KIND).add([{'receipt': me, 'cmd': 'hil_test.py'}])
  except (OSError, ValueError, helper.PendingRefused) as e:
    raise Refused(f'the board\'s pending mark could not name this invocation: {type(e).__name__}: {e}')
  left = [e['receipt'] for e in listed if e['receipt'] != me and not ended_quietly(board, e)]
  if left:
    raise Refused(f'an earlier invocation, or a tool it started, is not proven ended: {left}')


def invocation_ended(board, session):
  """A claim's invocation is gone (or is this one) and every tool it started rechecks done: run_tracked removes a
  child's files once it is, so each hil_op_*.log left in its ops dir is one to recheck. An ops dir this user cannot
  list, another user's or gone, is not ended (the listing raises). A receipt the board's holder names keeps its files:
  a cleanup holder's release rechecks them."""
  rec = lock_helper().read_record(board['lock'])
  held = {h.get('token') for h in rec.get('receipts') or [] if isinstance(h, dict)} if isinstance(rec, dict) else set()
  r = session['receipt']
  if r['pid'] != os.getpid() and lock_helper().proc_start(r['pid']) == r['start']:
    return False
  for name in os.listdir(r['ops']):
    if not (name.startswith('hil_op_') and name.endswith('.log')):
      continue
    base = os.path.join(r['ops'], name[:-len('.log')])
    receipt = {'token': name[len('hil_op_'):-len('.log')], 'supervisor': None, 'lock': f'{base}.lock',
               'record': f'{base}.json', 'log': f'{base}.log'}
    if process().recheck(receipt) != 'done':
      return False
    if receipt['token'] not in held:
      process().tidy(receipt)
  return True


def ended_quietly(board, session):
  try:
    return invocation_ended(board, session) is True
  except Exception:
    return False


def unmark(board):
  """The mark removed once every invocation it names, this one included, ended; kept otherwise, for the next claim
  to judge."""
  helper = lock_helper()
  try:
    helper.PendingSessions(board['lock'], PENDING_KIND).clear(lambda s: invocation_ended(board, s))
  except (OSError, helper.PendingRefused):
    pass


def prove_hold(board, reason):
  helper = lock_helper()
  rec = helper.read_record(board['lock'])
  if not rec or rec.get('reason') != reason or not helper.holder_alive(rec):
    raise Refused(f'{board["lock"]} is not held with reason {reason!r}: take it first (hil_lock.py hold)')


def prove_cleanup_holder(board, pid):
  """The board's lock record: a live cleanup holder `pid` (a failed run's, or a takeover's) of this board alone, every
  receipt it was handed rechecking `done`, and no tool of a run left on the host."""
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
  """Every JLinkExe, mcopy, adafruit-nrfutil and hil_test.py child on the host (recovery's host-wide check)."""
  out = subprocess.run(['pgrep', '-af', r'JLinkExe|mcopy|adafruit-nrfutil|hil_test\.py (fat-read|flush)'],
                       capture_output=True, text=True).stdout
  return [l for l in out.splitlines() if l.split(maxsplit=1)[0] != str(os.getpid())]


def probe_users(board):
  out = subprocess.run(['pgrep', '-af', 'JLinkExe'], capture_output=True, text=True).stdout
  return [l for l in out.splitlines() if board['flasher']['uid'] in l.split()]


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
  row = {'board': args.board, 'ran': False, 'pass': False, 'locked': False, 'steps': [],
         'cleanup': {'pass': True, 'state': 'untouched'}}
  result = {'pass': False, 'caveat': '', 'results': [row], 'reportPath': str(run_dir / 'hil_report.json'),
            'heldInProcess': False, 'uncovered': list(UNCOVERED)}

  def finish(code):
    result['exitCode'] = code
    publish(result, run_dir, args, argv)
    print(json.dumps(result, sort_keys=True))
    return code

  try:
    board = load_board(args.roster, args.board)
    if not board['reentry']:
      raise Refused(f'{args.board} is pending commissioning (no nrf52.reentry): commissioning.md')
    rec, rrow, copies = admit(args.build_record, args.board, args.roster, run_dir)
    f = facts(args.board, rrow, copies, run_dir)
    row['firmware'] = {'version': f['version'], 'buildRecord': args.build_record, 'merged': rrow['sha256']['merged'],
                       'head': rec['input']['head']}
    base = admit_base(board, args.base_dir, run_dir)
    row['selfUpdateBase'] = {k: base[k] for k in ('version', 'file', 'sha256')}
    for tool, what in (('mcopy', 'mtools'), ('adafruit-nrfutil', 'the farm\'s hil-venv')):
      if not shutil.which(tool):
        raise Refused(f'{tool} ({what}) must be on PATH')
  except Refused as e:
    row['detail'] = result['error'] = str(e)
    return finish(2)

  fh, held = take_lock(board, run_dir, args.lock_wait)
  if fh is None:
    row.update(held)
    return finish(1)
  bat = Battery(board, f, run_dir, log, args.fail_at)
  code, row['cleanup'] = 1, None
  try:
    try:
      claim(board, bat.tools.dir)
      blocked, users = None, probe_users(board)
    except Refused as e:
      blocked, users = str(e), []
    if blocked:
      row.update(recoveryBlocker=True, detail=blocked)
    elif users:
      row.update(probeUsers=users, detail='another JLinkExe uses the probe')
    else:
      row['ran'] = True
      try:
        bat.battery(base)
        row['pass'] = True
      except StepFail as e:
        row['detail'] = f'{bat.steps[-1]["step"]}: {e}'
      row['cleanup'] = bat.restore()
      code = 0 if row['pass'] and row['cleanup']['pass'] else 1
  except BaseException as e:   # an interrupt or an error outside a step: reported, the board handed over below
    row['detail'] = f'{type(e).__name__}: {e}'
  finally:
    row['steps'] = bat.steps
    if row['cleanup'] is None:   # never restored: an interrupt, or nothing written
      row['cleanup'] = ({'pass': False, 'state': 'unknown', 'detail': 'interrupted before cleanup',
                         'receipts': bat.tools.receipts} if bat.touched else {'pass': True, 'state': 'untouched'})
    if not row['cleanup']['pass']:
      try:
        token = lock_helper().hand_to_cleanup_holder(
          {board['lock']: fh}, REASON, detail=row['cleanup'].get('detail'),
          receipts=[r['receipt'] for r in bat.tools.receipts if r['receipt'] is not None])
        row['cleanup'].update(heldAfter=True, holder={'token': token})
      except Exception as e:
        row['cleanup'].update(heldAfter=True, holder={'error': str(e)})
      fh.close()
      code = 4
    else:
      unmark(board)
      lock_helper().release_board_lock(fh)
  result['pass'] = code == 0
  return finish(code)


def cmd_manual(args, argv):
  """install or restore under the caller's own hold, or restore under a cleanup holder, ended once it passed."""
  workdir = Path(args.run_root or ROOT / 'cmake-build' / 'hil') / \
    f'{args.cmd}-{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}'
  workdir.mkdir(parents=True)
  try:
    board = load_board(args.roster, args.board)
    holder = None
    if args.under_hold:
      prove_hold(board, args.under_hold)
    else:
      holder = prove_cleanup_holder(board, args.recover_holder)
    _, rrow, copies = admit(args.build_record, args.board, args.roster, workdir)
    f = facts(args.board, rrow, copies, workdir)
    bat = Battery(board, f, workdir, workdir / 'hil.log')
    claim(board, bat.tools.dir)
    if probe_users(board):
      raise Refused('another JLinkExe uses the probe')
  except Refused as e:
    print(json.dumps({'pass': False, 'error': str(e)}))
    return 2
  bat.touched = True
  try:
    if args.cmd == 'install':
      ev = {'install': bat.install(), 'dfu': bat.dfu()}
      out = {'pass': True, 'evidence': ev}
    else:
      out = bat.restore()
  except StepFail as e:
    print(json.dumps({'pass': False, 'error': str(e), 'evidence': e.evidence}))
    return 4
  if out['pass'] and holder:
    try:
      lock_helper().release_token(holder['token'])
      out['recovered'] = {'holder': holder['pid'], 'released': True}
    except lock_helper().ReleaseRefused as e:
      out.update({'pass': False, 'recovered': {'holder': holder['pid'], 'released': False, 'error': str(e)}})
  if out['pass']:
    unmark(board)
  print(json.dumps(out))
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
      p.add_argument('--fail-at', choices=STEPS)
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
