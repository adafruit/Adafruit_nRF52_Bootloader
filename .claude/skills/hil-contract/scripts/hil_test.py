#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The bootloader's HIL battery on one rig board (.claude/skills/hil-contract/SKILL.md). The last stdout line is JSON.

  hil_test.py run -b BOARD --build-record REC [--run-root DIR] [--lock-wait S] [--fail-at STEP] ROSTER
  hil_test.py install -b BOARD --build-record REC --under-hold REASON ROSTER
  hil_test.py restore -b BOARD --build-record REC --under-hold REASON ROSTER
  hil_test.py fat-read --dev DEV --file NAME [--offset N] [--length N]   (child: one bounded FAT read)
  hil_test.py flush --dev DEV                                             (child: fsync of the drive)

Exit: 0 pass (and cleaned up), 1 a step failed or the board did not run, 2 refused before any hardware access,
4 cleanup failed: the board stays held by a cleanup holder (SKILL.md Recovery).
"""
import argparse
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
STEPS = ('install', 'dfu', 'copy', 'app-ran', 'reenter', 'copy-b', 'app-ran-b')
REENTRY = ('gpregret',)
# nRF52 NVMC (PS v1.7 4.3.9) and POWER.GPREGRET (5.3.7.16); the same offsets on nRF52832/833
NVMC_READY, NVMC_CONFIG, NVMC_ERASEALL = 0x4001E400, 0x4001E504, 0x4001E50C
CONFIG_REN, CONFIG_EEN = 0, 2
GPREGRET = 0x4000051C
DFU_MAGIC_UF2_RESET = 0x57   # src/main.c
MBR_SIZE = 0x1000            # nrf_mbr.h
APP_FAMILY = {'nrf52840': 0xADA52840, 'nrf52833': 0x621E937A}   # src/usb/uf2/uf2cfg.h CFG_UF2_FAMILY_APP_ID
UF2_MAGIC = (0x0A324655, 0x9E5D5157, 0x0AB16F30)
UF2_FLAG_FAMILY = 0x2000
JLINK_SPEED = 4000
ERASEALL_WAIT_MS = 1000      # tERASEALL is at most 169 ms (PS v1.7 4.3.10); READY is read after it
PROBE_TIMEOUT, INSTALL_TIMEOUT = 60, 180
USB_TIMEOUT, CHILD_TIMEOUT, LEAVE_TIMEOUT, QUIET = 30, 30, 15, 3
APP_SAMPLE_MS = 200
LOCK_RETRY = 15
FLUSH_GONE = 3
JLINK_CRASH_RCS = {139, -11, 134, -6}
CHILDREN_ALIVE = []


class Refused(Exception):
  pass


class StepFail(Exception):
  def __init__(self, msg, evidence=None):
    super().__init__(msg)
    self.evidence = evidence or {}


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
  return {'name': name, 'entry': resolved, 'flasher': flasher, 'device': m[1], 'lock': lock,
          'uid': entry.get('uid'), 'reentry': extra.get('reentry')}


def sha256(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
  env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
  return subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, text=True, env=env).stdout


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
  row, inp = rows[0], rec['input']
  if inp.get('dirty') or inp.get('head') != git('rev-parse', 'HEAD').strip() or git('status', '--porcelain').strip():
    raise Refused(f'{record_path} was built from {inp.get("head")} (dirty {inp.get("dirty")}); this checkout is '
                  f'{git("rev-parse", "HEAD").strip()} (dirty {bool(git("status", "--porcelain").strip())}): rebuild '
                  'on a clean HEAD')
  subs = {}
  for line in git('submodule', 'status').splitlines():
    subs[line[1:].split()[1]] = line[1:].split()[0]
  if {p: s['commit'] for p, s in inp.get('submodules', {}).items()} != subs:
    raise Refused(f'{record_path}: submodules differ from this checkout ({subs})')
  if rec['hilRecord']['roster']['sha256'] != sha256(roster_path):
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
    if not src.is_file() or src.is_symlink() or sha256(src) != row['sha256'][key]:
      raise Refused(f'{rel} does not hash as {record_path} records: rebuild')
    copies[key] = Path(workdir) / src.name
    shutil.copyfile(src, copies[key])
    if sha256(copies[key]) != row['sha256'][key]:
      raise Refused(f'{rel} changed while it was copied')
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
    count, addr, kind, body = raw[0], int.from_bytes(raw[1:3], 'big'), raw[3], raw[4:4 + raw[0]]
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


def define(text, name):
  m = re.search(rf'^\s*#define\s+{name}\s+(.+?)\s*(//.*)?$', text, re.M)
  return m[1] if m else None


def facts(board, row, copies, workdir):
  """What the battery checks the board against, all from the record's build and this checkout's sources."""
  sys.path.insert(0, str(ROOT / '.claude/skills/build/scripts'))
  import check_build
  mcu, sd = check_build.board_config(board)
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
  if not (sd_size and ram and vid and pid and board_id):
    raise Refused(f'{board}: cannot establish SD_FLASH_SIZE, RAM, USB_DESC_VID/UF2_PID or UF2_BOARD_ID')
  merged = parse_hex(copies['merged'])
  image = bytes(merged.get(a, 0xFF) for a in range(min(merged), max(merged) + 1)) if merged else b''
  versions = set(re.findall(rb'UF2 Bootloader ([^\r\n\x00]+)\r\n', image))
  if len(versions) != 1:
    raise Refused(f'{copies["merged"].name}: expected one "UF2 Bootloader <version>" string, found {len(versions)}')
  hex_runs = runs(merged)
  for i, (a, b) in enumerate(hex_runs):
    (Path(workdir) / f'run-{a:08x}.bin').write_bytes(b)
  return {'mcu': mcu, 'family': APP_FAMILY[mcu], 'appStart': MBR_SIZE + int(sd_size, 0),
          'ramEnd': int(ram[1], 16) + int(ram[2], 16), 'vid': int(vid, 0), 'pid': int(pid, 0),
          'boardId': board_id.strip('"'), 'version': versions.pop().decode(),
          'softdevice': f'S{row["softdevice"]["name"][1:]} {row["softdevice"]["version"]}',
          'hex': str(copies['merged']), 'runs': [(a, str(Path(workdir) / f'run-{a:08x}.bin')) for a, _ in hex_runs]}


# ------------------------------------------------------------------ the fixture app

def fixture(f, nonce):
  """An app the probe can prove ran: SP and reset vectors, every other vector a fault loop, a reset handler that
  counts in r0 forever, then a 4 KiB pattern from `nonce`, so an earlier copy's image reads back different."""
  start, table = f['appStart'], 64
  handler, fault = start + table, start + table + 4
  vectors = [f['ramEnd'], handler | 1] + [fault | 1] * (table // 4 - 2)
  code = struct.pack('<HHH', 0x3001, 0xE7FD, 0xE7FE)   # adds r0,#1 ; b handler ; b . (fault)
  pattern = b''.join(hashlib.sha256(nonce + i.to_bytes(4, 'little')).digest() for i in range(128))
  img = struct.pack(f'<{len(vectors)}I', *vectors) + code
  img += b'\0' * (-len(img) % 4) + pattern
  img += b'\xff' * (-len(img) % 256)
  return {'image': img, 'handler': handler, 'fault': fault, 'start': start}


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

def spawn(argv, log, timeout, cwd=None):
  """(exit, output) of a bounded child; its transcript appended to `log`. rc 124 on the bound (its session killed)."""
  r = process().run_cmd([str(a) for a in argv], cwd=cwd, timeout=timeout, quiet=True)
  out = process().cmd_stdout_text(r.stdout) + process().cmd_stdout_text(r.stderr or '')
  with open(log, 'a') as fh:
    fh.write(f'$ {" ".join(map(str, argv))}\n{out}\n# exit {r.returncode}\n')
  return r.returncode, out


def tool_runner(log):
  def run(cmd, cwd=None, *, env=None, timeout):
    if env:
      raise ValueError('no tool here takes an environment')
    code, out = spawn(cmd, log, timeout, cwd)
    r = subprocess.CompletedProcess(cmd, code, out, None)
    r.cleanup = 'done' if code != 124 else 'unknown'
    return r
  return run


def child_json(argv, log, timeout, what):
  code, out = spawn([sys.executable, '-B', __file__, *argv], log, timeout)
  try:
    res = json.loads(out.strip().splitlines()[-1])
  except (IndexError, ValueError):
    res = None
  if code == 124:
    raise StepFail(f'{what}: timed out after {timeout}s')
  return code, res


# ------------------------------------------------------------------ probe

class Probe:
  """J-Link sessions through agentrc's provider, each command judged from its own output."""
  def __init__(self, board, workdir, log):
    self.b, self.dir, self.log = board, str(workdir), log
    self.profile = {'interface': 'swd', 'speed': JLINK_SPEED, 'autoconnect': True, 'device': board['device'],
                    'workdir': self.dir}

  def session(self, commands, timeout=PROBE_TIMEOUT, judge=True):
    def crashed(a):   # the OB's libjlinkarm crash before any target access (agentrc hil_flash JLINK_CRASH_RCS)
      out = process().cmd_stdout_text(a.result.stdout)
      return a.result.returncode in JLINK_CRASH_RCS and 'Found Cortex' not in out
    op = provider().run_jlink_commands(self.b['flasher'], list(commands), **self.profile, timeout=timeout,
                                       run=tool_runner(self.log), retry_if=crashed, retry_delay=2)
    out, last = process().cmd_stdout_text(op.stdout), op.attempts[-1]
    tail = {'tail': out.strip().splitlines()[-6:]}
    if last.timed_out:
      raise StepFail(f'JLinkExe timed out after {timeout}s', tail)
    if judge and (problem := provider().jlink_problem(op)):
      raise StepFail(f'J-Link session failed: {problem}', tail)
    return out

  def mem32(self, addr):
    out = self.session(['h', f'mem32 {addr:08X} 1', 'g'])
    m = re.search(rf'^{addr:08X} = ([0-9A-F]{{8}})', out, re.M)
    if not m:
      raise StepFail(f'no value read at {addr:#010x}')
    return int(m[1], 16)

  def install(self, f):
    """Erase all (NVMC, CPU halted: nRF52840 rev3 errata [184]), load the merged HEX, then verify every run of it."""
    out = self.session(['h', f'w4 {NVMC_CONFIG:08X} {CONFIG_EEN:08X}', f'w4 {NVMC_ERASEALL:08X} 00000001',
                        f'sleep {ERASEALL_WAIT_MS}', f'mem32 {NVMC_READY:08X} 1', f'w4 {NVMC_CONFIG:08X} {CONFIG_REN:08X}',
                        f'loadfile {f["hex"]}'], timeout=INSTALL_TIMEOUT)
    ready = re.search(rf'^{NVMC_READY:08X} = ([0-9A-F]{{8}})', out, re.M)
    if not ready or int(ready[1], 16) & 1 != 1:
      raise StepFail(f'NVMC not ready after ERASEALL ({ready[1] if ready else "no read"})')
    self.session(['h', *[f'verifybin {p} {a:08X}' for a, p in f['runs']], 'r', 'g'], timeout=INSTALL_TIMEOUT)
    return {'erased': True, 'verified': [f'{a:#010x}+{Path(p).stat().st_size}' for a, p in f['runs']]}

  def regs(self):
    out = self.session(['h', 'regs', 'g'])
    def one(pat):
      m = re.search(pat, out)
      if not m:
        raise StepFail(f'no {pat!r} in J-Link regs output')
      return int(m[1], 16)
    return {'pc': one(r'\bPC = ([0-9A-F]{8})'), 'r0': one(r'\bR0 = ([0-9A-F]{8})'),
            'ipsr': one(r'\bIPSR = ([0-9A-F]{3})')}

  def reenter(self):
    """GPREGRET = DFU_MAGIC_UF2_RESET, read back, then SYSRESETREQ: a soft reset keeps the retained register (PS
    v1.7 5.3.6.8), and the bootloader enters UF2 DFU on it (src/main.c)."""
    out = self.session(['h', f'w4 {GPREGRET:08X} {DFU_MAGIC_UF2_RESET:08X}', f'mem32 {GPREGRET:08X} 1', 'g'])
    m = re.search(rf'^{GPREGRET:08X} = ([0-9A-F]{{8}})', out, re.M)
    if not m or int(m[1], 16) & 0xFF != DFU_MAGIC_UF2_RESET:
      raise StepFail(f'GPREGRET reads back {m[1] if m else "nothing"}')
    op = provider().sysresetreq_jlink(self.b['flasher'], **self.profile, timeout=PROBE_TIMEOUT,
                                      run=tool_runner(self.log))
    if op.attempts[-1].timed_out or 'Writing 05FA0004 -> E000ED0C' not in process().cmd_stdout_text(op.stdout):
      raise StepFail('the SYSRESETREQ write was not issued')
    return {'gpregret': m[1]}


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
  def __init__(self, board, f, log):
    self.b, self.f, self.log = board, f, log

  def present(self):
    usb = usb_device(self.f['vid'], self.f['pid'], self.b['uid'])
    return block_device(usb) if usb else None

  def wait(self):
    dev = wait_for(self.present, USB_TIMEOUT, f'UF2 drive {self.f["vid"]:04x}:{self.f["pid"]:04x} {self.b["uid"]}')
    exclusive(dev)
    return dev

  def read(self, dev, name, offset=0, length=None):
    argv = ['fat-read', '--dev', dev, '--file', name, '--offset', str(offset)]
    code, res = child_json(argv + (['--length', str(length)] if length else []), self.log, CHILD_TIMEOUT, name)
    if code or not res or not res.get('ok'):
      raise StepFail(f'reading {name} failed', {'result': res, 'exit': code})
    return bytes.fromhex(res['data']), res

  def info(self, dev):
    text = self.read(dev, 'INFO_UF2.TXT')[0].decode(errors='replace')
    want = {'UF2 Bootloader': self.f['version'], 'Board-ID:': self.f['boardId'], 'SoftDevice:': self.f['softdevice']}
    lines = text.splitlines()
    bad = {k: v for k, v in want.items() if not any(l.strip() == f'{k} {v}' for l in lines)}
    if bad:
      raise StepFail(f'INFO_UF2.TXT does not name {bad}', {'info': lines})
    return lines

  def copy(self, dev, uf2, workdir):
    path = Path(workdir) / 'app.uf2'
    path.write_bytes(uf2)
    code, out = spawn(['mcopy', '-o', '-i', dev, path, '::APP.UF2'], self.log, 60)
    if code:
      raise StepFail('mcopy failed', {'tail': out.strip().splitlines()[-5:]})
    code, res = child_json(['flush', '--dev', dev], self.log, CHILD_TIMEOUT, 'flush')
    if code not in (0, FLUSH_GONE):
      raise StepFail('flush failed', {'result': res})
    wait_for(lambda: not self.present(), LEAVE_TIMEOUT, 'the drive leaving after the copy')
    return {'flush': 'flushed' if code == 0 else 'device gone before flush'}


# ------------------------------------------------------------------ the battery

class Battery:
  def __init__(self, board, f, workdir, log, fail_at=None):
    self.b, self.f, self.dir, self.log, self.fail_at = board, f, Path(workdir), log, fail_at
    self.probe, self.drive = Probe(board, workdir, log), Drive(board, f, log)
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
    """The app the copy wrote runs after the bootloader's own reset: no probe reset first. Two samples: thread mode,
    PC in the counting loop, r0 advancing; then the drive stays away."""
    fx = fixture(self.f, nonce)
    stays_away(self.drive.present, QUIET, 'the UF2 drive came back instead of the app')
    first = self.probe.regs()
    time.sleep(APP_SAMPLE_MS / 1000)
    second = self.probe.regs()
    loop = range(fx['handler'], fx['handler'] + 4)
    for s in (first, second):
      if s['ipsr'] or s['pc'] not in loop:
        raise StepFail('the app is not counting in thread mode', {'samples': [first, second], 'loop': hex(fx['handler'])})
    if (second['r0'] - first['r0']) & 0xFFFFFFFF == 0:
      raise StepFail('r0 did not advance', {'samples': [first, second]})
    return {'samples': [{k: hex(v) for k, v in s.items()} for s in (first, second)]}

  def reenter(self, nonce):
    fx = fixture(self.f, nonce)
    ev = self.probe.reenter()
    dev = self.drive.wait()
    blob, res = self.drive.read(dev, 'CURRENT.UF2', *self.current_span(len(fx['image'])))
    got = uf2_payload(blob, fx['start'], len(fx['image']))
    if got != fx['image']:
      raise StepFail('CURRENT.UF2 does not read back the copied app', {'read': len(got)})
    return {**ev, 'method': 'gpregret', 'readBack': len(got)}

  def current_span(self, length):
    """(offset, length) of CURRENT.UF2 holding [appStart, appStart + length): one 512-byte block per 256 bytes of flash
    from USER_FLASH_START = MBR_SIZE (src/usb/uf2/ghostfat.c, uf2cfg.h)."""
    return (self.f['appStart'] - MBR_SIZE) // 256 * 512, length // 256 * 512

  def battery(self):
    a, b = os.urandom(16), os.urandom(16)
    self.step('install', self.install)
    self.step('dfu', self.dfu)
    self.step('copy', lambda: self.copy(a))
    self.step('app-ran', lambda: self.app_ran(a))
    self.step('reenter', lambda: self.reenter(a))
    self.step('copy-b', lambda: self.copy(b))
    self.step('app-ran-b', lambda: self.app_ran(b))

  def restore(self):
    """The candidate installed again and the benign fixture copied through it and running."""
    if not self.touched:
      return {'pass': True, 'state': 'untouched', 'evidence': {}}
    ev, nonce = {}, b'restore'.ljust(16, b'\0')
    try:
      ev['install'] = self.probe.install(self.f)
      ev['dfu'] = self.dfu()
      ev['copy'] = self.copy(nonce)
      ev['appRan'] = self.app_ran(nonce)
      return {'pass': True, 'state': 'fixture running', 'evidence': ev}
    except StepFail as e:
      return {'pass': False, 'state': 'unknown', 'detail': str(e), 'evidence': {**ev, 'failed': e.evidence}}


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


def prove_hold(board, reason):
  helper = lock_helper()
  rec = helper.read_record(board['lock'])
  if not rec or rec.get('reason') != reason or not helper.holder_alive(rec):
    raise Refused(f'{board["lock"]} is not held with reason {reason!r}: take it first (hil_lock.py hold)')


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
            'heldInProcess': False}

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
    if not shutil.which('mcopy'):
      raise Refused('mcopy (mtools) must be on PATH')
  except Refused as e:
    row['detail'] = result['error'] = str(e)
    return finish(2)

  fh, held = take_lock(board, run_dir, args.lock_wait)
  if fh is None:
    row.update(held)
    return finish(1)
  bat = Battery(board, f, run_dir, log, args.fail_at)
  code = 1
  try:
    users = probe_users(board)
    if users:
      row.update(probeUsers=users, detail='another JLinkExe uses the probe')
    else:
      row['ran'] = True
      try:
        bat.battery()
        row['pass'] = True
      except StepFail as e:
        row['detail'] = f'{bat.steps[-1]["step"]}: {e}'
      row['cleanup'] = bat.restore()
      code = 0 if row['pass'] and row['cleanup']['pass'] else 1
  finally:
    row['steps'] = bat.steps
    if not row['cleanup']['pass']:
      try:
        token = lock_helper().hand_to_cleanup_holder({board['lock']: fh}, REASON, detail=row['cleanup'].get('detail'))
        row['cleanup'].update(heldAfter=True, holder={'token': token})
      except Exception as e:
        row['cleanup'].update(heldAfter=True, holder={'error': str(e)})
      fh.close()
      code = 4
    else:
      lock_helper().release_board_lock(fh)
  result['pass'] = code == 0
  return finish(code)


def cmd_manual(args, argv):
  """install or restore under the caller's own hold."""
  workdir = Path(args.run_root or ROOT / 'cmake-build' / 'hil') / f'{args.cmd}-{time.strftime("%Y%m%d-%H%M%S")}'
  workdir.mkdir(parents=True)
  try:
    board = load_board(args.roster, args.board)
    prove_hold(board, args.under_hold)
    _, rrow, copies = admit(args.build_record, args.board, args.roster, workdir)
    f = facts(args.board, rrow, copies, workdir)
    if probe_users(board):
      raise Refused('another JLinkExe uses the probe')
  except Refused as e:
    print(json.dumps({'pass': False, 'error': str(e)}))
    return 2
  bat = Battery(board, f, workdir, workdir / 'hil.log')
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
      p.add_argument('--fail-at', choices=STEPS)
    else:
      p.add_argument('--under-hold', required=True, metavar='REASON')
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
