#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for .claude/skills/hil-contract/scripts/hil_test.py and hil_run_report.py without hardware: the image and UF2
mechanics, the FAT reader, roster and record refusals, the J-Link transcripts it judges and the run's verdicts."""
import contextlib
import fcntl
import importlib.util
import io
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / '.claude/skills/hil-contract/scripts'
sys.path.insert(0, str(SCRIPTS))


def load(name):
  spec = importlib.util.spec_from_file_location(name, SCRIPTS / f'{name}.py')
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


ht = load('hil_test')
rr = load('hil_run_report')
FACTS = {'appStart': 0x26000, 'ramEnd': 0x20040000, 'family': 0xADA52840}


def ghostfat(files):
  """A FAT16 volume laid out as src/usb/uf2/ghostfat.c lays it: 1 reserved sector, 2 FATs, 64 root entries, one
  sector per cluster, each file contiguous from cluster 2."""
  spf, roots = 2, 64
  bs = bytearray(512)
  struct.pack_into('<HBHBHHBH', bs, 11, 512, 1, 1, 2, roots, 0, 0xF8, spf)
  root, data, cluster = bytearray(roots * 32), bytearray(), 2
  for i, (name, content) in enumerate(files.items()):
    stem, ext = name.split('.')
    root[i * 32:i * 32 + 11] = f'{stem:<8}{ext:<3}'.encode()
    struct.pack_into('<HI', root, i * 32 + 26, cluster, len(content))
    sectors = -(-len(content) // 512)
    data += content + b'\0' * (sectors * 512 - len(content))
    cluster += sectors
  return bytes(bs) + bytes(2 * spf * 512) + bytes(root) + bytes(data)


def fat_read(dev, name, offset=0, length=None):
  buf = io.StringIO()
  argv = ['fat-read', '--dev', dev, '--file', name, '--offset', str(offset)] + (['--length', str(length)] if length else [])
  with contextlib.redirect_stdout(buf):
    code = ht.main(argv)
  return code, json.loads(buf.getvalue())


class Images(unittest.TestCase):
  def test_parse_hex_and_runs(self):
    text = ':020000040001F9\n:04000000AABBCCDDEE\n:040010001122334442\n:00000001FF\n'
    with tempfile.NamedTemporaryFile('w', suffix='.hex') as f:
      f.write(text)
      f.flush()
      data = ht.parse_hex(f.name)
    self.assertEqual(ht.runs(data), [(0x10000, bytes.fromhex('AABBCCDD')), (0x10010, bytes.fromhex('11223344'))])

  def test_parse_hex_refuses_a_bad_checksum(self):
    with tempfile.NamedTemporaryFile('w', suffix='.hex') as f:
      f.write(':04000000AABBCCDD00\n')
      f.flush()
      with self.assertRaises(ht.Refused):
        ht.parse_hex(f.name)

  def test_fixture_vectors_and_loop(self):
    fx = ht.fixture(FACTS, b'a' * 16)
    sp, reset = struct.unpack_from('<2I', fx['image'])
    self.assertEqual((sp, reset), (0x20040000, (0x26000 + 64) | 1))
    self.assertEqual(struct.unpack_from('<I', fx['image'], 8)[0], fx['fault'] | 1)
    # ldr r1, [pc, #4]; adds r0, #1; b loop; b .; the marker
    self.assertEqual(fx['image'][64:72], bytes.fromhex('01490130fde7fee7'))
    self.assertEqual(struct.unpack_from('<I', fx['image'], 72)[0], fx['marker'])
    self.assertEqual((fx['loop'], fx['fault']), (0x26042, 0x26046))
    self.assertEqual(len(fx['image']) % 256, 0)

  def test_fixtures_of_different_nonces_differ_in_their_marker(self):
    a, b = ht.fixture(FACTS, b'a' * 16), ht.fixture(FACTS, b'b' * 16)
    self.assertEqual(a['image'][:72], b['image'][:72])
    self.assertNotEqual(a['marker'], b['marker'])

  def test_uf2_round_trip(self):
    img = ht.fixture(FACTS, b'x' * 16)['image']
    uf2 = ht.make_uf2(img, 0x26000, 0xADA52840)
    self.assertEqual(len(uf2), len(img) * 2)
    m0, m1, flags, addr, size, idx, n, fam = struct.unpack_from('<8I', uf2, 512)
    self.assertEqual((m0, m1, flags, addr, size, idx, n, fam),
                     (*ht.UF2_MAGIC[:2], 0x2000, 0x26100, 256, 1, len(img) // 256, 0xADA52840))
    self.assertEqual(struct.unpack_from('<I', uf2, 1020)[0], ht.UF2_MAGIC[2])
    self.assertEqual(ht.uf2_payload(uf2, 0x26000, len(img)), img)
    self.assertEqual(ht.uf2_payload(uf2, 0x26100, 256), img[256:512])

  def test_current_span_is_the_app_blocks_of_current_uf2(self):
    bat = ht.Battery.__new__(ht.Battery)
    bat.f = FACTS
    self.assertEqual(bat.current_span(4096), ((0x26000 - 0x1000) // 256 * 512, 8192))


class FatRead(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.NamedTemporaryFile(suffix='.img')
    self.info = b'UF2 Bootloader 0.9.2\r\nModel: x\r\nBoard-ID: nRF52840-pca10056-v1\r\n'
    self.current = bytes(range(256)) * 8
    self.tmp.write(ghostfat({'INFO_UF2.TXT': self.info, 'INDEX.HTM': b'<html>', 'CURRENT.UF2': self.current}))
    self.tmp.flush()

  def tearDown(self):
    self.tmp.close()

  def test_reads_a_file_and_a_cluster_aligned_slice(self):
    code, res = fat_read(self.tmp.name, 'INFO_UF2.TXT')
    self.assertEqual((code, bytes.fromhex(res['data'])), (0, self.info))
    self.assertEqual(res['entries'], ['CURRENT.UF2', 'INDEX.HTM', 'INFO_UF2.TXT'])
    code, res = fat_read(self.tmp.name, 'CURRENT.UF2', 512, 1024)
    self.assertEqual((code, bytes.fromhex(res['data'])), (0, self.current[512:1536]))

  def test_refuses_a_missing_file_and_an_unaligned_offset(self):
    self.assertEqual(fat_read(self.tmp.name, 'NOPE.TXT')[0], 1)
    code, res = fat_read(self.tmp.name, 'CURRENT.UF2', 100)
    self.assertEqual(code, 1)
    self.assertIn('cluster boundary', res['error'])


REGISTRY_ENTRY = {'name': 'pca10056', 'uid': '2467EB236E0E4F2D', 'probe': '683944004', 'nrf52': {'reentry': 'gpregret'}}


class Roster(unittest.TestCase):
  def board(self, entry, resolved=None):
    resolved = resolved or {**entry, 'flasher': {'name': 'jlink', 'args': '-device nrf52840_xxaa', 'uid': '683944004'}}
    reg = mock.Mock(resolve=mock.Mock(return_value=resolved), lock_name=mock.Mock(return_value='nrf52840dk'))
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f, mock.patch.object(ht, 'registry', return_value=reg):
      json.dump({'boards': [entry]}, f)
      f.flush()
      return ht.load_board(f.name, entry['name'])

  def test_resolves_device_lock_and_reentry(self):
    b = self.board(REGISTRY_ENTRY)
    self.assertEqual((b['device'], b['lock'], b['uid'], b['reentry']),
                     ('nrf52840_xxaa', 'nrf52840dk', '2467EB236E0E4F2D', 'gpregret'))

  def test_refuses_an_unknown_reentry_and_a_non_jlink_flasher(self):
    with self.assertRaises(ht.Refused):
      self.board({**REGISTRY_ENTRY, 'nrf52': {'reentry': 'double-reset'}})
    with self.assertRaises(ht.Refused):
      self.board(REGISTRY_ENTRY, {**REGISTRY_ENTRY, 'flasher': {'name': 'openocd', 'args': '-f x', 'uid': '1'}})

  def test_the_committed_roster_names_the_dk(self):
    entry = json.loads((ROOT / 'test/hil/ci.json').read_text())['boards'][0]
    self.assertEqual((entry['name'], entry['probe']), ('pca10056', '683944004'))
    self.assertEqual(self.board(entry)['base']['file'], 'update-pca10056_bootloader-0.11.0_nosd.uf2')

  def test_refuses_a_malformed_self_update_pin(self):
    for base in ('0.11.0', {'version': '0.11.0', 'file': 'x.uf2'},
                 {'version': '0.11.0', 'file': '../x.uf2', 'sha256': 'ab'}):
      with self.subTest(base=base), self.assertRaisesRegex(ht.Refused, 'selfUpdateBase'):
        self.board({**REGISTRY_ENTRY, 'nrf52': {'reentry': 'gpregret', 'selfUpdateBase': base}})

  def test_admits_only_the_pinned_base_bytes(self):
    with tempfile.TemporaryDirectory() as d:
      src, run = Path(d) / 'src', Path(d) / 'run'
      src.mkdir(), run.mkdir()
      (src / 'b.uf2').write_bytes(b'base')
      pin = {'version': '0.11.0', 'file': 'b.uf2', 'sha256': ht.hashlib.sha256(b'base').hexdigest()}
      got = ht.admit_base({'name': 'pca10056', 'base': pin}, src, run)
      self.assertEqual((got['path'], Path(got['path']).read_bytes()), (str(run / 'b.uf2'), b'base'))
      with self.assertRaisesRegex(ht.Refused, 'not the pinned'):
        ht.admit_base({'name': 'pca10056', 'base': {**pin, 'sha256': '0' * 64}}, src, run)
      with self.assertRaisesRegex(ht.Refused, 'pins no'):
        ht.admit_base({'name': 'pca10056', 'base': None}, src, run)


class Admit(unittest.TestCase):
  def test_refuses_a_record_that_is_not_a_hil_record(self):
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      json.dump({'pass': True, 'boards': [{'board': 'pca10056', 'status': 'ok'}], 'input': {}}, f)
      f.flush()
      with self.assertRaisesRegex(ht.Refused, 'not a passing HIL record'):
        ht.admit(f.name, 'pca10056', ROOT / 'test/hil/ci.json', '/nonexistent')

  def record(self, profile='default', **row):
    keys, profiles = dict.fromkeys(ht.ARTIFACTS, 'x'), ht.build().hil_profiles()
    config = {**ht.DEFAULT_CONFIG, 'BOARD': 'pca10056', **dict(d.split('=', 1) for d in profiles[profile])}
    row = {'board': 'pca10056', 'profile': profile, 'defines': profiles[profile], 'status': 'ok', 'artifacts': keys,
           'sha256': keys, 'config': config, **row}
    key = {'sha256': ht.build().sha256(ht.build().SIGNED_FW_TEST_KEY)}
    return {'pass': True, 'hilRecord': {'profiles': profiles, 'signedFwKey': key},
            'input': {'dirty': True, 'head': 'x'}, 'boards': [row]}

  def admit(self, rec, profile='default'):
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      json.dump(rec, f)
      f.flush()
      ht.admit(f.name, 'pca10056', ROOT / 'test/hil/ci.json', '/nonexistent', profile)

  def test_refuses_a_record_without_input(self):
    for inp in ({}, {'input': None}, {'input': []}):
      rec = {k: v for k, v in self.record().items() if k != 'input'}
      with self.subTest(inp=inp), self.assertRaisesRegex(ht.Refused, 'no input record'):
        self.admit({**rec, **inp})

  def test_refuses_a_dirty_record(self):
    with self.assertRaisesRegex(ht.Refused, 'rebuild on a clean HEAD'):
      self.admit(self.record())

  def test_refuses_a_non_default_configuration_or_incomplete_artifacts(self):
    cfg = {**ht.DEFAULT_CONFIG, 'BOARD': 'pca10056'}
    for bad in ({'config': {**cfg, 'SIGNED_FW': 'ON'}}, {'config': {**cfg, 'CMAKE_BUILD_TYPE': 'Debug'}},
                {'config': {**cfg, 'BOARD': 'pca10100'}}):
      with self.subTest(bad=bad), self.assertRaisesRegex(ht.Refused, "not profile default's configuration"):
        self.admit(self.record(**bad))
    with self.assertRaisesRegex(ht.Refused, "not profile default's configuration"):
      self.admit({**self.record(), 'defines': ['DUALBANK_FW=ON']})
    with self.assertRaisesRegex(ht.Refused, 'artifacts'):
      self.admit(self.record(artifacts={}))

  def test_a_profile_binds_its_row_its_options_and_the_test_key(self):
    rec = self.record('signed')
    with self.assertRaisesRegex(ht.Refused, 'rebuild on a clean HEAD'):   # every binding before it held
      self.admit(rec, 'signed')
    with self.assertRaisesRegex(ht.Refused, 'not a passing HIL record of pca10056 in profile signed'):
      self.admit(self.record('dualbank'), 'signed')
    other_key = dict(rec, hilRecord=dict(rec['hilRecord'], signedFwKey={'sha256': '0' * 64}))
    stale = dict(rec, boards=[dict(rec['boards'][0], defines=['SIGNED_FW=ON'])])
    for bad in (other_key, stale):
      with self.subTest(bad=bad['hilRecord']['signedFwKey']), self.assertRaisesRegex(ht.Refused, 'rebuild'):
        self.admit(bad, 'signed')
    wrong = self.record('dualbank', config={**ht.DEFAULT_CONFIG, 'BOARD': 'pca10056'})
    with self.assertRaisesRegex(ht.Refused, "not profile dualbank's configuration"):
      self.admit(wrong, 'dualbank')

  def test_spans(self):
    good = {a: 0 for a in (0, 0x1000, 0xF4000)}
    good.update(zip(range(0x10001014, 0x10001018), (0x00, 0x40, 0x0F, 0x00)))
    ht.check_spans(good, 0x100000, 0x26000, 0xF4000, 'm.hex')
    for bad, why in (({0x40000000: 0}, 'outside flash'), ({0x26000: 0}, 'app region'), ({0x10001000: 0}, 'outside'),
                     ({0x10001015: 0x41}, 'NRFFW')):
      with self.subTest(why=why), self.assertRaisesRegex(ht.Refused, why):
        ht.check_spans({**good, **bad}, 0x100000, 0x26000, 0xF4000, 'm.hex')
    no_sd = {a: v for a, v in good.items() if a != 0x1000}
    with self.assertRaisesRegex(ht.Refused, 'want runs'):
      ht.check_spans(no_sd, 0x100000, 0x26000, 0xF4000, 'm.hex')

  def test_facts_from_this_checkout(self):
    f = {'name': 's140', 'version': '6.1.1'}
    with tempfile.TemporaryDirectory() as d:
      merged = Path(d) / 'm.hex'
      merged.write_text(':10000000' + b'UF2 Bootloader 1.2\r\n'[:16].hex().upper() + '00\n:00000001FF\n')
      self.assertRaises(ht.Refused, ht.facts, 'pca10056', {'softdevice': f}, {'merged': merged}, d)   # bad checksum
      def record(addr, data):
        raw = bytes([len(data)]) + (addr & 0xFFFF).to_bytes(2, 'big') + b'\0' + data
        return ':' + (raw + bytes([-sum(raw) & 0xFF])).hex().upper()

      def ela(upper):
        raw = bytes([2, 0, 0, 4]) + upper.to_bytes(2, 'big')
        return ':' + (raw + bytes([-sum(raw) & 0xFF])).hex().upper()
      body = b'xxUF2 Bootloader 1.2.3\r\nModel'
      lines = [record(0, b'\0'), record(0x1000, b'\0'), ela(0xF), record(0x4000, body[:16]),
               record(0x4010, body[16:]), ela(0x1000), record(0x1014, (0xF4000).to_bytes(4, 'little'))]
      merged.write_text('\n'.join(lines) + '\n:00000001FF\n')
      got = ht.facts('pca10056', {'softdevice': f}, {'merged': merged, 'update': Path(d) / 'u.uf2',
                                                     'zip': Path(d) / 'p.zip'}, d)
    self.assertEqual((got['appStart'], got['ramEnd'], got['vid'], got['pid'], got['cdcPid'], got['boardId'],
                      got['version'], got['softdevice'], got['family']),
                     (0x26000, 0x20040000, 0x239A, 0x00DA, 0x00DA, 'nRF52840-pca10056-v1', '1.2.3', 'S140 6.1.1',
                      0xADA52840))

  def test_facts_refuse_a_board_without_usb(self):
    with self.assertRaisesRegex(ht.Refused, 'no USB'):
      ht.facts('feather_nrf52832', {'softdevice': {}}, {}, '/nonexistent')


def transcript(*chunks):
  return 'Connecting...\nCortex-M4 identified.\n' + ''.join(f'J-Link>{c}\n{o}' for c, o in chunks)


class Probe(unittest.TestCase):
  BOARD = {'flasher': {'name': 'jlink', 'args': '-device nrf52840_xxaa', 'uid': '683944004'}, 'device': 'nrf52840_xxaa'}

  def probe(self, out, cleanup='done'):
    """A Probe whose JLinkExe prints `out`, judged by the installed agentrc's hil_flash; `self.argv` its command."""
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    p = ht.Probe(self.BOARD, tmp, ht.Tools(tmp, Path(tmp) / 'hil.log'))

    def fake(cmd, cwd=None, *, timeout, settle, state_dir, quiet):
      self.argv = cmd
      return mock.Mock(returncode=0, stdout=out, stderr='', cleanup=cleanup, receipt=None)
    return p, mock.patch.object(ht.process(), 'run_tracked', side_effect=fake)

  def test_pin_resets_never_connect_and_need_every_echo(self):
    probe_line = 'J-Link connection not established yet but required for command.\nConnecting to J-Link ...O.K.\n'
    pulses = [('r0', probe_line), ('sleep 20', 'Sleep(20)\n'), ('r1', ''), ('sleep 150', 'Sleep(150)\n'),
              ('r0', ''), ('sleep 20', 'Sleep(20)\n'), ('r1', '')]
    p, patch = self.probe(transcript(*pulses, ('exit', 'Script processing completed.\n')))
    with patch:
      self.assertEqual(p.pin_resets([150]), {'pulses': 2, 'gapsMs': [150]})
    self.assertNotIn('-autoconnect', self.argv)
    p, patch = self.probe(transcript(*pulses[:3], ('exit', 'Script processing completed.\n')))
    with patch, self.assertRaisesRegex(ht.StepFail, 'nRESET pulses were not all issued'):
      p.pin_resets([150])
    p, patch = self.probe(transcript(('r0', '****** Error: no probe\n'), *pulses[1:3], ('exit', '')))
    with patch, self.assertRaisesRegex(ht.StepFail, r"failed: \['r0'\]"):
      p.pin_resets([])

  def test_a_tool_not_proven_ended_stops_with_its_receipt(self):
    p, patch = self.probe('', cleanup='unknown')
    with patch, self.assertRaises(ht.NotEnded):
      p.samples()
    self.assertEqual(p.tools.receipts, [{'cmd': 'JLinkExe', 'cleanup': 'unknown', 'receipt': None}])

  def test_a_receipt_is_kept_when_the_log_cannot_be_written(self):
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    tools = ht.Tools(tmp, Path(tmp) / 'missing-dir' / 'hil.log')
    pending = mock.Mock(returncode=0, stdout='', stderr='', cleanup='pending', receipt={'token': 'r'})
    with mock.patch.object(ht.process(), 'run_tracked', return_value=pending), self.assertRaises(OSError):
      tools.run(['JLinkExe'], timeout=1)
    self.assertEqual(tools.receipts, [{'cmd': 'JLinkExe', 'cleanup': 'pending', 'receipt': {'token': 'r'}}])

  @staticmethod
  def regs(pc, r0):
    return (f'PC = {pc:08X}, CycleCnt = 00000000\nR0 = {r0:08X}, R1 = 00000000\n'
            'XPSR = 61000000: APSR = nZCvq, EPSR = 01000000, IPSR = 000 (NoException)\n')

  def test_samples_parse_each_regs_output_of_one_session(self):
    out = transcript(('h', 'PC = 00026040, CycleCnt = 00000000\n'), ('regs', self.regs(0x26040, 0x2A)), ('g', ''),
                     ('sleep 200', 'Sleep(200)\n'), ('h', 'PC = 00026042, CycleCnt = 00000000\n'),
                     ('regs', self.regs(0x26042, 0x99)), ('g', ''), ('exit', 'Script processing completed.\n'))
    p, patch = self.probe(out)
    with patch:
      self.assertEqual(p.samples(), [{'pc': 0x26040, 'r0': 0x2A, 'r1': 0, 'ipsr': 0},
                                     {'pc': 0x26042, 'r0': 0x99, 'r1': 0, 'ipsr': 0}])

  def test_a_failed_command_is_a_step_failure(self):
    out = transcript(('h', 'PC = 00026040, CycleCnt = 00000000\n'), ('regs', self.regs(0x26040, 0x2A)), ('g', ''),
                     ('sleep 200', 'Sleep(200)\n'),
                     ('h', 'J-Link connection not established yet but required for command.\n'),
                     ('regs', ''), ('g', ''), ('exit', 'Script processing completed.\n'))
    p, patch = self.probe(out)
    with patch, self.assertRaisesRegex(ht.StepFail, 'J-Link session failed'):
      p.samples()

  def test_install_requires_nvmc_ready_after_eraseall(self):
    f = {'hex': '/x.hex', 'runs': []}
    cmds = [('r', 'Reset delay: 0 ms\nReset: Reset device via AIRCR.SYSRESETREQ.\n'),
            ('w4 4001E504 00000002', 'Writing 00000002 -> 4001E504\n'),
            ('w4 4001E50C 00000001', 'Writing 00000001 -> 4001E50C\n'), ('sleep 1000', 'Sleep(1000)\n'),
            ('mem32 4001E400 1', '4001E400 = 00000000 \n'), ('w4 4001E504 00000000', 'Writing 00000000 -> 4001E504\n'),
            ('loadfile /x.hex', 'O.K.\n'), ('h', 'PC = 00026040, CycleCnt = 00000000\n'),
            ('r', 'Reset delay: 0 ms\nReset: Reset device via AIRCR.SYSRESETREQ.\n'), ('g', ''),
            ('exit', 'Script processing completed.\n')]
    p, patch = self.probe(transcript(*cmds))
    with patch, self.assertRaisesRegex(ht.StepFail, 'NVMC not ready'):
      p.install(f)


class BatteryRun(unittest.TestCase):
  def battery(self):
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    bat = ht.Battery({'flasher': {'uid': '1'}, 'device': 'd', 'uid': 'u'}, FACTS, tmp, Path(tmp) / 'hil.log')
    return bat

  def serial_run(self, out, drive=None):
    bat = self.battery()
    bat.f = {**FACTS, 'vid': 0x239A, 'cdcPid': 0xDA}
    calls = []

    def spawn(argv, timeout):
      calls.append([str(a) for a in argv])
      return 0, out if 'serial' in argv else ''
    with mock.patch.object(bat.probe, 'reenter', return_value={'gpregret': '0000004E'}) as reenter, \
         mock.patch.object(ht, 'usb_device', return_value='1-1'), \
         mock.patch.object(ht, 'cdc_tty', return_value='/dev/ttyACM9'), \
         mock.patch.object(ht.os, 'access', return_value=True), \
         mock.patch.object(ht, 'block_device', return_value=drive), \
         mock.patch.object(bat.tools, 'spawn', side_effect=spawn):
      ev = bat.serial(b'c' * 16)
    reenter.assert_called_once_with(ht.DFU_MAGIC_SERIAL_ONLY_RESET)
    return ev, calls

  def test_serial_dfu_packages_for_the_adafruit_device_type_and_needs_programmed(self):
    ev, calls = self.serial_run('Upgrading target\nDevice programmed.\n')
    self.assertEqual(ev['tty'], '/dev/ttyACM9')
    self.assertEqual(calls[0][:5], ['adafruit-nrfutil', 'dfu', 'genpkg', '--dev-type', '0x0052'])
    self.assertEqual(calls[1][-7:], ['--package', calls[0][-1], '-p', '/dev/ttyACM9', '-b', '115200', '--singlebank'])
    with self.assertRaisesRegex(ht.StepFail, 'did not program'):
      self.serial_run('Failed to upgrade target. Error is: timeout\n')
    with self.assertRaisesRegex(ht.StepFail, 'enumerated a drive'):
      self.serial_run('Device programmed.\n', drive='/dev/sdz')

  def test_update_enters_dfu_when_the_app_runs_and_checks_the_version(self):
    bat = self.battery()
    with tempfile.NamedTemporaryFile(suffix='.uf2') as uf2, \
         mock.patch.object(bat.drive, 'present', return_value=None), \
         mock.patch.object(bat.probe, 'reenter', return_value={}) as reenter, \
         mock.patch.object(bat.drive, 'wait', return_value='/dev/sdz'), \
         mock.patch.object(bat.drive, 'read', return_value=(b'UF2 Bootloader 1.0\r\n', {})), \
         mock.patch.object(bat.drive, 'copy', return_value={}) as copy, \
         mock.patch.object(bat, 'app_ran', return_value={}) as app_ran, \
         mock.patch.object(bat.drive, 'info', return_value=['UF2 Bootloader 0.11.0']) as info:
      ev = bat.update(uf2.name, '0.11.0', b'c' * 16)
    self.assertEqual((reenter.call_count, ev['before']), (2, 'UF2 Bootloader 1.0'))
    copy.assert_called_once()
    app_ran.assert_called_once_with(b'c' * 16)
    info.assert_called_once_with('/dev/sdz', '0.11.0')

  def test_double_reset_runs_both_controls_before_the_window(self):
    bat, calls = self.battery(), []
    with mock.patch.object(bat.probe, 'pin_resets', side_effect=lambda g: calls.append(('pulse', g)) or {}), \
         mock.patch.object(bat, 'app_ran', side_effect=lambda n: calls.append(('app', n)) or {}), \
         mock.patch.object(bat.drive, 'wait', side_effect=lambda: calls.append(('drive',)) or '/dev/sdz'), \
         mock.patch.object(bat.drive, 'info', return_value=['UF2 Bootloader x']):
      ev = bat.double_reset(b'b' * 16)
    self.assertEqual(calls, [('pulse', []), ('app', b'b' * 16), ('pulse', [1500]), ('app', b'b' * 16),
                             ('pulse', [150]), ('drive',)])
    self.assertEqual(ev['inWindow']['info'], ['UF2 Bootloader x'])

  def test_app_ran_rejects_another_copys_marker(self):
    bat = self.battery()
    a, b = ht.fixture(FACTS, b'a' * 16), ht.fixture(FACTS, b'b' * 16)
    samples = iter([{'pc': a['loop'], 'r0': 1, 'r1': a['marker'], 'ipsr': 0},
                    {'pc': a['loop'], 'r0': 9, 'r1': a['marker'], 'ipsr': 0}] * 2)
    with mock.patch.object(ht, 'stays_away'), mock.patch.object(ht.time, 'sleep'), \
         mock.patch.object(bat.probe, 'samples', side_effect=lambda: [next(samples), next(samples)]):
      bat.app_ran(b'a' * 16)
      with self.assertRaisesRegex(ht.StepFail, "copy's app"):
        bat.app_ran(b'b' * 16)
    self.assertNotEqual(a['marker'], b['marker'])

  def test_a_profiles_cleanup_reinstalls_the_default_image(self):
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    home = {**FACTS, 'version': 'default'}
    bat = ht.Battery({'flasher': {'uid': '1'}, 'device': 'd', 'uid': 'u'}, {**FACTS, 'version': 'signed'}, tmp,
                     Path(tmp) / 'hil.log', profile='signed', home=home)
    bat.touched = True
    with mock.patch.object(bat.probe, 'install', return_value={}) as install, \
         mock.patch.object(ht.Battery, 'dfu', return_value={}), \
         mock.patch.object(ht.Battery, 'copy', return_value={}), \
         mock.patch.object(ht.Battery, 'app_ran', return_value={}):
      self.assertTrue(bat.restore()['pass'])
    install.assert_called_once_with(home)
    self.assertIs(bat.drive.f, home)

  def test_a_step_turns_an_unexpected_error_into_its_failure(self):
    bat = self.battery()
    with self.assertRaisesRegex(ht.StepFail, 'OSError: boom'):
      bat.step('dfu', lambda: (_ for _ in ()).throw(OSError('boom')))
    self.assertEqual((bat.steps[0]['step'], bat.steps[0]['pass']), ('dfu', False))


PREVIOUS = {'event': 'failed', 'id': 'f1', 'at': '2026-11-08T10:00:00+0700', 'how': 'ended-failed',
            'reason': 'hil_test.py', 'detail': 'earlier run', 'receipts': []}
BOARD = {'name': 'pca10056', 'reentry': 'gpregret', 'uid': 'u', 'lock': 'nrf52840dk', 'flasher': {'uid': '1'},
         'device': 'd'}


def mock_helper(**kw):
  """The lock helper mocked, its failure_report and exceptions the agentrc's."""
  real = ht.lock_helper()
  return mock.Mock(failure_report=real.failure_report, ReleaseRefused=real.ReleaseRefused,
                   resolve_restored=mock.Mock(return_value=[]), end_failed=mock.Mock(return_value={}), **kw)


class RunLock(unittest.TestCase):
  """cmd_run's lock lifecycle (agentrc#474): released after a proven cleanup, else ended through end_failed."""
  def run_cmd(self, fail=None, restored=None, claim=None, end_errors=None):
    """cmd_run with the battery raising `fail` after install touched the board, then the restore returning `restored`
    (None: the real one); claim and unclaim stubbed (Claim tests them)."""
    fh = mock.Mock(cleanup_failures=[PREVIOUS])
    helper = mock_helper(take_board_lock=mock.Mock(return_value=fh))
    if end_errors:
      helper.end_failed.return_value = end_errors

    def battery(bat, base):
      bat.touched = True
      if callable(fail):
        fail(bat)
      elif fail:
        raise fail
    restores = []
    patches = [mock.patch.object(ht, 'lock_helper', return_value=helper),
               mock.patch.object(ht, 'load_board', return_value=BOARD),
               mock.patch.object(ht, 'admit', return_value=({'input': {'head': 'h'}}, {'sha256': {'merged': 'm'}}, {})),
               mock.patch.object(ht, 'facts', return_value={**FACTS, 'version': 'v', 'vid': 1, 'pid': 2}),
               mock.patch.object(ht, 'publish'),
               mock.patch.object(ht, 'admit_base', return_value={'version': 'v', 'file': 'f', 'sha256': 's'}),
               mock.patch.object(ht.shutil, 'which', return_value='/bin/true'),
               mock.patch.object(ht.Battery, 'battery', battery),
               mock.patch.object(ht, 'claim',
                                 side_effect=claim or (lambda b, ops, found: ({'receipt': 'me'}, found, ''))),
               mock.patch.object(ht, 'unclaim', return_value='')]
    if restored is not None:
      patches.append(mock.patch.object(ht.Battery, 'restore', lambda bat: restores.append(1) or restored))
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out, \
         contextlib.ExitStack() as stack:
      for patch in patches:
        stack.enter_context(patch)
      code = ht.main(['run', '-b', 'pca10056', '--build-record', 'r', '--base-dir', d, '--run-root', d, 'roster'])
    res = json.loads(out.getvalue().splitlines()[-1])
    return code, helper, fh, restores, res

  def test_a_passing_run_resolves_the_failures_it_found_then_releases(self):
    code, helper, fh, restores, res = self.run_cmd(restored={'pass': True, 'state': 'fixture running'})
    row = res['results'][0]
    self.assertEqual((code, res['exitCode'], restores), (0, 0, [1]))
    self.assertEqual(row['previousCleanupFailure'], PREVIOUS)
    names = [c[0] for c in helper.mock_calls]
    self.assertLess(names.index('resolve_restored'), names.index('release_board_lock'))
    self.assertEqual(helper.resolve_restored.call_args[0], ('nrf52840dk', [PREVIOUS]))
    helper.release_board_lock.assert_called_once_with(fh)
    helper.end_failed.assert_not_called()
    self.assertTrue(row['cleanup']['released'])
    self.assertNotIn('heldInProcess', res)

  def test_a_failed_restore_ends_the_lock_through_end_failed(self):
    def step_fails(bat):
      bat.step('dfu', lambda: (_ for _ in ()).throw(ht.StepFail('no drive')))
    code, helper, fh, _, res = self.run_cmd(step_fails,
                                           {'pass': False, 'state': 'unknown', 'detail': 'restore: no drive'})
    self.assertEqual((code, res['results'][0]['detail']), (4, 'dfu: no drive'))
    helper.end_failed.assert_called_once_with({'nrf52840dk': fh}, 'hil_test.py', detail='restore: no drive',
                                              receipts=[])
    helper.release_board_lock.assert_not_called()
    helper.resolve_restored.assert_not_called()
    cleanup = res['results'][0]['cleanup']
    self.assertEqual((cleanup['failureLogged'], cleanup['released']), (True, True))
    self.assertEqual(rr.verdict(code, res, 'pca10056')[0], 'blocker')
    self.assertIn("logged in the board's cleanup log; lock released", rr.verdict(code, res, 'pca10056')[1])

  def test_an_error_or_interrupt_outside_a_step_skips_the_restore_and_logs(self):
    for fail, detail in ((OSError('boom'), 'OSError: boom'), (KeyboardInterrupt(), 'KeyboardInterrupt: ')):
      with self.subTest(fail=detail):
        code, helper, _, restores, res = self.run_cmd(fail, {'pass': True, 'state': 'x'})
        row = res['results'][0]
        self.assertEqual((code, restores, row['detail'], row['cleanup']['state']), (4, [], detail, 'unknown'))
        self.assertEqual(helper.end_failed.call_args.kwargs['detail'], 'interrupted before cleanup')
        helper.release_board_lock.assert_not_called()

  def test_tools_not_proven_ended_go_to_the_cleanup_log_with_their_receipts(self):
    def not_ended(bat):
      bat.tools.receipts += [{'cmd': 'JLinkExe', 'cleanup': 'pending', 'receipt': {'token': 'r'}},
                             {'cmd': 'mcopy', 'cleanup': 'unknown', 'receipt': None}]
      raise ht.NotEnded('JLinkExe: a tool is not proven ended')
    code, helper, _, _, res = self.run_cmd(not_ended)   # the real restore refuses to touch the board
    self.assertEqual((code, res['results'][0]['cleanup']['state']), (4, 'unknown'))
    self.assertEqual(helper.end_failed.call_args.kwargs['receipts'],
                     [{'token': 'r'}, {'unrecheckable': 'mcopy: cleanup unknown'}])

  def test_a_cleanup_log_that_cannot_be_written_is_reported(self):
    errors = {'nrf52840dk': ['cleanup log not written: OSError(28)']}
    code, _, _, _, res = self.run_cmd(OSError('boom'), end_errors=errors)
    cleanup = res['results'][0]['cleanup']
    self.assertEqual((code, cleanup['failureLogged'], cleanup['endErrors']), (4, False, errors['nrf52840dk']))
    self.assertIn('NOT logged', rr.verdict(code, res, 'pca10056')[1])

  def test_a_board_in_use_is_not_run_and_its_lock_released(self):
    def in_use(occupied):
      def claim(board, ops, found):
        raise ht.InUse('x', found + [dict(PREVIOUS, id='f2')], occupied)
      return claim
    code, helper, fh, restores, res = self.run_cmd(claim=in_use(None))
    row = res['results'][0]
    self.assertEqual((code, row['ran'], row['locked'], restores), (1, False, True, []))
    self.assertEqual((row['openCleanupFailures'], row['previousCleanupFailure']['id']), (2, 'f2'))
    self.assertEqual(rr.verdict(code, res, 'pca10056')[0], 'not-run')
    helper.release_board_lock.assert_called_once_with(fh)
    helper.resolve_restored.assert_not_called()
    code, _, _, _, res = self.run_cmd(claim=in_use(['123 JLinkExe -USB 1']))
    self.assertEqual((res['results'][0]['occupiedBy'], rr.verdict(code, res, 'pca10056')[0]),
                     (['123 JLinkExe -USB 1'], 'blocker'))


class Manual(unittest.TestCase):
  """install and restore under the caller's hold, or restore under an older helper's cleanup holder: they only log."""
  def manual(self, cmd, outcome, hold=('--under-hold', 'dbg'), receipts=()):
    helper = mock_helper(open_cleanup_failures=mock.Mock(return_value=[PREVIOUS]))

    def restore(bat):
      bat.tools.receipts += list(receipts)
      if isinstance(outcome, BaseException):
        raise outcome
      return outcome
    holder = {'pid': 7, 'token': 'tok'}
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out, \
         mock.patch.object(ht, 'lock_helper', return_value=helper), \
         mock.patch.object(ht, 'load_board', return_value=BOARD), mock.patch.object(ht, 'prove_hold'), \
         mock.patch.object(ht, 'prove_cleanup_holder', return_value=holder), \
         mock.patch.object(ht, 'admit', return_value=({}, {}, {})), \
         mock.patch.object(ht, 'facts', return_value=FACTS), \
         mock.patch.object(ht, 'claim', side_effect=lambda b, ops, found: ({'receipt': 'me'}, found, '')), \
         mock.patch.object(ht, 'unclaim', return_value='') as unclaim, \
         mock.patch.object(ht.Battery, 'restore', restore), mock.patch.object(ht.Battery, 'install', restore):
      code = ht.main([cmd, '-b', 'pca10056', '--build-record', 'r', '--run-root', d, *hold, 'roster'])
    unclaim.assert_called_once()
    helper.end_failed.assert_not_called()
    helper.release_board_lock.assert_not_called()
    return code, helper, json.loads(out.getvalue().splitlines()[-1])

  def test_a_passing_restore_resolves_the_open_failures(self):
    code, helper, out = self.manual('restore', {'pass': True, 'state': 'fixture running'})
    self.assertEqual((code, out['previousCleanupFailure']), (0, PREVIOUS))
    self.assertEqual(helper.resolve_restored.call_args[0], ('nrf52840dk', [PREVIOUS]))
    helper.log_cleanup_failure.assert_not_called()

  def test_a_failed_restore_is_logged_never_ending_the_hold(self):
    code, helper, out = self.manual('restore', {'pass': False, 'state': 'unknown', 'detail': 'no drive'},
                                    receipts=[{'cmd': 'mcopy', 'cleanup': 'pending', 'receipt': {'token': 'r'}}])
    self.assertEqual((code, out['failureLogged']), (4, True))
    helper.log_cleanup_failure.assert_called_once_with('nrf52840dk', 'hil_test.py', detail='restore failed: no drive',
                                                       task=None, receipts=[{'token': 'r'}])
    helper.resolve_restored.assert_not_called()

  def test_a_failed_install_is_logged_only_with_a_tool_not_proven_ended(self):
    code, helper, _ = self.manual('install', ht.StepFail('NVMC not ready'))
    self.assertEqual(code, 4)
    helper.log_cleanup_failure.assert_not_called()
    code, helper, _ = self.manual('install', ht.NotEnded('JLinkExe: not ended'),
                                  receipts=[{'cmd': 'JLinkExe', 'cleanup': 'pending', 'receipt': {'token': 'r'}}])
    self.assertEqual(helper.log_cleanup_failure.call_args.kwargs['receipts'], [{'token': 'r'}])

  def test_a_restore_under_an_older_helpers_cleanup_holder_ends_it_once_passed(self):
    code, helper, out = self.manual('restore', {'pass': True, 'state': 'fixture running'},
                                    hold=('--recover-holder', '7'))
    self.assertEqual((code, out['recovered']), (0, {'holder': 7, 'released': True}))
    helper.release_token.assert_called_once_with('tok')
    code, helper, out = self.manual('restore', {'pass': False, 'state': 'unknown', 'detail': 'x'},
                                    hold=('--recover-holder', '7'))
    self.assertEqual(code, 4)
    helper.release_token.assert_not_called()


class Verdict(unittest.TestCase):
  def row(self, **kw):
    return {'results': [{'board': 'pca10056', 'ran': True, 'pass': False, 'steps': [], 'cleanup': {'pass': True}, **kw}]}

  def test_pass_fail_not_run_and_blocker(self):
    ok = self.row(**{'pass': True, 'steps': [{'step': 'install', 'pass': True}]})
    self.assertEqual(rr.verdict(0, ok, 'pca10056')[0], 'pass')
    failed = self.row(steps=[{'step': 'install', 'pass': True}, {'step': 'dfu', 'pass': False, 'detail': 'x'}])
    self.assertEqual(rr.verdict(1, failed, 'pca10056'), ('fail', 'dfu: x'))
    self.assertEqual(rr.verdict(1, self.row(ran=False, detail='locked'), 'pca10056')[0], 'not-run')
    self.assertEqual(rr.verdict(4, failed, 'pca10056')[0], 'blocker')
    unclean = self.row(steps=failed['results'][0]['steps'], cleanup={'pass': False, 'state': 'unknown'})
    self.assertEqual(rr.verdict(1, unclean, 'pca10056')[0], 'blocker')
    self.assertEqual(rr.verdict(1, None, 'pca10056')[0], 'blocker')

  def test_render_names_each_step(self):
    core = mock.Mock(table=lambda h, rows, center: '\n'.join(' | '.join(r) for r in rows))
    body = self.row(steps=[{'step': 'install', 'pass': True}, {'step': 'dfu', 'pass': False, 'detail': 'no drive'}])
    text = rr.render(core, {'pass': False, **body})
    self.assertIn('install | ✅', text)
    self.assertIn('dfu | ❌ | no drive', text)

  def test_render_names_a_failed_cleanup_and_an_earlier_one(self):
    core = mock.Mock(table=lambda h, rows, center: '')
    body = self.row(cleanup={'pass': False, 'state': 'unknown', 'failureLogged': True, 'released': True},
                    previousCleanupFailure=PREVIOUS, openCleanupFailures=2)
    text = rr.render(core, {'pass': False, **body})
    self.assertIn("Cleanup failed (unknown): logged in the board's cleanup log; lock released", text)
    self.assertIn('earlier failed cleanup on the board (2 open): 2026-11-08T10:00:00+0700 hil_test.py: earlier run',
                  text)


class Cli(unittest.TestCase):
  def test_run_refuses_a_board_pending_commissioning(self):
    buf = io.StringIO()
    entry = {**REGISTRY_ENTRY, 'nrf52': {}}
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(buf), \
         mock.patch.object(ht, 'load_board', return_value={'reentry': None, 'uid': 'x'}), \
         mock.patch.object(ht, 'publish'):
      roster = Path(d) / 'r.json'
      roster.write_text(json.dumps({'boards': [entry]}))
      code = ht.main(['run', '-b', 'pca10056', '--build-record', '/nonexistent', '--base-dir', d, '--run-root', d,
                      str(roster)])
    res = json.loads(buf.getvalue().splitlines()[-1])
    self.assertEqual((code, res['exitCode'], res['results'][0]['ran']), (2, 2, False))
    self.assertIn('pending commissioning', res['error'])

  def test_run_refuses_a_step_its_profile_does_not_run(self):
    buf = io.StringIO()
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(buf), \
         mock.patch.object(ht, 'load_board', return_value={'reentry': 'gpregret', 'uid': 'x'}), \
         mock.patch.object(ht, 'PROFILE_STEPS', {'default': ('install',), 'other': ('dfu',)}), \
         mock.patch.object(ht, 'publish'):
      code = ht.main(['run', '-b', 'pca10056', '--build-record', '/nonexistent', '--base-dir', d, '--run-root', d,
                      '--profile', 'other', '--fail-at', 'install', 'r.json'])
    res = json.loads(buf.getvalue().splitlines()[-1])
    self.assertEqual((code, res['results'][0]['profile']), (2, 'other'))
    self.assertIn('--fail-at install is no step of profile other', res['error'])


  def test_recovery_refuses_a_lock_not_held_by_that_cleanup_holder(self):
    board = {'lock': 'nrf52840dk'}
    for rec in (None, {'role': 'cleanup-holder', 'pid': 2, 'token': 't', 'locks': ['nrf52840dk']},
                {'role': 'hold', 'pid': 1, 'token': 't', 'locks': ['nrf52840dk']},
                {'role': 'cleanup-holder', 'pid': 1, 'token': 't', 'locks': ['nrf52840dk', 'other']}):
      helper = mock.Mock(read_record=mock.Mock(return_value=rec), holder_alive=mock.Mock(return_value=True),
                         is_cleanup_holder=lambda r: isinstance(r, dict) and r.get('role') == 'cleanup-holder')
      with mock.patch.object(ht, 'lock_helper', return_value=helper), self.assertRaises(ht.Refused):
        ht.prove_cleanup_holder(board, 1)
    rec = {'role': 'cleanup-holder', 'pid': 1, 'token': 't', 'locks': ['nrf52840dk'], 'receipts': [{'token': 'r'}]}
    helper.read_record.return_value = rec
    proc = mock.Mock(recheck=mock.Mock(return_value='done'))
    with mock.patch.object(ht, 'lock_helper', return_value=helper), mock.patch.object(ht, 'process', return_value=proc):
      with mock.patch.object(ht, 'tools_left', return_value=[]):
        self.assertEqual(ht.prove_cleanup_holder(board, 1), rec)
      with mock.patch.object(ht, 'tools_left', return_value=['123 mcopy -i /dev/sde']), \
           self.assertRaisesRegex(ht.Refused, 'still on the host'):
        ht.prove_cleanup_holder(board, 1)
      proc.recheck.return_value = 'pending'
      with mock.patch.object(ht, 'tools_left', return_value=[]), self.assertRaisesRegex(ht.Refused, 'not yet done'):
        ht.prove_cleanup_holder(board, 1)


class Claim(unittest.TestCase):
  """The claim sequence against the agentrc's real PendingSessions, cleanup log and recheck_dir, in a private lock dir;
  an earlier invocation's run_tracked files are planted, a live one's lock held by this test as its supervisor would."""
  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(self.tmp))
    self.helper = ht.lock_helper()
    for patch in (mock.patch.object(self.helper, 'BOARD_LOCK_DIR', str(Path(self.tmp) / 'locks')),
                  mock.patch.object(ht, 'probe_users', return_value=[]),
                  mock.patch.object(ht, 'CLAIM_SETTLE', 0.3)):
      patch.start()
      self.addCleanup(patch.stop)
    self.board = {'name': 'pca10056', 'lock': 'nrf52840dk', 'flasher': {'uid': '683944004'}}
    self.tools = ht.Tools(Path(self.tmp) / 'run', Path(self.tmp) / 'hil.log')
    self.earlier = ht.Tools(Path(self.tmp) / 'earlier', Path(self.tmp) / 'hil.log').dir

  def listed(self):
    return [e['receipt'] for e in self.helper.PendingSessions('nrf52840dk', ht.CLAIM_KIND).read()]

  def other(self, pid, start, ops=None):
    receipt = {'ops': ops or self.earlier, 'pid': pid, 'start': start}
    self.helper.PendingSessions('nrf52840dk', ht.CLAIM_KIND).add([{'receipt': receipt, 'cmd': 'hil_test.py'}])
    return receipt

  def plant(self, ops, token, live):
    """A run_tracked operation's files in `ops`: done, or `live` (its lock held, as a running supervisor holds it)."""
    base = Path(ops, f'hil_op_{token}')
    lock = Path(f'{base}.lock')
    lock.touch(mode=0o600)
    Path(f'{base}.log').touch(mode=0o600)
    st = lock.stat()
    Path(f'{base}.json').write_text(json.dumps({'token': token, 'lock': [st.st_dev, st.st_ino], 'pid': 1,
                                                'cleanup': 'pending' if live else 'done'}))
    Path(f'{base}.json').chmod(0o600)
    if live:
      fd = ht.os.open(lock, ht.os.O_RDONLY)
      fcntl.flock(fd, fcntl.LOCK_EX)
      self.addCleanup(ht.os.close, fd)
      return fd
    return None

  def end(self, ops, token, fd):
    """The planted live operation ends: its record says done, its lock is let go."""
    path = Path(ops, f'hil_op_{token}.json')
    path.write_text(json.dumps({**json.loads(path.read_text()), 'cleanup': 'done'}))
    fcntl.flock(fd, fcntl.LOCK_UN)

  def test_a_claim_names_this_invocation_and_unclaim_removes_only_it(self):
    entry, found, why = ht.claim(self.board, self.tools.dir, [])
    self.assertEqual((found, why), ([], ''))
    self.assertEqual(entry['receipt'], {'ops': ht.os.path.realpath(self.tools.dir), 'pid': ht.os.getpid(),
                                        'start': self.helper.proc_start(ht.os.getpid())})
    later = self.other(ht.os.getppid(), self.helper.proc_start(ht.os.getppid()))
    self.assertEqual(ht.unclaim(self.board, entry, self.tools), '')
    self.assertEqual(self.listed(), [later])

  def test_a_live_invocation_refuses_and_this_claim_goes(self):
    parent = self.other(ht.os.getppid(), self.helper.proc_start(ht.os.getppid()))
    with self.assertRaisesRegex(ht.InUse, 'another invocation on the board is running') as e:
      ht.claim(self.board, self.tools.dir, [])
    self.assertIsNone(e.exception.occupied)
    self.assertEqual(self.listed(), [parent])

  def test_a_dead_invocation_whose_tools_ended_is_removed_and_tidied(self):
    self.other(999999999, '1')
    self.plant(self.earlier, 'abc', live=False)
    entry, found, _ = ht.claim(self.board, self.tools.dir, [])
    self.assertEqual((self.listed(), found), ([entry['receipt']], []))
    self.assertEqual(ht.os.listdir(self.earlier), [])
    self.assertEqual(self.helper.open_cleanup_failures('nrf52840dk'), [])

  def test_a_dead_invocations_unproven_tool_is_logged_settled_then_resolved(self):
    dead = self.other(999999999, '1')
    fd = self.plant(self.earlier, 'abc', live=True)
    with self.assertRaisesRegex(ht.InUse, r'still on the board after 0.3s: \[.receipt abc') as e:
      ht.claim(self.board, self.tools.dir, [])
    (failure,) = self.helper.open_cleanup_failures('nrf52840dk')
    self.assertEqual((failure['task'], [r['token'] for r in failure['receipts']]), (dead, ['abc']))
    self.assertEqual((e.exception.found, self.listed()), ([failure], []))
    # the next attempt, its take reporting the failure, does not find that claim: it refuses nothing, its files kept
    found = self.helper.open_cleanup_failures('nrf52840dk')
    entry, found, _ = ht.claim(self.board, self.tools.dir, found)
    self.assertEqual((found, self.listed()), ([failure], [entry['receipt']]))
    self.assertEqual(ht.unclaim(self.board, entry, self.tools), '')
    self.assertTrue(Path(self.earlier, 'hil_op_abc.lock').exists())
    (why,) = self.helper.resolve_restored('nrf52840dk', found)
    self.assertIn(f'{failure["id"]} stays open', why)
    self.end(self.earlier, 'abc', fd)
    self.assertEqual(self.helper.resolve_restored('nrf52840dk', found), [])
    self.assertEqual(self.helper.open_cleanup_failures('nrf52840dk'), [])

  def test_a_failure_logged_since_the_take_is_returned_with_what_the_log_cannot_hold(self):
    self.helper.log_cleanup_failure('nrf52840dk', 'hil_test.py', detail='a previous command under the hold')
    left = {'how': 'found-left', 'detail': 'x', 'noteError': 'cleanup log not written: x'}   # the take's, unlogged
    restored_since = dict(PREVIOUS, id='restored-since')
    _, found, _ = ht.claim(self.board, self.tools.dir, [left, restored_since])
    (logged,) = self.helper.open_cleanup_failures('nrf52840dk')
    self.assertEqual(found, [logged, left])
    self.assertEqual(self.helper.resolve_restored('nrf52840dk', found), [])
    self.assertEqual(self.helper.open_cleanup_failures('nrf52840dk'), [])
    unreadable = [{'unavailable': 'cleanup log unreadable: x'}]
    with mock.patch.object(self.helper, 'open_cleanup_failures', return_value=unreadable):
      _, found, _ = ht.claim(self.board, self.tools.dir, [left, restored_since])
    self.assertEqual(found, [left, restored_since, *unreadable])

  def test_a_recovery_keeps_the_receipts_its_legacy_holder_releases_on(self):
    self.other(999999999, '1')
    self.plant(self.earlier, 'legacy', live=False)
    base = str(Path(self.earlier, 'hil_op_legacy'))
    holder = {'role': 'cleanup-holder', 'pid': 7, 'start': '1', 'token': 'tok', 'locks': ['nrf52840dk'],
              'receipts': [{'token': 'legacy', 'supervisor': None, 'lock': f'{base}.lock', 'record': f'{base}.json',
                            'log': f'{base}.log'}]}
    predicate = []

    def release_token(token):   # what the real one rechecks before it signals the holder
      predicate.append(self.helper._release_blocker(holder['receipts']))
      return 7, ['nrf52840dk']
    with contextlib.redirect_stdout(io.StringIO()) as out, \
         mock.patch.object(self.helper, 'read_record', return_value=holder), \
         mock.patch.object(self.helper, 'holder_alive', return_value=True), \
         mock.patch.object(self.helper, 'release_token', side_effect=release_token), \
         mock.patch.object(ht, 'tools_left', return_value=[]), \
         mock.patch.object(ht, 'load_board', return_value=BOARD), \
         mock.patch.object(ht, 'admit', return_value=({}, {}, {})), \
         mock.patch.object(ht, 'facts', return_value=FACTS), \
         mock.patch.object(ht.Battery, 'restore', lambda bat: {'pass': True, 'state': 'fixture running'}):
      code = ht.main(['restore', '-b', 'pca10056', '--build-record', 'r', '--run-root', self.tmp,
                      '--recover-holder', '7', 'roster'])
    self.assertEqual((code, predicate), (0, ['']))
    self.assertEqual(json.loads(out.getvalue().splitlines()[-1])['recovered'], {'holder': 7, 'released': True})
    self.assertEqual(self.listed(), [])

  def test_a_probe_user_refuses_without_a_wait(self):
    with mock.patch.object(ht, 'probe_users', return_value=['123 JLinkExe -USB 683944004']), \
         self.assertRaisesRegex(ht.InUse, 'after 0s') as e:
      ht.claim(self.board, self.tools.dir, [])
    self.assertEqual((e.exception.occupied, self.listed()), (['123 JLinkExe -USB 683944004'], []))

  def test_an_ops_dir_gone_is_logged_once(self):
    self.other(999999999, '1', str(Path(self.tmp) / 'gone'))
    with self.assertRaises(ht.InUse):
      ht.claim(self.board, self.tools.dir, [])
    (failure,) = self.helper.open_cleanup_failures('nrf52840dk')
    self.assertIn('unrecheckable', failure['receipts'][0])
    entry, _, _ = ht.claim(self.board, self.tools.dir, [])
    self.assertEqual(self.listed(), [entry['receipt']])

  def test_a_claim_that_cannot_be_written_refuses_before_any_tool(self):
    with mock.patch.object(self.helper.PendingSessions, 'claim', side_effect=OSError(28, 'No space left')), \
         self.assertRaisesRegex(ht.Refused, 'could not name this invocation') as e:
      ht.claim(self.board, self.tools.dir, [])
    self.assertNotIsInstance(e.exception, ht.InUse)

  def test_an_unclean_end_keeps_its_claim(self):
    entry, _, _ = ht.claim(self.board, self.tools.dir, [])
    self.plant(self.tools.dir, 'mine', live=True)
    self.tools.receipts.append({'cmd': 'JLinkExe', 'cleanup': 'pending', 'receipt': {'token': 'mine'}})
    self.assertEqual(ht.unclaim(self.board, entry, self.tools), 'claim kept: its tools recheck pending')
    self.assertEqual(self.listed(), [entry['receipt']])


class Tools(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(self.tmp))

  def test_the_ops_dir_is_private_whatever_the_umask(self):
    old = ht.os.umask(0o002)
    try:
      tools = ht.Tools(Path(self.tmp) / 'run', Path(self.tmp) / 'hil.log')
    finally:
      ht.os.umask(old)
    self.assertEqual((ht.os.stat(tools.dir).st_mode & 0o777, (Path(self.tmp) / 'run').stat().st_mode & 0o777),
                     (0o700, 0o755))

  def test_a_supervisor_that_never_started_leaves_nothing(self):
    tools = ht.Tools(self.tmp, Path(self.tmp) / 'hil.log')
    with mock.patch.object(ht.process().subprocess, 'Popen', side_effect=OSError(11, 'EAGAIN')), \
         self.assertRaises(OSError):
      tools.run(['JLinkExe'], timeout=1)
    self.assertEqual((ht.os.listdir(tools.dir), tools.receipts), ([], []))


class DriveAbsence(unittest.TestCase):
  """Drive.absent through the agentrc's usb_observe over a planted sysfs tree."""
  def drive(self, serial):
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    if serial is not False:
      d = Path(tmp, '1-1')
      d.mkdir()
      (d / 'idVendor').write_text('239a\n')
      (d / 'idProduct').write_text('00da\n')
      if serial is not None:
        (d / 'serial').write_text(serial + '\n')
    patch = mock.patch.object(ht.usb(), 'SYS_USB_DEVICES', tmp)
    patch.start()
    self.addCleanup(patch.stop)
    return ht.Drive({'uid': '2467EB236E0E4F2D'}, {'vid': 0x239A, 'pid': 0x00DA}, None)

  def test_absent_only_when_no_match_could_be_the_board(self):
    for serial, absent in ((False, True), ('2467eb236e0e4f2d', False), ('0000000000000001', True), (None, False)):
      with self.subTest(serial=serial):
        self.assertEqual(self.drive(serial).absent(), absent)

  def test_a_copy_waits_for_the_device_to_leave_not_only_the_drive(self):
    drive = self.drive(None)   # the drive node gone, the device's serial unreadable
    drive.tools = mock.Mock(spawn=mock.Mock(return_value=(0, '')), child_json=mock.Mock(return_value=(0, {})))
    with tempfile.TemporaryDirectory() as d, mock.patch.object(drive, 'present', return_value=None), \
         mock.patch.object(ht, 'LEAVE_TIMEOUT', 0.3), self.assertRaisesRegex(ht.StepFail, 'leaving after the copy'):
      drive.copy('/dev/sdz', b'uf2', d)


if __name__ == '__main__':
  unittest.main()
