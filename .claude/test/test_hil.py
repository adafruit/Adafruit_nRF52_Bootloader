#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for .claude/skills/hil-contract/scripts/hil_test.py and hil_run_report.py without hardware: the image and UF2
mechanics, the FAT reader, roster and record refusals, the J-Link transcripts it judges and the run's verdicts."""
import contextlib
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


class Admit(unittest.TestCase):
  def test_refuses_a_record_that_is_not_a_hil_record(self):
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      json.dump({'pass': True, 'boards': [{'board': 'pca10056', 'status': 'ok'}], 'input': {}}, f)
      f.flush()
      with self.assertRaisesRegex(ht.Refused, 'not a passing HIL record'):
        ht.admit(f.name, 'pca10056', ROOT / 'test/hil/ci.json', '/nonexistent')

  def record(self, **row):
    keys = dict.fromkeys(ht.ARTIFACTS, 'x')
    row = {'board': 'pca10056', 'status': 'ok', 'artifacts': keys, 'sha256': keys,
           'config': {**ht.DEFAULT_CONFIG, 'BOARD': 'pca10056'}, **row}
    return {'pass': True, 'hilRecord': {}, 'input': {'dirty': True, 'head': 'x'}, 'boards': [row]}

  def admit(self, rec):
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      json.dump(rec, f)
      f.flush()
      ht.admit(f.name, 'pca10056', ROOT / 'test/hil/ci.json', '/nonexistent')

  def test_refuses_a_dirty_record(self):
    with self.assertRaisesRegex(ht.Refused, 'rebuild on a clean HEAD'):
      self.admit(self.record())

  def test_refuses_a_non_default_configuration_or_incomplete_artifacts(self):
    cfg = {**ht.DEFAULT_CONFIG, 'BOARD': 'pca10056'}
    for bad in ({'config': {**cfg, 'SIGNED_FW': 'ON'}}, {'config': {**cfg, 'CMAKE_BUILD_TYPE': 'Debug'}},
                {'config': {**cfg, 'BOARD': 'pca10100'}}):
      with self.subTest(bad=bad), self.assertRaisesRegex(ht.Refused, 'not the default configuration'):
        self.admit(self.record(**bad))
    with self.assertRaisesRegex(ht.Refused, 'not the default configuration'):
      self.admit({**self.record(), 'defines': ['DUALBANK_FW=ON']})
    with self.assertRaisesRegex(ht.Refused, 'artifacts'):
      self.admit(self.record(artifacts={}))

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
      got = ht.facts('pca10056', {'softdevice': f}, {'merged': merged}, d)
    self.assertEqual((got['appStart'], got['ramEnd'], got['vid'], got['pid'], got['boardId'], got['version'],
                      got['softdevice'], got['family']),
                     (0x26000, 0x20040000, 0x239A, 0x00DA, 'nRF52840-pca10056-v1', '1.2.3', 'S140 6.1.1', 0xADA52840))

  def test_facts_refuse_a_board_without_usb(self):
    with self.assertRaisesRegex(ht.Refused, 'no USB'):
      ht.facts('feather_nrf52832', {'softdevice': {}}, {}, '/nonexistent')


def transcript(*chunks):
  return 'Connecting...\nCortex-M4 identified.\n' + ''.join(f'J-Link>{c}\n{o}' for c, o in chunks)


class Probe(unittest.TestCase):
  BOARD = {'flasher': {'name': 'jlink', 'args': '-device nrf52840_xxaa', 'uid': '683944004'}, 'device': 'nrf52840_xxaa'}

  def probe(self, out, cleanup='done'):
    """A Probe whose JLinkExe prints `out`, judged by the installed agentrc's hil_flash."""
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    p = ht.Probe(self.BOARD, tmp, ht.Tools(tmp, Path(tmp) / 'hil.log'))

    def fake(cmd, cwd=None, *, timeout, settle, state_dir, quiet):
      return mock.Mock(returncode=0, stdout=out, stderr='', cleanup=cleanup, receipt=None)
    return p, mock.patch.object(ht.process(), 'run_tracked', side_effect=fake)

  def test_a_tool_not_proven_ended_stops_with_its_receipt(self):
    p, patch = self.probe('', cleanup='unknown')
    with patch, self.assertRaises(ht.NotEnded):
      p.regs()
    self.assertEqual(p.tools.receipts, [{'cmd': 'JLinkExe', 'cleanup': 'unknown', 'receipt': None}])

  def test_a_receipt_is_kept_when_the_log_cannot_be_written(self):
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    tools = ht.Tools(tmp, Path(tmp) / 'missing-dir' / 'hil.log')
    pending = mock.Mock(returncode=0, stdout='', stderr='', cleanup='pending', receipt={'token': 'r'})
    with mock.patch.object(ht.process(), 'run_tracked', return_value=pending), self.assertRaises(OSError):
      tools.run(['JLinkExe'], timeout=1)
    self.assertEqual(tools.receipts, [{'cmd': 'JLinkExe', 'cleanup': 'pending', 'receipt': {'token': 'r'}}])

  def test_regs_parses_pc_r0_ipsr(self):
    regs = ('PC = 00026040, CycleCnt = 00000000\nR0 = 0000002A, R1 = 00000000\n'
            'XPSR = 61000000: APSR = nZCvq, EPSR = 01000000, IPSR = 000 (NoException)\n')
    out = transcript(('h', 'PC = 00026040, CycleCnt = 00000000\n'), ('regs', regs), ('g', ''),
                     ('exit', 'Script processing completed.\n'))
    p, patch = self.probe(out)
    with patch:
      self.assertEqual(p.regs(), {'pc': 0x26040, 'r0': 0x2A, 'r1': 0, 'ipsr': 0})

  def test_a_failed_command_is_a_step_failure(self):
    out = transcript(('h', 'J-Link connection not established yet but required for command.\n'),
                     ('regs', ''), ('g', ''), ('exit', 'Script processing completed.\n'))
    p, patch = self.probe(out)
    with patch, self.assertRaisesRegex(ht.StepFail, 'J-Link session failed'):
      p.regs()

  def test_install_requires_nvmc_ready_after_eraseall(self):
    f = {'hex': '/x.hex', 'runs': []}
    cmds = [('r', 'Reset delay: 0 ms\nReset: Reset device via AIRCR.SYSRESETREQ.\n'),
            ('w4 4001E504 00000002', 'Writing 00000002 -> 4001E504\n'),
            ('w4 4001E50C 00000001', 'Writing 00000001 -> 4001E50C\n'), ('sleep 1000', 'Sleep(1000)\n'),
            ('mem32 4001E400 1', '4001E400 = 00000000 \n'), ('w4 4001E504 00000000', 'Writing 00000000 -> 4001E504\n'),
            ('loadfile /x.hex', 'O.K.\n'), ('exit', 'Script processing completed.\n')]
    p, patch = self.probe(transcript(*cmds))
    with patch, self.assertRaisesRegex(ht.StepFail, 'NVMC not ready'):
      p.install(f)


class BatteryRun(unittest.TestCase):
  def battery(self):
    tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(tmp))
    bat = ht.Battery({'flasher': {'uid': '1'}, 'device': 'd', 'uid': 'u'}, FACTS, tmp, Path(tmp) / 'hil.log')
    return bat

  def test_app_ran_rejects_another_copys_marker(self):
    bat = self.battery()
    a, b = ht.fixture(FACTS, b'a' * 16), ht.fixture(FACTS, b'b' * 16)
    samples = iter([{'pc': a['loop'], 'r0': 1, 'r1': a['marker'], 'ipsr': 0},
                    {'pc': a['loop'], 'r0': 9, 'r1': a['marker'], 'ipsr': 0}] * 2)
    with mock.patch.object(ht, 'stays_away'), mock.patch.object(ht.time, 'sleep'), \
         mock.patch.object(bat.probe, 'regs', side_effect=lambda: next(samples)):
      bat.app_ran(b'a' * 16)
      with self.assertRaisesRegex(ht.StepFail, "copy's app"):
        bat.app_ran(b'b' * 16)
    self.assertNotEqual(a['marker'], b['marker'])

  def run_cmd(self, fail):
    """cmd_run with the battery raising `fail` after install touched the board; the lock helper mocked."""
    helper = mock.Mock(take_board_lock=mock.Mock(return_value=mock.Mock()),
                       hand_to_cleanup_holder=mock.Mock(return_value='tok'))
    helper.PendingSessions.return_value.add.return_value = []
    board = {'reentry': 'gpregret', 'uid': 'u', 'lock': 'nrf52840dk', 'flasher': {'uid': '1'}, 'device': 'd'}

    def battery(bat):
      bat.touched = True
      raise fail
    restored = []
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out, \
         mock.patch.object(ht, 'lock_helper', return_value=helper), \
         mock.patch.object(ht, 'load_board', return_value=board), \
         mock.patch.object(ht, 'admit', return_value=({'input': {'head': 'h'}}, {'sha256': {'merged': 'm'}}, {})), \
         mock.patch.object(ht, 'facts', return_value={**FACTS, 'version': 'v', 'vid': 1, 'pid': 2}), \
         mock.patch.object(ht, 'probe_users', return_value=[]), mock.patch.object(ht, 'publish'), \
         mock.patch.object(ht.Battery, 'battery', battery), \
         mock.patch.object(ht.Battery, 'restore', lambda bat: restored.append(1) or {'pass': True, 'state': 'x'}):
      try:
        code = ht.main(['run', '-b', 'pca10056', '--build-record', 'r', '--run-root', d, 'roster'])
      except BaseException as e:
        code = e
    return code, helper, restored, out.getvalue()

  def test_a_step_turns_an_unexpected_error_into_its_failure(self):
    bat = self.battery()
    with self.assertRaisesRegex(ht.StepFail, 'OSError: boom'):
      bat.step('dfu', lambda: (_ for _ in ()).throw(OSError('boom')))
    self.assertEqual((bat.steps[0]['step'], bat.steps[0]['pass']), ('dfu', False))

  def test_an_error_outside_a_step_hands_the_board_to_a_cleanup_holder(self):
    code, helper, restored, _ = self.run_cmd(OSError('boom'))
    self.assertIsInstance(code, OSError)
    self.assertEqual(restored, [])
    helper.hand_to_cleanup_holder.assert_called_once()
    helper.release_board_lock.assert_not_called()

  def test_an_interrupt_after_touching_hands_the_board_to_a_cleanup_holder(self):
    code, helper, restored, _ = self.run_cmd(KeyboardInterrupt())
    self.assertIsInstance(code, KeyboardInterrupt)
    self.assertEqual(restored, [])
    helper.hand_to_cleanup_holder.assert_called_once()
    helper.release_board_lock.assert_not_called()


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
    self.assertEqual(rr.verdict(1, None, 'pca10056')[0], 'blocker')

  def test_render_names_each_step(self):
    core = mock.Mock(table=lambda h, rows, center: '\n'.join(' | '.join(r) for r in rows))
    body = self.row(steps=[{'step': 'install', 'pass': True}, {'step': 'dfu', 'pass': False, 'detail': 'no drive'}])
    text = rr.render(core, {'pass': False, **body})
    self.assertIn('install | ✅', text)
    self.assertIn('dfu | ❌ | no drive', text)


class Cli(unittest.TestCase):
  def test_run_refuses_a_board_pending_commissioning(self):
    buf = io.StringIO()
    entry = {**REGISTRY_ENTRY, 'nrf52': {}}
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(buf), \
         mock.patch.object(ht, 'load_board', return_value={'reentry': None, 'uid': 'x'}), \
         mock.patch.object(ht, 'publish'):
      roster = Path(d) / 'r.json'
      roster.write_text(json.dumps({'boards': [entry]}))
      code = ht.main(['run', '-b', 'pca10056', '--build-record', '/nonexistent', '--run-root', d, str(roster)])
    res = json.loads(buf.getvalue().splitlines()[-1])
    self.assertEqual((code, res['exitCode'], res['results'][0]['ran']), (2, 2, False))
    self.assertIn('pending commissioning', res['error'])


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


class Pending(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(self.tmp))
    self.helper = ht.lock_helper()
    patch = mock.patch.object(self.helper, 'BOARD_LOCK_DIR', str(Path(self.tmp) / 'locks'))
    patch.start()
    self.addCleanup(patch.stop)
    self.board = {'lock': 'nrf52840dk'}
    self.ops = ht.Tools(self.tmp, Path(self.tmp) / 'hil.log').dir

  def mark(self):
    return self.helper.PendingSessions('nrf52840dk', ht.PENDING_KIND).read()

  def other(self, pid, start, ops):
    self.helper.PendingSessions('nrf52840dk', ht.PENDING_KIND).add(
      [{'receipt': {'ops': ops, 'pid': pid, 'start': start}, 'cmd': 'hil_test.py'}])

  def test_a_claim_names_the_invocation_by_absolute_ops_and_unmark_clears_it(self):
    with contextlib.chdir(self.tmp):
      ht.claim(self.board, 'ops')
    self.assertEqual(self.mark()[0]['receipt'], {'ops': ht.os.path.realpath(self.ops), 'pid': ht.os.getpid(),
                                                 'start': self.helper.proc_start(ht.os.getpid())})
    ht.unmark(self.board)
    self.assertEqual(self.mark(), [])

  def test_a_live_invocation_refuses_a_later_claim(self):
    parent = ht.os.getppid()
    self.other(parent, self.helper.proc_start(parent), self.ops)
    with self.assertRaisesRegex(ht.Refused, 'not proven ended'):
      ht.claim(self.board, self.ops)
    self.assertEqual(len(self.mark()), 2)   # both kept: the refused one ends with its process

  def test_a_dead_invocations_tool_left_unresolved_refuses_until_done(self):
    self.other(999999999, '1', self.ops)
    Path(self.ops, 'hil_op_abc.log').write_text('')
    with mock.patch.object(ht.process(), 'recheck', return_value='pending') as recheck:
      with self.assertRaisesRegex(ht.Refused, 'not proven ended'):
        ht.claim(self.board, self.ops)
      self.assertEqual(recheck.call_args[0][0]['lock'], str(Path(self.ops, 'hil_op_abc.lock')))
      recheck.return_value = 'done'
      ht.claim(self.board, self.ops)
      ht.unmark(self.board)
    self.assertEqual(self.mark(), [])
    self.assertFalse(Path(self.ops, 'hil_op_abc.log').exists())

  def test_an_ops_dir_gone_or_unlistable_is_not_ended(self):
    self.other(999999999, '1', str(Path(self.tmp) / 'gone'))
    with self.assertRaisesRegex(ht.Refused, 'not proven ended'):
      ht.claim(self.board, self.ops)

  def test_a_claim_that_cannot_be_written_refuses_before_any_tool(self):
    with mock.patch.object(self.helper.PendingSessions, 'add', side_effect=OSError(28, 'No space left')), \
         self.assertRaisesRegex(ht.Refused, 'could not name this invocation'):
      ht.claim(self.board, self.ops)

  def test_unmark_keeps_a_mark_it_cannot_clear(self):
    with mock.patch.object(self.helper.PendingSessions, 'clear', side_effect=PermissionError(1, 'not ours')):
      ht.unmark(self.board)

  def test_a_supervisor_that_never_started_leaves_no_log(self):
    tools = ht.Tools(self.tmp, Path(self.tmp) / 'hil.log')

    def no_supervisor(*a, **kw):
      Path(tools.dir, 'hil_op_abc.log').write_text('')
      e = OSError(11, 'Resource temporarily unavailable')
      e.cleanup, e.receipt = 'done', None
      raise e
    with mock.patch.object(ht.process(), 'run_tracked', side_effect=no_supervisor), self.assertRaises(OSError):
      tools.run(['JLinkExe'], timeout=1)
    self.assertEqual(ht.os.listdir(tools.dir), [])
    self.assertEqual(tools.receipts, [])


if __name__ == '__main__':
  unittest.main()
