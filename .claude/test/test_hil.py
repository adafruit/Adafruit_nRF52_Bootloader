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
    self.assertEqual(fx['image'][64:70], bytes.fromhex('0130fde7fee7'))   # adds r0,#1; b handler; b .
    self.assertEqual(len(fx['image']) % 256, 0)

  def test_fixtures_of_different_nonces_differ_only_past_the_code(self):
    a, b = ht.fixture(FACTS, b'a' * 16), ht.fixture(FACTS, b'b' * 16)
    self.assertEqual(a['image'][:72], b['image'][:72])
    self.assertNotEqual(a['image'], b['image'])

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

  def test_refuses_a_dirty_record(self):
    rec = {'pass': True, 'hilRecord': {}, 'input': {'dirty': True, 'head': 'x'},
           'boards': [{'board': 'pca10056', 'status': 'ok', 'sha256': {}}]}
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      json.dump(rec, f)
      f.flush()
      with self.assertRaisesRegex(ht.Refused, 'rebuild on a clean HEAD'):
        ht.admit(f.name, 'pca10056', ROOT / 'test/hil/ci.json', '/nonexistent')

  def test_facts_from_this_checkout(self):
    f = {'name': 's140', 'version': '6.1.1'}
    with tempfile.TemporaryDirectory() as d:
      merged = Path(d) / 'm.hex'
      merged.write_text(':10000000' + b'UF2 Bootloader 1.2\r\n'[:16].hex().upper() + '00\n:00000001FF\n')
      self.assertRaises(ht.Refused, ht.facts, 'pca10056', {'softdevice': f}, {'merged': merged}, d)   # bad checksum
      body = b'xxUF2 Bootloader 1.2.3\r\nModel'
      recs = [body[i:i + 16] for i in range(0, len(body), 16)]
      lines = []
      for i, r in enumerate(recs):
        raw = bytes([len(r), 0, i * 16, 0]) + r
        lines.append(':' + (raw + bytes([-sum(raw) & 0xFF])).hex().upper())
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

  def probe(self, out):
    """A Probe whose JLinkExe prints `out`, judged by the installed agentrc's hil_flash."""
    p = ht.Probe(self.BOARD, tempfile.gettempdir(), Path(tempfile.gettempdir()) / 'hil_test_unit.log')

    def fake(cmd, timeout, cwd=None, quiet=True):
      return mock.Mock(returncode=0, stdout=out, stderr='')
    return p, mock.patch.object(ht.process(), 'run_cmd', side_effect=fake)

  def test_regs_parses_pc_r0_ipsr(self):
    regs = ('PC = 00026040, CycleCnt = 00000000\nR0 = 0000002A, R1 = 00000000\n'
            'XPSR = 61000000: APSR = nZCvq, EPSR = 01000000, IPSR = 000 (NoException)\n')
    out = transcript(('h', 'PC = 00026040, CycleCnt = 00000000\n'), ('regs', regs), ('g', ''),
                     ('exit', 'Script processing completed.\n'))
    p, patch = self.probe(out)
    with patch:
      self.assertEqual(p.regs(), {'pc': 0x26040, 'r0': 0x2A, 'ipsr': 0})

  def test_a_failed_command_is_a_step_failure(self):
    out = transcript(('h', 'J-Link connection not established yet but required for command.\n'),
                     ('regs', ''), ('g', ''), ('exit', 'Script processing completed.\n'))
    p, patch = self.probe(out)
    with patch, self.assertRaisesRegex(ht.StepFail, 'J-Link session failed'):
      p.regs()

  def test_install_requires_nvmc_ready_after_eraseall(self):
    f = {'hex': '/x.hex', 'runs': []}
    cmds = [('r', 'Reset delay: 0 ms\nReset: Reset device via AIRCR.SYSRESETREQ.\n'), ('w4 4001E504 00000002', 'Writing 00000002 -> 4001E504\n'),
            ('w4 4001E50C 00000001', 'Writing 00000001 -> 4001E50C\n'), ('sleep 1000', 'Sleep(1000)\n'),
            ('mem32 4001E400 1', '4001E400 = 00000000 \n'), ('w4 4001E504 00000000', 'Writing 00000000 -> 4001E504\n'),
            ('loadfile /x.hex', 'O.K.\n'), ('exit', 'Script processing completed.\n')]
    p, patch = self.probe(transcript(*cmds))
    with patch, self.assertRaisesRegex(ht.StepFail, 'NVMC not ready'):
      p.install(f)


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
    rec = {'role': 'cleanup-holder', 'pid': 1, 'token': 't', 'locks': ['nrf52840dk']}
    helper.read_record.return_value = rec
    with mock.patch.object(ht, 'lock_helper', return_value=helper):
      self.assertEqual(ht.prove_cleanup_holder(board, 1), rec)


if __name__ == '__main__':
  unittest.main()
