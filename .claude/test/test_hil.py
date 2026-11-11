#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for .claude/skills/hil-contract/scripts/hil_test.py and hil_run_report.py without hardware: the image and UF2
mechanics, the FAT reader, roster and record refusals, the J-Link transcripts it judges and the run's verdicts."""
import contextlib
import importlib.util
import io
import json
import signal
import struct
import sys
import tempfile
import threading
import time
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


def mem32_line(addr, *words):
  """What J-Link prints for `mem32 <addr> <n>`: each word an int, or 16 bytes read as four little-endian words."""
  if len(words) == 1 and isinstance(words[0], bytes):
    words = struct.unpack('<4I', words[0])
  return f'{addr:08X} = ' + ' '.join(f'{w:08X}' for w in words) + ' \n'


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

  def test_bank_1_starts_half_the_app_region_up(self):
    # nRF52840, S140 6.1.1: (0xF4000 - 0x26000 - 10 * 4096) rounded to two pages, halved
    self.assertEqual(ht.bank_1_start('nrf52840', 0x26000, 0xF4000), 0x88000)
    with self.assertRaises(ht.Refused):
      ht.bank_1_start('nrf52999', 0x26000, 0xF4000)

  def test_repackage_replaces_each_init_packet(self):
    import zipfile
    with tempfile.TemporaryDirectory() as tmp:
      src, dst = Path(tmp) / 'a.zip', Path(tmp) / 'b.zip'
      manifest = {'manifest': {'application': {'bin_file': 'app.bin', 'dat_file': 'app.dat'}, 'dfu_version': 0.5}}
      with zipfile.ZipFile(src, 'w') as z:
        z.writestr('manifest.json', json.dumps(manifest))
        z.writestr('app.dat', bytes.fromhex('5200ffffffffffff0100feff65d6'))
        z.writestr('app.bin', b'img')
      ht.repackage(src, dst, ht.flip_crc)
      with zipfile.ZipFile(dst) as z:
        self.assertEqual(z.read('app.dat'), bytes.fromhex('5200ffffffffffff0100feff9a29'))
        self.assertEqual(z.read('app.bin'), b'img')

  def test_the_signing_key_gives_the_public_key_the_build_compiles_in(self):
    key = json.loads((ROOT / 'test/hil/signed-fw-test-key.json').read_text())
    self.assertEqual(ht.public_key(ht.signing_key()), bytes.fromhex(key['qx'] + key['qy']))

  def test_a_signed_init_packet_verifies_as_dfu_init_c_checks_it(self):
    dat, image, d = bytes.fromhex('5200ffffffffffff0100feff65d6'), b'\x01' * 4352, ht.signing_key()
    pkt = ht.signed_init_packet(dat, image, d)
    self.assertEqual(len(pkt), 12 + 40 + 64)   # init fields, ECDS id, length, SHA-256; r || s
    self.assertEqual(pkt[:12], dat[:-2])
    self.assertEqual(struct.unpack_from('<II', pkt, 12), (2, len(image)))
    self.assertEqual(pkt[20:52], __import__('hashlib').sha256(image).digest())
    try:
      from cryptography.hazmat.primitives import hashes
      from cryptography.hazmat.primitives.asymmetric import ec
      from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature
    except ImportError:
      self.skipTest('no cryptography to cross-check the signature')
    pub = ec.EllipticCurvePublicNumbers(*(int.from_bytes(ht.public_key(d)[i:i + 32], 'big') for i in (0, 32)),
                                        ec.SECP256R1()).public_key()
    r, s_ = (int.from_bytes(pkt[52 + i:84 + i], 'big') for i in (0, 32))
    pub.verify(encode_dss_signature(r, s_), pkt[:52], ec.ECDSA(hashes.SHA256()))

  def test_tampering_after_signing_changes_the_image_not_its_signed_hash(self):
    dat, image = bytes.fromhex('5200ffffffffffff0100feff65d6'), b'\x01' * 256
    pkt, changed = ht.tampered_after_signing(ht.signing_key())(dat, image)
    self.assertNotEqual(changed, image)
    self.assertEqual(len(changed), len(image))
    self.assertEqual(pkt[20:52], __import__('hashlib').sha256(image).digest())

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
    self.assertTrue(got['drive'])
    self.assertEqual(got['bank1'], 0x88000)

  def test_a_signed_build_has_no_drive_and_no_version_string(self):
    self.assertFalse(ht.has_drive({'defines': ['SIGNED_FW=ON', 'SIGNED_FW_QX=0x01']}))
    self.assertTrue(ht.has_drive({'defines': ['SIGNED_FW=ON', 'FORCE_UF2=ON']}))
    self.assertTrue(ht.has_drive({'defines': ['DUALBANK_FW=ON']}))

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
    bat.f = {**FACTS, 'vid': 0x239A, 'pid': 0xDA, 'cdcPid': 0xDA}
    calls = []

    def spawn(argv, timeout):
      calls.append([str(a) for a in argv])
      return 0, out if 'serial' in argv else ''
    with mock.patch.object(bat.probe, 'reenter', return_value={'gpregret': '0000004E'}) as reenter, \
         mock.patch.object(ht, 'usb_device',
                           side_effect=lambda *a, n=iter(range(99)): {'busport': '1-1', 'ino': min(next(n), 2)}), \
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
    with mock.patch.object(ht, 'USB_TIMEOUT', 0.3), self.assertRaisesRegex(ht.StepFail, 'enumerated a drive'):
      self.serial_run('Device programmed.\n', drive='/dev/sdz')

  def test_cdc_only_waits_for_the_previous_drive_to_drop(self):
    bat = self.battery()
    bat.f = {**FACTS, 'vid': 0x239A, 'cdcPid': 0xDA}
    with mock.patch.object(ht, 'usb_device', return_value={'busport': '1-1', 'ino': 1}), \
         mock.patch.object(ht, 'cdc_tty', return_value='/dev/ttyACM9'), \
         mock.patch.object(ht.os, 'access', return_value=True), \
         mock.patch.object(ht.time, 'sleep'), \
         mock.patch.object(ht, 'block_device', side_effect=['/dev/sdz', '/dev/sdz', None, None]) as blk:
      self.assertEqual(bat.cdc_only(), '/dev/ttyACM9')
    self.assertEqual(blk.call_count, 4)

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

  def interrupted(self, third=None, settings=0x0000_0001):
    """Battery.interrupted with the probe and drive mocked: the first J-Link session reads `settings` back, and the
    third page's first 16 bytes as `third`, by default bytes D does not hold there."""
    bat, copies, sessions, calls = self.battery(), [], [], []
    nonce = b'd' * 16
    fx = ht.fixture(FACTS, nonce, pattern=3 * ht.PAGE)
    words = third if third is not None else fx['image'][2 * ht.PAGE:2 * ht.PAGE + 16][::-1]
    third_addr = FACTS['appStart'] + 2 * ht.PAGE

    def session(cmds):
      sessions.append(cmds)
      calls.append('probe')
      return mem32_line(ht.BOOTLOADER_SETTINGS, settings) + mem32_line(third_addr, words)

    def copy(dev, uf2, d, leaves=True):
      copies.append((len(uf2), leaves))
      calls.append('copy')
      return {}
    with mock.patch.object(bat.probe, 'reenter', side_effect=lambda: calls.append('enter') or {}), \
         mock.patch.object(bat.probe, 'session', side_effect=session), \
         mock.patch.object(bat.drive, 'wait', return_value='/dev/sdz'), \
         mock.patch.object(bat.drive, 'copy', side_effect=copy), \
         mock.patch.object(bat.drive, 'absent', return_value=True), \
         mock.patch.object(bat, 'app_ran', side_effect=lambda n: calls.append(('app', n)) or {}):
      ev = bat.interrupted(nonce)
    return ev, copies, sessions, fx, calls

  def test_interrupted_reads_back_two_pages_runs_the_mixed_image_then_copies_whole(self):
    ev, copies, sessions, fx, calls = self.interrupted()
    nonce = b'd' * 16
    self.assertEqual(calls, ['enter', 'copy', 'probe', ('app', nonce), 'enter', 'copy', 'probe', ('app', nonce)])
    self.assertEqual(copies, [(33 * 512, False), (len(fx['image']) * 2, True)])
    cmds = sessions[0]
    self.assertEqual((cmds[0], cmds[-2:]), ('h', ['r', 'g']))
    verify = next(c for c in cmds if c.startswith('verifybin'))
    self.assertTrue(verify.endswith(f'{FACTS["appStart"]:08X}'))
    self.assertEqual(Path(verify.split()[1]).read_bytes(), fx['image'][:2 * ht.PAGE])
    self.assertEqual(Path(sessions[1][1].split()[1]).read_bytes(), fx['image'])
    self.assertEqual(ev['bank0'], {'state': 1, 'crc': 0})

  def test_interrupted_refuses_a_whole_third_page_or_another_bank_state(self):
    fx = ht.fixture(FACTS, b'd' * 16, pattern=3 * ht.PAGE)
    with self.assertRaisesRegex(ht.StepFail, 'not cut short'):
      self.interrupted(third=fx['image'][2 * ht.PAGE:2 * ht.PAGE + 16])
    for settings in (0x0000_00FF, 0x1234_0001):
      with self.subTest(settings=hex(settings)), self.assertRaisesRegex(ht.StepFail, 'without a CRC'):
        self.interrupted(settings=settings)

  def test_softdevice_sends_the_release_package_then_verifies_every_run(self):
    bat = self.battery()
    bat.f = {**FACTS, 'vid': 1, 'pid': 2, 'cdcPid': 2, 'zip': '/r/pca10056_bootloader-1_s140_6.1.1.zip',
             'runs': [(0, '/w/run-00000000.bin'), (0x1000, '/w/run-00001000.bin')]}
    calls = []
    with mock.patch.object(bat.probe, 'reenter', side_effect=lambda m: calls.append(('enter', m)) or {}), \
         mock.patch.object(bat, 'cdc_only', return_value='/dev/ttyACM9'), \
         mock.patch.object(bat, 'dfu_serial', side_effect=lambda pkg, tty: calls.append(('dfu', pkg, tty))), \
         mock.patch.object(bat.drive, 'wait', side_effect=lambda: calls.append(('drive',)) or '/dev/sdz'), \
         mock.patch.object(bat.probe, 'session', side_effect=lambda c: calls.append(('probe', c)) or ''), \
         mock.patch.object(ht.Path, 'stat', return_value=mock.Mock(st_size=4)), \
         mock.patch.object(bat, 'enumerations', return_value=set()), \
         mock.patch.object(bat.drive, 'info', return_value=[]), \
         mock.patch.object(bat.drive, 'copy', side_effect=lambda *a: calls.append(('copy',)) or {}), \
         mock.patch.object(bat, 'app_ran', side_effect=lambda n: calls.append(('app', n)) or {}):
      ev = bat.softdevice(b'e' * 16)
    self.assertEqual(calls, [('enter', ht.DFU_MAGIC_SERIAL_ONLY_RESET), ('dfu', bat.f['zip'], '/dev/ttyACM9'),
                             ('drive',), ('probe', ['h', 'verifybin /w/run-00000000.bin 00000000',
                                                    'verifybin /w/run-00001000.bin 00001000', 'g']),
                             ('drive',), ('copy',), ('app', b'e' * 16)])
    self.assertEqual(ev['package'], 'pca10056_bootloader-1_s140_6.1.1.zip')

  def test_a_copy_the_bootloader_cannot_complete_keeps_the_drive(self):
    bat = self.battery()
    bat.f = {**FACTS, 'vid': 1, 'pid': 2}
    with mock.patch.object(bat.tools, 'spawn', return_value=(0, '')), \
         mock.patch.object(bat.tools, 'child_json', return_value=(0, {'ok': True})), \
         mock.patch.object(ht, 'QUIET', 0.3):
      with mock.patch.object(bat.drive, 'present', return_value='/dev/sdz'):
        self.assertEqual(bat.drive.copy('/dev/sdz', b'x', bat.dir, leaves=False)['stayed'], 0.3)
      with mock.patch.object(bat.drive, 'present', return_value=None), \
           self.assertRaisesRegex(ht.StepFail, 'left after a copy it cannot complete'):
        bat.drive.copy('/dev/sdz', b'x', bat.dir, leaves=False)

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

  def dual(self):
    bat = self.battery()
    bat.f = {**FACTS, 'vid': 0x239A, 'pid': 0xDA, 'cdcPid': 0xDA, 'bank1': 0x88000}
    return bat

  def test_usb_device_takes_the_exact_serial_and_refuses_two(self):
    scan = [{'busport': '1-1', 'ino': 5, 'serial': 'ABC'}, {'busport': '1-2', 'ino': 6, 'serial': 'abc'}]
    with mock.patch.object(ht.usb(), 'usb_scan', return_value=scan) as us:
      self.assertEqual(ht.usb_device(0x239A, 0xDA, 'ABC')['busport'], '1-1')   # usb_scan matches case aside
      self.assertEqual(us.call_args.kwargs, {'vid_pid': ('239a', '00da'), 'serial': 'ABC'})
    with mock.patch.object(ht.usb(), 'usb_scan', return_value=scan[:1] * 2), \
         self.assertRaisesRegex(ht.StepFail, '2 USB devices'):
      ht.usb_device(0x239A, 0xDA, 'ABC')

  def test_an_unreadable_serial_is_neither_the_board_nor_proof_it_left(self):
    bat = self.dual()
    with mock.patch.object(ht.usb(), 'usb_devices', return_value=[{'dir': '/x/1-1', 'ino': 5}]), \
         mock.patch.object(ht.usb(), 'read_sysfs', return_value=None):
      self.assertIsNone(ht.usb_device(0x239A, 0xDA, 'u'))
      self.assertFalse(bat.cdc_absent())
      with mock.patch.object(ht, 'USB_TIMEOUT', 0.3), self.assertRaisesRegex(ht.StepFail, 'not seen'):
        bat.cdc_gone('the bootloader leaving')

  def test_enter_cdc_skips_the_enumeration_from_before_the_reset(self):
    bat = self.dual()
    inos = iter([7, 7, 7, 9])   # before the reset (one PID on pca10056), twice still the old one, then the new one
    with mock.patch.object(ht, 'usb_device', side_effect=lambda *a: {'busport': '1-1', 'ino': next(inos)}), \
         mock.patch.object(ht, 'cdc_tty', side_effect=['/dev/ttyACM9', '/dev/ttyACM9']) as tty, \
         mock.patch.object(ht, 'block_device', return_value=None), \
         mock.patch.object(ht.os, 'access', return_value=True), \
         mock.patch.object(ht.time, 'sleep'), \
         mock.patch.object(bat.probe, 'reenter', return_value={'gpregret': '0000004E'}):
      self.assertEqual(bat.enter_cdc(), ({'gpregret': '0000004E'}, '/dev/ttyACM9'))
    self.assertEqual(tty.call_count, 2)   # the old enumeration's tty is never taken

  def test_serial_dfu_omits_singlebank_for_a_dual_bank_build(self):
    bat = self.battery()
    with mock.patch.object(bat, 'nrfutil', return_value='Device programmed.\n') as nrfutil:
      bat.dfu_serial('/p.zip', '/dev/ttyACM9', singlebank=False)
    self.assertNotIn('--singlebank', nrfutil.call_args.args[0])

  def bank_session(self, settings=0x0000_0001, tail=None):
    sessions = []

    def session(cmds):
      sessions.append(cmds)
      out = mem32_line(ht.BOOTLOADER_SETTINGS, settings)
      mem = [c for c in cmds if c.startswith('mem32') and not c.startswith(f'mem32 {ht.BOOTLOADER_SETTINGS:08X}')]
      if mem and tail is not None:
        out += mem32_line(int(mem[0].split()[1], 16), tail)
      return out
    return sessions, session

  def test_banked_reads_the_app_back_from_both_banks_once_the_bootloader_left(self):
    bat, calls = self.dual(), []
    sessions, session = self.bank_session()
    fx = ht.fixture(bat.f, b'a' * 16)
    serial = mock.Mock(side_effect=lambda n, singlebank: calls.append(('serial', singlebank)) or {})
    with mock.patch.object(bat, 'serial', serial), \
         mock.patch.object(ht.Battery, 'cdc_absent', return_value=True), \
         mock.patch.object(bat.probe, 'session', side_effect=session):
      ev = bat.banked(b'a' * 16)
    self.assertEqual(calls, [('serial', False)])
    cmds = sessions[0]
    self.assertEqual((cmds[0], cmds[-1]), ('h', 'g'))
    self.assertTrue(cmds[2].endswith('00088000') and cmds[3].endswith('00026000'))
    self.assertEqual(Path(cmds[2].split()[1]).read_bytes(), fx['image'])
    self.assertEqual(ev['bank0']['state'], 1)
    sessions, session = self.bank_session(settings=0xFF)
    with mock.patch.object(bat, 'serial', return_value={}), \
         mock.patch.object(ht.Battery, 'cdc_absent', return_value=True), \
         mock.patch.object(bat.probe, 'session', side_effect=session), self.assertRaisesRegex(ht.StepFail, 'valid'):
      bat.banked(b'a' * 16)

  def refuse(self, nrfutil=(0, 'Sending firmware file\nActivating new firmware\nDevice programmed.\n')):
    bat, calls = self.dual(), []
    sessions, session = self.bank_session()
    with mock.patch.object(bat.probe, 'reenter', return_value={}), \
         mock.patch.object(bat, 'cdc_only', return_value='/dev/ttyACM9'), \
         mock.patch.object(bat, 'package', side_effect=lambda img, name, edit=None: calls.append(edit) or '/p.zip'), \
         mock.patch.object(bat.tools, 'spawn', side_effect=lambda argv, t: calls.append(argv) or nrfutil), \
         mock.patch.object(ht, 'stays_away', side_effect=lambda *a: calls.append('stays')), \
         mock.patch.object(ht.Battery, 'cdc_absent', return_value=True), \
         mock.patch.object(bat.probe, 'session', side_effect=session), \
         mock.patch.object(bat, 'app_ran', side_effect=lambda n: calls.append(('app', n)) or {}):
      bat.refused(b'c' * 16, b'b' * 16)
    return bat, calls, sessions

  def test_refused_keeps_the_bootloader_in_dfu_then_runs_the_old_app(self):
    bat, calls, sessions = self.refuse()
    c, b = ht.fixture(bat.f, b'c' * 16), ht.fixture(bat.f, b'b' * 16)
    self.assertIs(calls[0], ht.flip_crc)
    self.assertNotIn('--singlebank', calls[1])
    self.assertEqual(calls[2:], ['stays', ('app', b'b' * 16)])
    cmds = sessions[0]
    self.assertEqual(cmds[-2:], ['r', 'g'])
    self.assertEqual(Path(cmds[2].split()[1]).read_bytes(), c['image'])
    self.assertEqual(Path(cmds[3].split()[1]).read_bytes(), b['image'])

  def test_no_crc_refusal_without_the_stop_packet(self):
    # it exits 0 on a failed upgrade too; the bootloader checks the CRC only on the stop packet
    for nrfutil in ((0, 'Sending firmware file\nFailed to upgrade target. Error is: Three failed tx attempts\n'),
                    (1, 'Activating new firmware\n')):
      with self.subTest(nrfutil), self.assertRaisesRegex(ht.StepFail, 'stop packet'):
        self.refuse(nrfutil)

  def cut(self, code=124, tail=None):
    bat = self.dual()
    fx = ht.fixture(bat.f, b'd' * 16, pattern=ht.CUT_PAGES * ht.PAGE)
    sessions, session = self.bank_session(tail=tail if tail is not None else b'\xff' * 16)
    with mock.patch.object(bat.probe, 'reenter', return_value={}), \
         mock.patch.object(bat, 'cdc_only', return_value='/dev/ttyACM9'), \
         mock.patch.object(bat, 'package', return_value='/p.zip'), \
         mock.patch.object(bat.tools, 'spawn', return_value=(code, 'Sending DFU start packet\n')) as spawn, \
         mock.patch.object(ht.Battery, 'cdc_absent', return_value=True), \
         mock.patch.object(bat.probe, 'session', side_effect=session), \
         mock.patch.object(bat, 'app_ran', return_value={}) as app_ran:
      ev = bat.cut_short(b'd' * 16, b'b' * 16)
    self.assertEqual(spawn.call_args.args[1], ht.CUT_AFTER)
    app_ran.assert_called_once_with(b'b' * 16)
    return ev, sessions, fx

  def test_cut_short_reads_the_first_page_but_not_the_last_from_bank_1(self):
    ev, sessions, fx = self.cut()
    cmds = sessions[0]
    self.assertEqual(Path(cmds[2].split()[1]).read_bytes(), fx['image'][:ht.PAGE])
    self.assertIn(f'mem32 {0x88000 + ht.CUT_PAGES * ht.PAGE:08X} 4', cmds)
    self.assertEqual(cmds[-2:], ['r', 'g'])
    self.assertIn('notWritten', ev['bank0'])
    with self.assertRaisesRegex(ht.StepFail, 'not cut short: exit 0'):
      self.cut(code=0)
    with self.assertRaisesRegex(ht.StepFail, 'holds the whole image'):
      self.cut(tail=fx['image'][ht.CUT_PAGES * ht.PAGE:][:16])

  def signed_refusal(self, received, words=None, settings=0x0000_00FF, dhcsr=0x00010001, reas=ht.RESETREAS_SREQ,
                     nrfutil=(1, 'Exception: port gone\n'), verify_fails=False):
    """Battery.refused_signed with the probe and tools mocked: the read-back session returns `dhcsr`, `reas`,
    `settings` and the app start's `words` (by default what the case leaves); `verify_fails` makes its verifybin fail
    as the J-Link session's judge would."""
    bat, calls, sessions = self.dual(), [], []
    fx = ht.fixture(bat.f, b'c' * 16)
    words = words if words is not None else (struct.unpack_from('<4I', fx['image']) if received else (0xFFFFFFFF,) * 4)

    def session(cmds):
      sessions.append(cmds)
      calls.append('probe')
      if cmds[0].startswith('w4'):
        return mem32_line(ht.RESETREAS, 0)
      if verify_fails and any(c.startswith('verifybin') for c in cmds):
        raise ht.StepFail('J-Link session failed: Verify failed')
      return (mem32_line(ht.DHCSR, dhcsr) + mem32_line(ht.RESETREAS, reas)
              + mem32_line(ht.BOOTLOADER_SETTINGS, settings) + mem32_line(bat.f['appStart'], *words))
    with mock.patch.object(bat, 'serial', side_effect=lambda n, edit: calls.append('fresh') or {}), \
         mock.patch.object(bat, 'app_ran', side_effect=lambda n: calls.append('ran') or {}), \
         mock.patch.object(bat.probe, 'reenter', return_value={}), \
         mock.patch.object(bat, 'cdc_only',
                           side_effect=lambda pid=None, stale=(): calls.append(('cdc', pid)) or '/dev/ttyACM9'), \
         mock.patch.object(bat, 'package', return_value=Path('/p.zip')), \
         mock.patch.object(ht, 'repackage_image', return_value=fx['image']), \
         mock.patch.object(bat.tools, 'spawn', side_effect=lambda argv, t: calls.append(argv) or nrfutil), \
         mock.patch.object(bat.probe, 'session', side_effect=session):
      ev = bat.refused_image(b'c' * 16, 1) if received else bat.refused_init(b'c' * 16, None, 1)
    self.assertEqual(calls[:4], ['fresh', 'ran', ('cdc', None), 'probe'])   # a fresh app, then RESETREAS cleared
    self.assertIn('--singlebank', calls[4])
    self.assertEqual(calls[5:], ['probe', ('cdc', bat.f['pid'])])
    self.assertEqual(sessions[0], [f'w4 {ht.RESETREAS:08X} FFFFFFFF', f'mem32 {ht.RESETREAS:08X} 1'])
    self.assertEqual((sessions[1][0], sessions[1][1], sessions[1][-2:]), (f'mem32 {ht.DHCSR:08X} 1', 'h', ['r', 'g']))
    self.assertEqual(any(c.startswith('verifybin') for c in sessions[1]), received)
    return ev

  def test_an_init_packet_refusal_erases_the_fresh_app_and_resets_or_halts(self):
    ev = self.signed_refusal(False)
    self.assertEqual((ev['bank0']['state'], ev['halted'], ev['resetreas']), (0xFF, False, '00000004'))
    self.assertTrue(self.signed_refusal(False, dhcsr=0x00030003, reas=0)['halted'])
    with self.assertRaisesRegex(ht.StepFail, 'not marked invalid'):
      self.signed_refusal(False, settings=0x01)

  def test_no_refusal_without_a_reset_or_halt_or_with_the_app_kept(self):
    # nrfutil failing before it sent anything leaves the bootloader idle: no reset, no halt
    with self.assertRaisesRegex(ht.StepFail, 'no init packet refusal'):
      self.signed_refusal(False, reas=0)
    with self.assertRaisesRegex(ht.StepFail, 'no init packet refusal'):
      self.signed_refusal(False, words=(1, 2, 3, 4))

  def test_a_tampered_image_is_received_whole_and_never_activated(self):
    done = (0, 'Sending firmware file\nActivating new firmware\nDevice programmed.\n')
    ev = self.signed_refusal(True, reas=0, nrfutil=done)
    self.assertEqual(ev['verified'], '0x00026000+4352')
    # cut short after its first page: no stop packet, so validation was never requested
    with self.assertRaisesRegex(ht.StepFail, 'stop packet'):
      self.signed_refusal(True, reas=0, nrfutil=(124, 'Sending firmware file\n####\n'))
    with self.assertRaisesRegex(ht.StepFail, 'Verify failed'):
      self.signed_refusal(True, reas=0, nrfutil=done, verify_fails=True)
    with self.assertRaisesRegex(ht.StepFail, 'halted or reset'):
      self.signed_refusal(True, nrfutil=done)

  def test_signed_dfu_is_one_cdc_port_on_the_uf2_pid(self):
    bat = self.dual()
    bat.f['pid'] = 0xDB
    with mock.patch.object(bat, 'cdc_only', return_value='/dev/ttyACM9') as cdc:
      self.assertEqual(bat.cdc_dfu(), {'tty': '/dev/ttyACM9'})
    cdc.assert_called_once_with(0xDB, ())

  def test_each_profile_runs_its_steps_in_order(self):
    for profile, steps in rr.PROFILE_STEPS.items():
      with self.subTest(profile):
        bat, names = self.dual(), []
        bat.profile = profile
        with mock.patch.object(bat, 'step', side_effect=lambda name, fn: names.append(name)):
          bat.battery({})
        self.assertEqual(tuple(names), steps)

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
  """The lock helper mocked, its failure_report, session mutex and exceptions the agentrc's."""
  real = ht.lock_helper()
  return mock.Mock(failure_report=real.failure_report, ReleaseRefused=real.ReleaseRefused,
                   session_mutex=real.session_mutex, SessionBusy=real.SessionBusy,
                   LockUnavailable=real.LockUnavailable, resolve_restored=mock.Mock(return_value=[]),
                   end_failed=mock.Mock(return_value={}), **kw)


class PrivateLocks(unittest.TestCase):
  """agentrc's real lock helper over a private lock dir."""
  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.addCleanup(lambda: __import__('shutil').rmtree(self.tmp))
    self.helper = ht.lock_helper()
    patch = mock.patch.object(self.helper, 'BOARD_LOCK_DIR', str(Path(self.tmp) / 'locks'))
    patch.start()
    self.addCleanup(patch.stop)

  def lock_is_free(self):
    self.helper.release_board_lock(self.helper.take_board_lock('nrf52840dk', 'check'))
    return True


def raises(e):
  def fn(*_):
    raise e
  return fn


def dfu_fails(bat):
  bat.touched = True
  bat.step('dfu', raises(ht.StepFail('no drive')))


class RunLock(PrivateLocks):
  """cmd_run on agentrc's real board_task: the busy check under the lock, one cleanup verdict by precedence failed,
  restored, clean, and the lock ended with the run."""
  def setUp(self):
    super().setUp()
    self.helper.log_cleanup_failure('nrf52840dk', 'hil_test.py', detail='earlier run')
    (self.previous,) = self.helper.open_cleanup_failures('nrf52840dk')

  def run_cmd(self, fail=None, restored=None, busy=(), lock_wait=0):
    """cmd_run with the battery raising `fail` after install touched the board (`fail(bat)` when callable, which
    touches it itself), then the restore returning `restored` (None: the real one); `busy` the probe users, or what
    the check raises."""
    def battery(bat, base):
      if callable(fail):
        return fail(bat)
      bat.touched = True
      if fail:
        raise fail
    restores = []
    patches = [mock.patch.object(ht, 'load_board', return_value=BOARD),
               mock.patch.object(ht, 'admit', return_value=({'input': {'head': 'h'}}, {'sha256': {'merged': 'm'}}, {})),
               mock.patch.object(ht, 'facts', return_value={**FACTS, 'version': 'v', 'vid': 1, 'pid': 2}),
               mock.patch.object(ht, 'publish'),
               mock.patch.object(ht, 'admit_base', return_value={'version': 'v', 'file': 'f', 'sha256': 's'}),
               mock.patch.object(ht.shutil, 'which', return_value='/bin/true'),
               mock.patch.object(ht.Battery, 'battery', battery),
               mock.patch.object(ht, 'probe_busy', side_effect=raises(busy) if isinstance(busy, Exception)
                                 else lambda board: list(busy))]
    if restored is not None:
      patches.append(mock.patch.object(ht.Battery, 'restore', lambda bat: restores.append(1) or restored))
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out, \
         contextlib.ExitStack() as stack:
      for patch in patches:
        stack.enter_context(patch)
      code = ht.main(['run', '-b', 'pca10056', '--build-record', 'r', '--base-dir', d, '--run-root', d,
                      '--lock-wait', str(lock_wait), 'roster'])
    res = json.loads(out.getvalue().splitlines()[-1])
    return code, restores, res

  def new_events(self):
    return self.helper.cleanup_events('nrf52840dk')[1:]

  def test_a_passing_run_resolves_the_failures_it_found_then_releases(self):
    code, restores, res = self.run_cmd(restored={'pass': True, 'state': 'fixture running'})
    row = res['results'][0]
    self.assertEqual((code, res['exitCode'], restores), (0, 0, [1]))
    self.assertEqual(row['previousCleanupFailure'], self.previous)
    (event,) = self.new_events()
    self.assertEqual((event['event'], event['failure'], event['evidence']),
                     ('restored', self.previous['id'], 'pca10056: fixture running'))
    self.assertEqual((row['cleanup']['released'], 'endErrors' in row['cleanup']), (True, False))
    self.assertTrue(self.lock_is_free())

  def test_a_failed_step_whose_restore_passed_is_restored_and_fails(self):
    code, restores, res = self.run_cmd(dfu_fails, {'pass': True, 'state': 'fixture running'})
    row = res['results'][0]
    self.assertEqual((code, restores, row['detail'], row['cleanup']['pass']), (1, [1], 'dfu: no drive', True))
    self.assertEqual([e['event'] for e in self.new_events()], ['restored'])
    self.assertEqual(rr.verdict(code, res, 'pca10056')[0], 'fail')

  def test_a_failed_restore_ends_the_lock_as_a_logged_failure(self):
    code, _, res = self.run_cmd(dfu_fails, {'pass': False, 'state': 'unknown', 'detail': 'restore: no drive'})
    (event,) = self.new_events()
    self.assertEqual((code, event['event'], event['how'], event['detail'], event['receipts']),
                     (4, 'failed', 'ended-failed', 'restore: no drive', []))
    cleanup = res['results'][0]['cleanup']
    self.assertEqual((res['results'][0]['detail'], cleanup['failureLogged'], cleanup['released']),
                     ('dfu: no drive', True, True))
    self.assertEqual(len(self.helper.open_cleanup_failures('nrf52840dk')), 2)
    self.assertEqual(rr.verdict(code, res, 'pca10056')[0], 'blocker')
    self.assertIn("logged in the board's cleanup log; lock released", rr.verdict(code, res, 'pca10056')[1])
    self.assertTrue(self.lock_is_free())

  def test_an_error_or_interrupt_after_touching_skips_the_restore_and_logs(self):
    for fail, detail in ((OSError('boom'), 'OSError: boom'), (KeyboardInterrupt(), 'KeyboardInterrupt: ')):
      with self.subTest(fail=detail):
        code, restores, res = self.run_cmd(fail, {'pass': True, 'state': 'x'})
        row = res['results'][0]
        self.assertEqual((code, restores, row['detail'], row['cleanup']['state']), (4, [], detail, 'unknown'))
        self.assertTrue(self.new_events()[-1]['detail'].startswith('interrupted before cleanup; then '))
        self.assertTrue(self.lock_is_free())

  def test_an_interrupt_before_touching_aborts_the_run_and_ends_clean(self):
    def interrupted(bat):
      raise KeyboardInterrupt
    code, restores, res = self.run_cmd(interrupted)
    row = res['results'][0]
    self.assertEqual((code, row['ran'], row['cleanup']['state'], row['cleanup']['released']),
                     (1, True, 'untouched', True))
    self.assertEqual((self.new_events(), rr.verdict(code, res, 'pca10056')[0]), ([], 'blocker'))
    self.assertIn('aborted', rr.verdict(code, res, 'pca10056')[1])

  def test_tools_not_proven_ended_go_to_the_cleanup_log_with_their_receipts(self):
    def not_ended(bat):
      bat.touched = True
      bat.tools.receipts += [{'cmd': 'JLinkExe', 'cleanup': 'pending', 'receipt': {'token': 'r'}},
                             {'cmd': 'mcopy', 'cleanup': 'unknown', 'receipt': None}]
      raise ht.NotEnded('JLinkExe: a tool is not proven ended')
    code, _, res = self.run_cmd(not_ended)   # the real restore refuses to touch the board
    self.assertEqual((code, res['results'][0]['cleanup']['state']), (4, 'unknown'))
    self.assertEqual(self.new_events()[0]['receipts'], [{'token': 'r'}, {'unrecheckable': 'mcopy: cleanup unknown'}])

  def test_a_cleanup_log_that_cannot_be_written_is_reported_then_a_manual_restore_resolves(self):
    with mock.patch.object(self.helper, 'log_cleanup_failure', side_effect=OSError(28, 'No space left')):
      code, _, res = self.run_cmd(OSError('boom'))
    cleanup = res['results'][0]['cleanup']
    self.assertEqual((code, cleanup['failureLogged'], cleanup['released']), (4, False, True))
    self.assertTrue(cleanup['endErrors'][0].startswith('cleanup log not written'))
    self.assertIn('NOT logged', rr.verdict(code, res, 'pca10056')[1])
    code, _, _ = self.run_cmd(dfu_fails, {'pass': False, 'state': 'unknown', 'detail': 'restore: no drive'})
    self.assertEqual((code, len(self.helper.open_cleanup_failures('nrf52840dk'))), (4, 2))
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out, \
         mock.patch.object(ht, 'load_board', return_value=BOARD), mock.patch.object(ht, 'prove_hold'), \
         mock.patch.object(ht, 'probe_busy', return_value=[]), \
         mock.patch.object(ht, 'admit', return_value=({}, {}, {})), \
         mock.patch.object(ht, 'facts', return_value=FACTS), \
         mock.patch.object(ht.Battery, 'restore', lambda bat: {'pass': True, 'state': 'fixture running'}):
      code = ht.main(['restore', '-b', 'pca10056', '--build-record', 'r', '--run-root', d, '--under-hold', 'dbg',
                      'roster'])
    self.assertEqual((code, json.loads(out.getvalue())['pass']), (0, True))
    self.assertEqual(self.helper.open_cleanup_failures('nrf52840dk'), [])

  def test_a_busy_probe_refuses_before_the_body_without_an_event(self):
    users = [{'pid': 123, 'cmd': 'JLinkExe -USB 1'}]
    code, restores, res = self.run_cmd(restored={'pass': True, 'state': 'x'}, busy=users)
    row = res['results'][0]
    self.assertEqual((code, row['ran'], restores, row['occupiedBy']), (1, False, [], users))
    self.assertIn('board busy', row['detail'])
    self.assertEqual((row['previousCleanupFailure'], row['cleanup']['released']), (self.previous, True))
    self.assertEqual((self.new_events(), rr.verdict(code, res, 'pca10056')[0]), ([], 'blocker'))
    self.assertTrue(self.lock_is_free())

  def test_an_inconclusive_busy_check_refuses_without_an_event(self):
    code, restores, res = self.run_cmd(restored={'pass': True, 'state': 'x'},
                                       busy=ht.provider().ProbeCheckInconclusive('/proc may hide others'))
    row = res['results'][0]
    self.assertEqual((code, row['ran'], restores, row['cleanup']['released']), (1, False, [], True))
    self.assertIn('ProbeCheckInconclusive: /proc may hide others', row['detail'])
    self.assertEqual((self.new_events(), rr.verdict(code, res, 'pca10056')[0]), ([], 'not-run'))
    self.assertTrue(self.lock_is_free())

  def test_an_interrupt_while_the_lock_ends_keeps_the_restoration_and_aborts_the_run(self):
    def interrupted(board, failures, **evidence):   # a real SIGINT, deferred by the task's InterruptHold
      ht.os.kill(ht.os.getpid(), signal.SIGINT)
      return []
    with mock.patch.object(self.helper, 'resolve_restored', side_effect=interrupted) as resolve:
      code, restores, res = self.run_cmd(restored={'pass': True, 'state': 'fixture running'})
    row = res['results'][0]
    resolve.assert_called_once()
    self.assertEqual((code, res['pass'], row['pass'], row['detail'], restores),
                     (1, False, False, 'KeyboardInterrupt: ', [1]))
    self.assertEqual((row['cleanup']['pass'], row['cleanup']['released'], self.new_events()), (True, True, []))
    self.assertEqual(rr.verdict(code, res, 'pca10056'), ('blocker', 'aborted: KeyboardInterrupt: '))
    self.assertTrue(self.lock_is_free())

  def test_a_refusal_whose_lock_does_not_end_cleanly_exits_4(self):
    with mock.patch.object(self.helper, 'clear_record', return_value=False):
      code, restores, res = self.run_cmd(busy=ht.provider().ProbeCheckInconclusive('/proc may hide others'))
    row = res['results'][0]
    self.assertEqual((code, row['ran'], restores, row['cleanup']['endErrors']), (4, False, [], ['record not cleared']))
    self.assertEqual(rr.verdict(code, res, 'pca10056')[0], 'blocker')
    fh = self.helper.take_board_lock('nrf52840dk', 'other')   # a lock never taken ends nothing: still exit 1
    self.addCleanup(self.helper.release_board_lock, fh)
    code, _, res = self.run_cmd()
    self.assertEqual((code, res['results'][0]['locked'], 'endErrors' in res['results'][0]['cleanup']), (1, True, False))

  def test_contention_waits_for_lock_wait_then_runs_or_reports_locked(self):
    fh = self.helper.take_board_lock('nrf52840dk', 'other')
    t = time.monotonic()
    code, restores, res = self.run_cmd(restored={'pass': True, 'state': 'x'}, lock_wait=0.5)
    row = res['results'][0]
    self.assertGreaterEqual(time.monotonic() - t, 0.5)
    self.assertEqual((code, row['ran'], row['locked'], row['recoveryBlocker'], restores), (1, False, True, False, []))
    self.assertEqual(rr.verdict(code, res, 'pca10056')[0], 'not-run')
    timer = threading.Timer(0.3, self.helper.release_board_lock, (fh,))
    timer.start()
    self.addCleanup(timer.cancel)
    code, restores, res = self.run_cmd(restored={'pass': True, 'state': 'fixture running'}, lock_wait=10)
    self.assertEqual((code, res['results'][0]['ran'], restores), (0, True, [1]))


class AdmissionRefusal(unittest.TestCase):
  def test_each_refusal_maps_onto_the_row(self):
    h = ht.lock_helper()
    users = [{'pid': 1, 'cmd': 'JLinkExe'}]
    for e, fields in ((h.BoardBusy('b', users, []), {'occupiedBy': users}),
                      (h.RecoveryBlocked('x'), {'locked': True, 'recoveryBlocker': True}),
                      (h.BoardLocked('x'), {'locked': True, 'recoveryBlocker': False}),
                      (h.LockUnavailable('x'), {})):
      with self.subTest(e=type(e).__name__):
        self.assertEqual(ht.admission_refusal(h, e), {'detail': str(e), **fields})
    self.assertEqual(ht.admission_refusal(h, ht.provider().ProbeCheckInconclusive('x')), {})


class HostChecks(unittest.TestCase):
  def test_the_busy_check_is_agentrcs_probe_users_on_the_boards_flasher(self):
    users = [{'pid': 12, 'cmd': 'JLinkExe -USB 683944004'}]
    with mock.patch.object(ht.provider(), 'probe_users', return_value=users) as probe_users:
      self.assertEqual(ht.probe_busy(BOARD), users)
    probe_users.assert_called_once_with(BOARD['flasher'])

  def test_tools_left_refuses_when_pgrep_cannot_tell(self):
    for rc, out, left in ((0, '12 mcopy -i /dev/sde\n', ['12 mcopy -i /dev/sde']), (1, '', [])):
      with mock.patch.object(ht.subprocess, 'run', return_value=mock.Mock(returncode=rc, stdout=out, stderr='')):
        self.assertEqual(ht.tools_left(), left)
    for rc in (2, 3):
      with mock.patch.object(ht.subprocess, 'run', return_value=mock.Mock(returncode=rc, stdout='', stderr='x')), \
           self.assertRaisesRegex(ht.Refused, f'pgrep exited {rc}'):
        ht.tools_left()


class Manual(PrivateLocks):
  """install and restore under the caller's hold, or restore under an older helper's cleanup holder, one at a time
  under the board's session mutex: they only log."""
  def manual(self, cmd, outcome, hold=('--under-hold', 'dbg'), receipts=(), busy=()):
    helper = mock_helper(open_cleanup_failures=mock.Mock(return_value=[PREVIOUS]))
    ran = []

    def restore(bat):
      ran.append(1)
      bat.tools.receipts += list(receipts)
      if isinstance(outcome, BaseException):
        raise outcome
      return outcome
    holder = {'pid': 7, 'token': 'tok'}
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()) as out, \
         mock.patch.object(ht, 'lock_helper', return_value=helper), \
         mock.patch.object(ht, 'load_board', return_value=BOARD), mock.patch.object(ht, 'prove_hold') as prove, \
         mock.patch.object(ht, 'prove_cleanup_holder', return_value=holder), \
         mock.patch.object(ht, 'admit', return_value=({}, {}, {})), \
         mock.patch.object(ht, 'facts', return_value=FACTS), \
         mock.patch.object(ht, 'probe_busy', side_effect=raises(busy) if isinstance(busy, Exception)
                           else lambda board: list(busy)), \
         mock.patch.object(ht.Battery, 'restore', restore), mock.patch.object(ht.Battery, 'install', restore):
      code = ht.main([cmd, '-b', 'pca10056', '--build-record', 'r', '--run-root', d, *hold, 'roster'])
    helper.end_failed.assert_not_called()
    helper.release_board_lock.assert_not_called()
    self.prove, self.ran = prove, ran
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
                                                       receipts=[{'token': 'r'}])
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

  def test_a_busy_or_inconclusive_probe_refuses_before_any_tool_keeping_the_hold(self):
    inconclusive = ht.provider().ProbeCheckInconclusive('/proc may hide others')
    for busy, error in (([{'pid': 123, 'cmd': 'JLinkExe -USB 1'}], 'board busy'), (inconclusive, 'may hide others')):
      for hold in (('--under-hold', 'dbg'), ('--recover-holder', '7')):
        with self.subTest(busy=busy, hold=hold[0]):
          code, helper, out = self.manual('restore', {'pass': True, 'state': 'x'}, hold=hold, busy=busy)
          self.assertEqual((code, out['pass'], self.ran), (2, False, []))
          self.assertIn(error, out['error'])
          self.assertEqual(out.get('occupiedBy'), busy if isinstance(busy, list) else None)
          for unused in (helper.log_cleanup_failure, helper.resolve_restored, helper.release_token):
            unused.assert_not_called()

  def test_a_second_command_under_the_hold_refuses_before_checking_the_hold(self):
    restored = {'pass': True, 'state': 'fixture running'}
    with self.helper.session_mutex('nrf52840dk'):
      code, _, out = self.manual('restore', restored)
    self.assertEqual((code, self.ran), (2, []))
    self.assertIn('session busy', out['error'])
    self.prove.assert_not_called()
    code, _, _ = self.manual('restore', restored)   # free again once the other command ended
    self.assertEqual((code, self.ran), (0, [1]))


class Plan(unittest.TestCase):
  """hil_run_report.py profiles and hil.yml's battery step, which runs each board's profiles from it."""
  def test_fail_at_goes_to_the_profiles_that_have_the_step(self):
    self.assertEqual(rr.plan(), [(p, None) for p in rr.PROFILE_STEPS])
    self.assertEqual(rr.plan('refused'), [('default', None), ('dualbank', 'refused'), ('signed', None)])
    with self.assertRaises(ValueError):
      rr.plan('bogus')
    with contextlib.redirect_stdout(io.StringIO()) as out:
      self.assertEqual(rr.main(['profiles', '--fail-at', 'app-ran']), 0)
    self.assertEqual(out.getvalue().splitlines(), [f'{p} app-ran' for p in rr.PROFILE_STEPS])
    with contextlib.redirect_stderr(io.StringIO()):
      self.assertEqual(rr.main(['profiles', '--fail-at', 'bogus']), 2)

  def battery_step(self, fail_at='', producer=None):
    """hil.yml's 'Battery on the rig' script under bash -e with python3 stubbed: hil_run_report.py runs for real
    unless `producer` replaces its output and exit, hil_test.py only records its argv. (exit, recorded argvs)."""
    import re
    import subprocess
    text = (ROOT / '.github/workflows/hil.yml').read_text()
    body = re.search(r'- name: Battery on the rig\n\s+run: \|\n((?:          .*\n|\n)+)', text)[1]
    script = '\n'.join(line[10:] for line in body.splitlines())
    with tempfile.TemporaryDirectory() as d:
      stub = Path(d, 'python3')
      fake = f'printf %s {json.dumps(producer[0])}; exit {producer[1]}' if producer else f'exec {sys.executable} "$@"'
      stub.write_text(f'#!/bin/sh\ncase "$1" in\n  *hil_test.py) echo "$*" >> {d}/runs ;;\n  *) {fake} ;;\nesac\n')
      stub.chmod(0o755)
      env = {'PATH': f'{d}:/usr/bin:/bin', 'BOARDS': 'pca10056', 'FAIL_AT': fail_at, 'HIL_OUT': '/o'}
      p = subprocess.run(['bash', '-e', '-c', script], cwd=ROOT, env=env, capture_output=True, text=True)
      runs = Path(d, 'runs').read_text().splitlines() if Path(d, 'runs').exists() else []
    return p.returncode, runs

  def test_the_workflow_runs_every_profile_with_fail_at_where_it_applies(self):
    code, runs = self.battery_step('refused')
    self.assertEqual(code, 0)
    self.assertEqual([r.split('--profile ')[1].split()[0] for r in runs], list(rr.PROFILE_STEPS))
    self.assertEqual(['--fail-at refused' in r for r in runs], [False, True, False])

  def test_the_workflow_fails_without_a_plan(self):
    self.assertEqual(self.battery_step('bogus'), (1, []))
    self.assertEqual(self.battery_step(producer=('default\n', 1)), (1, []))   # a producer that fails
    self.assertEqual(self.battery_step(producer=('', 0)), (1, []))


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
