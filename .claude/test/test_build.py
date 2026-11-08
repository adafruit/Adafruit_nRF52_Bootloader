#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for .claude/skills/build/scripts/check_build.py: selection, refusals and the artifact check (no toolchain)."""
import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('check_build', ROOT / '.claude/skills/build/scripts/check_build.py')
cb = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cb)

BOARDS = cb.all_boards()
ROSTER = {'boards': [{'name': 'pca10056', 'probe': '683944004'}]}


def run(*argv):
  buf = io.StringIO()
  with contextlib.redirect_stdout(buf):
    code = cb.main(list(argv))
  return code, json.loads(buf.getvalue().splitlines()[-1])


class Classify(unittest.TestCase):
  def rule(self, path):
    return cb.classify(path, BOARDS)

  def test_docs_and_agent_files_build_nothing(self):
    for p in ('README.md', 'CLAUDE.md', 'AGENTS.md', '.claude/skills/build/SKILL.md',
              '.claude/skills/hil-contract/commissioning.md', '.github/ISSUE_TEMPLATE/bug_report.md'):
      self.assertEqual(self.rule(p)[0], 'docs', p)
      self.assertEqual(self.rule(p)[2], {}, p)

  def test_contract_scripts_are_not_docs(self):
    self.assertEqual(self.rule('.claude/skills/build/scripts/check_build.py')[0], 'contract-build')
    self.assertEqual(self.rule('.claude/skills/hil-contract/scripts/agentrc.py')[0], 'contract-hil')
    self.assertEqual(self.rule('test/hil/ci.json')[0], 'contract-hil')
    self.assertEqual(self.rule('test/hil/fixture/main.c')[0], 'contract-hil')
    self.assertEqual(self.rule('.claude/test/test_build.py')[0], 'tests')

  def test_board_files_and_the_board_directory_select_the_board(self):
    for p in ('src/boards/pca10100/board.h', 'src/boards/pca10100'):
      rule, _, sel = self.rule(p)
      self.assertEqual((rule, sel['boards']), ('board', {'pca10100'}), p)

  def test_contract_changes_build_the_representatives(self):
    for p in ('.claude/skills/build/scripts/check_build.py', 'test/hil/fixture/main.c', '.github/workflows/hil.yml'):
      self.assertEqual(self.rule(p)[2]['boards'], set(cb.REPRESENTATIVES), p)

  def test_removed_board_builds_nothing(self):
    rule, reason, sel = self.rule('src/boards/no_such_board/board.h')
    self.assertEqual(sel, {})
    self.assertIn('removed', reason)

  def test_shared_code_and_unknown_paths_build_the_representatives(self):
    for p in ('src/main.c', 'src/boards/boards.c', 'src/boards/boards.h', 'CMakeLists.txt',
              'cmake/toolchain/arm_gcc.cmake', 'linker/nrf52840.ld', 'lib/tinyusb', 'tools/hexmerge.py',
              'some/new/file.c'):
      self.assertEqual(self.rule(p)[2].get('boards'), set(cb.REPRESENTATIVES), p)

  def test_softdevice_selects_its_users(self):
    sel = self.rule('lib/softdevice/s140_nrf52_7.2.0/s140_nrf52_7.2.0_softdevice.hex')[2]
    self.assertEqual(sel['boards'], {'bluemicro_nrf52833'})
    self.assertEqual(self.rule('lib/softdevice/s340_nrf52_7.0.1/readme.md')[2]['boards'], set())

  def test_makefile_selects_a_wrapper_build_and_build_all_the_mandated_boards(self):
    self.assertEqual(self.rule('Makefile')[2], {'tests': True, 'make': set(cb.MANDATED)})
    self.assertEqual(self.rule('tools/build_all.py')[2], {'boards': set(cb.MANDATED)})


class BoardConfig(unittest.TestCase):
  def test_defaults_and_overrides(self):
    self.assertEqual(cb.board_config('feather_nrf52832'), ('nrf52', 's132_nrf52_6.1.1'))
    self.assertEqual(cb.board_config('pca10100'), ('nrf52833', 's140_nrf52_7.3.0'))
    self.assertEqual(cb.board_config('feather_nrf52840_express'), ('nrf52840', 's140_nrf52_6.1.1'))
    self.assertEqual(cb.board_config('xiao_nrf52840_ble'), ('nrf52840', 's140_nrf52_7.3.0'))

  def test_every_board_resolves_to_a_shipped_softdevice(self):
    for b in BOARDS:
      sd = cb.board_config(b)[1]
      self.assertTrue((ROOT / 'lib/softdevice' / sd / f'{sd}_softdevice.hex').is_file(), (b, sd))


class Select(unittest.TestCase):
  def test_firmware_selection_always_adds_the_mandated_boards(self):
    sel = cb.select(['src/boards/pca10100/board.h'], BOARDS)
    self.assertEqual(sel['build']['boards'], sorted({'pca10100', *cb.MANDATED}))

  def test_docs_only_reports_nothing_to_build(self):
    sel = cb.select(['README.md'], BOARDS)
    self.assertEqual(sel['build'], {'boards': [], 'make': [], 'tests': False})
    self.assertTrue(sel['nothingToBuild'])

  def test_hil_shared_change_runs_every_roster_board_and_names_uncovered_variants(self):
    hil = cb.select(['src/main.c'], BOARDS, ROSTER)['hil']
    self.assertEqual((hil['boards'], hil['full']), (['pca10056'], True))
    self.assertEqual(sorted(hil['uncovered']), ['nRF52832: no roster board on this host',
                                                'nRF52833: no roster board on this host'])

  def test_hil_board_change_runs_that_board_only_when_on_the_roster(self):
    self.assertEqual(cb.select(['src/boards/pca10056/board.h'], BOARDS, ROSTER)['hil']['boards'], ['pca10056'])
    hil = cb.select(['src/boards/pca10100/board.h'], BOARDS, ROSTER)['hil']
    self.assertEqual((hil['boards'], hil['uncovered']), ([], ['pca10100: no roster board on this host']))

  def test_hil_contract_change_runs_every_roster_board(self):
    hil = cb.select(['.claude/skills/hil-contract/scripts/hil_test.py'], BOARDS, ROSTER)['hil']
    self.assertEqual((hil['boards'], hil['full']), (['pca10056'], True))

  def test_a_mandated_board_changed_directly_is_hil_scope(self):
    hil = cb.select(['src/boards/feather_nrf52840_express/board.h'], BOARDS, ROSTER)['hil']
    self.assertEqual((hil['boards'], hil['uncovered']),
                     ([], ['feather_nrf52840_express: no roster board on this host']))

  def test_docs_select_no_hil_board(self):
    self.assertEqual(cb.select(['CLAUDE.md'], BOARDS, ROSTER)['hil']['boards'], [])


class Cli(unittest.TestCase):
  def test_select_only_from_a_diff_file(self):
    with tempfile.NamedTemporaryFile('w', suffix='.txt') as f:
      f.write('src/main.c\nREADME.md\n')
      f.flush()
      code, res = run('--diff-file', f.name, '--select-only')
    self.assertEqual(code, 0)
    self.assertEqual(res['build']['boards'], sorted(cb.REPRESENTATIVES))
    self.assertEqual([p['rule'] for p in res['paths']], ['shared', 'docs'])

  def test_shared_board_code_selects_every_roster_board(self):
    res = run('--scope', 'src/boards/boards.c', 'src/boards/boards.h', '--select-only')[1]
    self.assertEqual([p['rule'] for p in res['paths']], ['shared', 'shared'])
    self.assertTrue(cb.select(['src/boards/boards.c'], BOARDS, ROSTER)['hil']['full'])

  def test_scope_directory_with_a_trailing_slash(self):
    res = run('--scope', 'src/boards/xiao_nrf52840_ble/', '--select-only')[1]
    self.assertIn('xiao_nrf52840_ble', res['build']['boards'])

  def test_scope_directory_classifies_like_the_paths_inside_it(self):
    for scope, rule in (('.claude/test/', 'tests'), ('.claude/test', 'tests'),
                        ('.claude/skills/build/scripts', 'contract-build'), ('.claude/skills/build', 'contract-build'),
                        ('.claude/skills/hil-contract', 'contract-hil'), ('test/hil', 'contract-hil'),
                        ('src/boards/pca10056', 'board'), ('src', 'shared'),
                        ('lib/softdevice/s140_nrf52_6.1.1', 'softdevice'), ('lib/softdevice', 'softdevice')):
      res = run('--scope', scope, '--select-only')[1]
      self.assertIn(rule, {p['rule'] for p in res['paths']}, scope)
    self.assertTrue(run('--scope', '.claude/test/', '--select-only')[1]['build']['tests'])

  def test_scope_directory_selects_what_its_files_select(self):
    for scope in ('.claude/skills/build', '.claude/skills/hil-contract'):
      self.assertTrue(run('--scope', scope, '--select-only')[1]['build']['tests'], scope)
    boards = run('--scope', 'lib/softdevice', '--select-only')[1]['build']['boards']
    self.assertLessEqual({'pca10100', 'xiao_nrf52840_ble', 'bluemicro_nrf52833', 'pca10056'}, set(boards))

  def test_scope_directory_without_files_keeps_its_slash(self):
    res = run('--scope', 'src/no_such_dir/', '--select-only')[1]
    self.assertEqual([p['path'] for p in res['paths']], ['src/no_such_dir/'])

  def test_docs_only_needs_no_toolchain(self):
    with mock.patch.object(cb.shutil, 'which', return_value=None):
      code, res = run('--scope', 'README.md')
    self.assertEqual((code, res['pass'], res['boards']), (0, True, []))

  def test_ant_is_not_offered(self):
    self.assertEqual(run('--board', 'pca10056', '--select-only', '-D', 'ANT_LICENSE_KEY=x')[0], 2)

  def test_unknown_option_define_is_refused(self):
    code, res = run('--board', 'pca10056', '--select-only', '-D', 'CMAKE_C_FLAGS=-O0')
    self.assertEqual(code, 2)
    self.assertIn('only NAME=VALUE', res['error'])

  def test_hil_record_refuses_defines(self):
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      json.dump(ROSTER, f)
      f.flush()
      code, res = run('--board', 'pca10056', '--shared', '--hil-config', f.name, '-D', 'DUALBANK_FW=ON')
    self.assertEqual(code, 2)
    self.assertIn('builds its profiles: drop -D', res['error'])

  def test_hil_profiles_build_the_published_test_key(self):
    profiles = cb.hil_profiles()
    self.assertEqual(list(profiles), ['default', 'dualbank', 'signed'])
    key = json.loads(cb.SIGNED_FW_TEST_KEY.read_text())
    qx = dict(d.split('=', 1) for d in profiles['signed'])['SIGNED_FW_QX']
    self.assertEqual(bytes(int(b, 16) for b in qx.split(',')), bytes.fromhex(key['qx']))
    self.assertIn('TEST ONLY', key['use'])

  def test_unknown_board_and_shared_without_board_are_refused(self):
    self.assertEqual(run('--board', 'nope', '--select-only')[0], 2)
    self.assertEqual(run('--scope', 'src/main.c', '--shared')[0], 2)
    self.assertEqual(run('--scope', 'src/main.c', '--worktree')[0], 2)

  def test_signed_fw_needs_tinycrypt(self):
    with tempfile.TemporaryDirectory() as d, mock.patch.object(cb, 'ROOT', Path(d)), \
         mock.patch.object(cb.shutil, 'which', return_value='/bin/true'), \
         mock.patch.dict(sys.modules, {'intelhex': mock.Mock()}):
      for sub in ('lib/tinyusb/src', 'lib/nrfx/mdk', 'lib/uf2/utils'):
        (Path(d) / sub).mkdir(parents=True)
      cb.prerequisites(defines=['DUALBANK_FW=ON'])
      cb.prerequisites(defines=['SIGNED_FW=OFF'])
      with self.assertRaisesRegex(cb.Refusal, r'lib/tinycrypt.*git submodule update --init'):
        cb.prerequisites(defines=['SIGNED_FW=ON'])

  def test_signed_fw_follows_cmake_truth(self):
    for value in ('ON', '1', 'yes', 'TRUE', 'y'):
      self.assertTrue(cb.signed_fw([f'SIGNED_FW={value}']), value)
    for value in ('OFF', '0', 'no', 'false', 'n', '', 'x-NOTFOUND'):
      self.assertFalse(cb.signed_fw([f'SIGNED_FW={value}']), value)
    self.assertFalse(cb.signed_fw(['SIGNED_FW=ON', 'SIGNED_FW=OFF']))
    self.assertFalse(cb.signed_fw(['SIGNED_FW_QX=1']))

  def test_malformed_roster_is_refused(self):
    with tempfile.NamedTemporaryFile('w', suffix='.json') as f:
      f.write('{"boards": 3}')
      f.flush()
      self.assertEqual(run('--scope', 'src/main.c', '--select-only', '--hil-config', f.name)[0], 2)


class Artifacts(unittest.TestCase):
  NAMES = ('bootloader.elf', 'bootloader.elf.map', 'bootloader.hex', 'bootloader_mbr.hex', 'bootloader_mbr.uf2',
           'b_bootloader-1.0_s140_6.1.1.hex', 'b_bootloader-1.0_s140_6.1.1.zip', 'update-b_bootloader-1.0_nosd.uf2')

  def test_complete_set(self):
    with tempfile.TemporaryDirectory() as d:
      for n in self.NAMES:
        (Path(d) / n).write_text('x')
      found, problems = cb.expected_artifacts(Path(d), 'b')
    self.assertEqual(problems, [])
    self.assertEqual(sorted(found), ['elf', 'hex', 'hexMbr', 'map', 'merged', 'uf2Mbr', 'update', 'zip'])

  def test_missing_and_ambiguous_files_are_problems(self):
    with tempfile.TemporaryDirectory() as d:
      for n in self.NAMES[1:-2]:
        (Path(d) / n).write_text('x')
      (Path(d) / 'b_bootloader-2.0_s140_6.1.1.hex').write_text('x')
      _, problems = cb.expected_artifacts(Path(d), 'b')
    self.assertIn('bootloader.elf missing', problems)
    self.assertIn('b_bootloader-*_s*.hex: 2 match(es)', problems)
    self.assertIn('update-b_bootloader-*_nosd.uf2: 0 match(es)', problems)


class Make(unittest.TestCase):
  def test_make_flags_are_scrubbed(self):
    with tempfile.TemporaryDirectory() as d, mock.patch.dict(cb.os.environ, {'MAKEFLAGS': 'n', 'MFLAGS': '-n'}):
      log = Path(d) / 'log'
      cb.run_logged([sys.executable, '-c', 'import os; print(os.environ.get("MAKEFLAGS"), os.environ.get("MFLAGS"))'],
                    log, False)
      self.assertIn('None None', log.read_text())

  def test_make_exit_0_without_its_outputs_fails_and_keeps_the_log(self):
    with tempfile.TemporaryDirectory() as d, mock.patch.object(cb, 'ROOT', Path(d)), \
         mock.patch.object(cb, 'run_logged', return_value=0):
      row = cb.make_board('b', False)
      self.assertEqual(row['status'], 'failed')
      self.assertIn('bootloader.elf: 0 match(es)', row['firstError'])
      self.assertTrue((Path(d) / row['buildDir']).is_dir())
      self.assertIn('log', row)

  def test_make_with_every_output_passes(self):
    names = ('bootloader.elf', 'b_bootloader-1.0_nosd.hex', 'update-b_bootloader-1.0_nosd.uf2',
             'b_bootloader-1.0_s140_6.1.1.hex', 'b_bootloader-1.0_s140_6.1.1.zip')

    def fake_make(cmd, log, verbose):
      bdir = Path(next(a for a in cmd if a.startswith('BUILD='))[6:])
      for n in names:
        (bdir / n).write_text('x')
      return 0

    with tempfile.TemporaryDirectory() as d, mock.patch.object(cb, 'ROOT', Path(d)), \
         mock.patch.object(cb, 'run_logged', side_effect=fake_make):
      row = cb.make_board('b', False)
      self.assertEqual(row['status'], 'ok')
      self.assertFalse((Path(d) / row['buildDir']).exists())


if __name__ == '__main__':
  unittest.main()
