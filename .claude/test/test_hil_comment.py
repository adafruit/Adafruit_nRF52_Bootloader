#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for .claude/skills/hil-contract/scripts/hil_comment.py: the sticky PR comment and its freshness check."""
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / '.claude/skills/hil-contract/scripts'
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location('hil_comment', SCRIPTS / 'hil_comment.py')
hc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hc)

META = {'head': 'a' * 40, 'tested': 'b' * 40, 'runUrl': 'https://x/runs/1/attempts/1', 'runId': '1', 'attempt': '1'}
SELECTION = {'boards': ['pca10056'], 'full': False, 'uncovered': ['nRF52832: no roster board on this host']}


def report(passed=True, failed_step=None, exit_code=0, cleanup=True):
  steps = [{'step': s, 'pass': s != failed_step, 'seconds': 2} for s in hc.STEPS]
  if failed_step:
    steps = steps[:hc.STEPS.index(failed_step) + 1]
    steps[-1]['detail'] = 'no drive'
  row = {'board': 'pca10056', 'ran': True, 'pass': passed, 'steps': steps, 'detail': None if passed else
         f'{failed_step}: no drive', 'firmware': {'version': '0.11.0'},
         'cleanup': {'pass': cleanup, 'state': 'fixture running' if cleanup else 'unknown', 'heldAfter': not cleanup}}
  return {'pass': passed, 'exitCode': exit_code, 'results': [row], 'uncovered': ['BLE OTA DFU']}


class Render(unittest.TestCase):
  def test_a_pass(self):
    body = hc.render({'pca10056': report()}, SELECTION, META)
    self.assertIn('**✅ passed** · bootloader `0.11.0`', body)
    self.assertIn('| `pca10056` | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | 14s |', body)
    self.assertIn('- `nRF52832: no roster board on this host`', body)
    self.assertIn('- `BLE OTA DFU`', body)

  def test_a_failed_step_and_a_blocker(self):
    body = hc.render({'pca10056': report(False, 'dfu', 1)}, SELECTION, META)
    self.assertIn('**❌ failed**', body)
    self.assertIn('- `pca10056`: `dfu: no drive`', body)
    body = hc.render({'pca10056': report(False, 'dfu', 4, cleanup=False)}, SELECTION, META)
    self.assertIn('**⛔ blocked**', body)

  def test_missing_report_selection_and_no_boards(self):
    self.assertIn('HIL report unavailable for `pca10056`', hc.render({}, SELECTION, META))
    self.assertIn('_Selection unavailable._', hc.render({}, None, META))
    self.assertIn('_No affected boards on this rig', hc.render({}, {'boards': [], 'uncovered': []}, META))

  def test_report_text_cannot_render_markdown_or_mention(self):
    r = report(False, 'dfu', 1)
    r['results'][0]['steps'][-1]['detail'] = '@someone [x](http://y) `z`'
    body = hc.render({'pca10056': r}, SELECTION, META)
    self.assertIn('`` dfu: @\u200bsomeone [x](http://y) `z` ``', body)
    self.assertNotIn('@someone', body)

  def test_reads_the_newest_report_per_board(self):
    with tempfile.TemporaryDirectory() as d:
      for name, version in (('run-20260101-000000-1', 'old'), ('run-20260101-000100-2', 'new')):
        (Path(d) / name).mkdir()
        r = report()
        r['results'][0]['firmware']['version'] = version
        (Path(d) / name / 'hil_report.json').write_text(json.dumps(r))
      (Path(d) / 'run-20260101-000200-3').mkdir()
      (Path(d) / 'run-20260101-000200-3' / 'hil_report.json').write_text('not json')
      got = hc.read_reports(d)
    self.assertEqual(got['pca10056']['results'][0]['firmware']['version'], 'new')


class Fresh(unittest.TestCase):
  def api(self, head, runs):
    return lambda path: {'head': {'sha': head}} if '/pulls/' in path else {'workflow_runs': runs}

  def test_newest_run_and_attempt_only(self):
    runs = [{'id': 7, 'run_attempt': 2, 'display_title': 'HIL PR #5'}, {'id': 6, 'run_attempt': 1,
                                                                         'display_title': 'HIL PR #5'}]
    self.assertEqual(hc.fresh(self.api('h', runs), 'r', 5, 'h', 7, 2), (True, 'fresh'))
    self.assertFalse(hc.fresh(self.api('h', runs), 'r', 5, 'h', 7, 1)[0])
    self.assertFalse(hc.fresh(self.api('h', runs), 'r', 5, 'h', 6, 1)[0])
    self.assertFalse(hc.fresh(self.api('moved', runs), 'r', 5, 'h', 7, 2)[0])
    self.assertFalse(hc.fresh(self.api('h', [{'id': 7, 'run_attempt': 2, 'display_title': 'HIL PR #9'}]),
                             'r', 5, 'h', 7, 2)[0])


if __name__ == '__main__':
  unittest.main()
