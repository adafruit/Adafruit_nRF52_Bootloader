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


def report(passed=True, failed_step=None, exit_code=0, cleanup=True, **extra):
  steps = [{'step': s, 'pass': s != failed_step, 'seconds': 2} for s in hc.STEPS]
  if failed_step:
    steps = steps[:hc.STEPS.index(failed_step) + 1]
    steps[-1]['detail'] = 'no drive'
  row = {'board': 'pca10056', 'ran': True, 'pass': passed, 'steps': steps, 'detail': None if passed else
         f'{failed_step}: no drive', 'firmware': {'version': '0.11.0'},
         'cleanup': {'pass': cleanup, 'state': 'fixture running' if cleanup else 'unknown', 'released': True,
                     **({} if cleanup else {'failureLogged': True})}, **extra}
  return {'pass': passed, 'exitCode': exit_code, 'results': [row], 'uncovered': ['BLE OTA DFU']}


class Render(unittest.TestCase):
  def test_a_pass(self):
    body = hc.render({'pca10056': ('pass', '', report())}, SELECTION, META)
    self.assertIn('**✅ passed** · bootloader `0.11.0`', body)
    self.assertIn('| `pca10056` |' + ' ✅ |' * 12 + ' 22s |', body)
    self.assertIn('- `nRF52832: no roster board on this host`', body)
    self.assertIn('- `BLE OTA DFU`', body)

  def test_a_failed_step_and_a_blocker(self):
    body = hc.render({'pca10056': ('fail', 'dfu: no drive', report(False, 'dfu', 1))}, SELECTION, META)
    self.assertIn('**❌ failed**', body)
    self.assertIn('- `pca10056`: `dfu: no drive`', body)
    body = hc.render({'pca10056': ('blocker', 'report corrupt: x', None)}, SELECTION, META)
    self.assertIn('**⛔ blocked**', body)
    self.assertIn('`report corrupt: x`', body)

  def test_a_failed_cleanup_and_an_earlier_one_are_named(self):
    earlier = {'event': 'failed', 'id': 'f1', 'at': '2026-11-08T10:00:00+0700', 'reason': 'hil_test.py',
               'detail': 'interrupted before cleanup'}
    body = report(False, 'dfu', 4, cleanup=False, previousCleanupFailure=earlier)
    verdict, detail = hc.run_report.verdict(4, body, 'pca10056')
    self.assertEqual(verdict, 'blocker')
    text = hc.render({'pca10056': (verdict, detail, body)}, SELECTION, META)
    self.assertIn("Cleanup failed (unknown): logged in the board's cleanup log; lock released", text)
    self.assertIn('- `Found an earlier failed cleanup on the board: 2026-11-08T10:00:00+0700 hil_test.py: '
                  'interrupted before cleanup`', text)
    self.assertNotIn('earlier failed cleanup', hc.render({'pca10056': ('pass', '', report())}, SELECTION, META))

  def test_missing_run_selection_and_no_boards(self):
    self.assertIn('HIL report unavailable for `pca10056`', hc.render({'pca10056': None}, SELECTION, META))
    self.assertIn('_Selection unavailable._', hc.render({}, None, META))
    self.assertIn('_No affected boards on this rig', hc.render({}, {'boards': [], 'uncovered': []}, META))

  def test_report_text_cannot_render_markdown_or_mention(self):
    body = hc.render({'pca10056': ('fail', 'dfu: @someone [x](http://y) `z`', report(False, 'dfu', 1))}, SELECTION,
                     META)
    self.assertIn('`` dfu: @\u200bsomeone [x](http://y) `z` ``', body)
    self.assertNotIn('@someone', body)


class BoardResult(unittest.TestCase):
  """Through the installed agentrc's report core, as the workflow's comment job loads the pinned one."""
  def setUp(self):
    import agentrc
    self.core = agentrc.load_file('skills/hil/scripts/hil_report_core.py')
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)

  def publish(self, name, body, termination='completed'):
    run_dir = Path(self.tmp.name, 'pca10056', name)
    run_dir.mkdir(parents=True)
    schema = hc.run_report.schema(self.core)
    attempt = self.core.start_attempt(run_dir, schema, False, name)
    meta = self.core.make_meta(schema, attempt, termination=termination, invocation=['hil_test.py'],
                               selection=['pca10056'], host='ci', roster={'name': 'ci.json', 'sha256': '0' * 64})
    self.core.publish(run_dir, schema, body, meta)
    return run_dir

  def test_a_published_pass(self):
    self.publish('run-20260101-000000-1', report())
    self.assertEqual(hc.board_result(self.core, self.tmp.name, 'pca10056')[:2], ('pass', ''))

  def test_an_aborted_run_or_a_caveat_is_not_a_pass(self):
    self.publish('run-20260101-000000-1', report(), termination='aborted')
    self.assertEqual(hc.board_result(self.core, self.tmp.name, 'pca10056')[0], 'blocker')
    self.publish('run-20260101-000100-2', dict(report(), caveat='lock lost'))
    self.assertEqual(hc.board_result(self.core, self.tmp.name, 'pca10056')[0], 'blocker')

  def test_an_unreadable_newest_report_never_falls_back_to_an_older_pass(self):
    self.publish('run-20260101-000000-1', report())
    newest = Path(self.tmp.name, 'pca10056', 'run-20260101-000100-2')
    newest.mkdir()
    (newest / 'hil_report.json').write_text('{"truncated')
    verdict, detail, body = hc.board_result(self.core, self.tmp.name, 'pca10056')
    self.assertEqual((verdict, body), ('blocker', None))
    self.assertIn('report', detail)
    self.assertIsNone(hc.board_result(self.core, self.tmp.name, 'pca10100'))


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
