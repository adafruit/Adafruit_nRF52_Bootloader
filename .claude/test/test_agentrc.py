#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for .claude/skills/hil-contract/scripts/agentrc.py: which agentrc is selected and the CI pin's shape."""
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('agentrc_bootstrap', ROOT / '.claude/skills/hil-contract/scripts/agentrc.py')
ab = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ab)

NO_CI = {'GITHUB_ACTIONS': '', 'CI': ''}


class Ci(unittest.TestCase):
  def test_true_false_and_refused_values(self):
    for env, want in (({'CI': 'true'}, True), ({'GITHUB_ACTIONS': '1'}, True), (NO_CI, False),
                      ({'CI': 'false', 'GITHUB_ACTIONS': ''}, False)):
      with mock.patch.dict(os.environ, {**NO_CI, **env}):
        self.assertEqual(ab.ci(), want, env)
    with mock.patch.dict(os.environ, {**NO_CI, 'CI': 'maybe'}):
      with self.assertRaises(ImportError):
        ab.ci()


class Pin(unittest.TestCase):
  def pin_from(self, data):
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
      f.write(data if isinstance(data, str) else json.dumps(data))
    self.addCleanup(os.unlink, f.name)
    with mock.patch.object(ab, 'PIN_FILE', Path(f.name)):
      return ab.pin()

  def test_tracked_pin_is_well_formed(self):
    url, commit = ab.pin()
    self.assertTrue(url.startswith('https://'))
    self.assertEqual(len(commit), 40)

  def test_malformed_pins_are_refused(self):
    for bad in ('not json', [], {'url': 'u'}, {'url': 'u', 'commit': 'abc'}, {'url': ' u', 'commit': 'a' * 40},
                {'url': 'u', 'commit': 'A' * 40}):
      with self.assertRaises(ImportError, msg=bad):
        self.pin_from(bad)


class Selected(unittest.TestCase):
  def test_ci_selects_the_pin_and_a_dev_pc_the_install(self):
    with mock.patch.dict(os.environ, {**NO_CI, 'CI': 'true'}):
      source, root, commit, _ = ab.selected()
    self.assertEqual((source, root, commit), ('pin', ab.ROOT / ab.DEP, ab.pin()[1]))
    with mock.patch.dict(os.environ, {**NO_CI, 'AGENTRC_DIR': '/opt/agentrc'}):
      self.assertEqual(ab.selected()[:3], ('installed', Path('/opt/agentrc'), None))

  def test_fetch_is_ci_only(self):
    with mock.patch.dict(os.environ, NO_CI):
      with self.assertRaises(ImportError):
        ab.fetch()


class Api(unittest.TestCase):
  def api_of(self, source):
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / ab.SOURCE
      path.parent.mkdir(parents=True)
      path.write_text(source, encoding='utf-8')
      ab._api(Path(d), 'the remedy')

  def test_the_api_this_checkout_speaks_is_accepted(self):
    self.api_of(f'API = {ab.API}\n')

  def test_an_unusable_or_other_api_is_refused_with_the_remedy(self):
    for source in ('API = foo()\n', 'API = (\n', f'API = {ab.API + 1}\n', ''):
      with self.assertRaisesRegex(ImportError, 'the remedy', msg=source):
        self.api_of(source)


if __name__ == '__main__':
  unittest.main()
