# SPDX-License-Identifier: MIT
"""The bootloader's run report as agentrc's report core (`hil` Reporting) reads and writes it: the body schema, its Markdown
view of the steps, the native `hil` verdict of a board's row (SKILL.md Results, Verdicts) and its rows normalised for
the core's shared run rules. The core module is passed in, loaded through the agentrc bootstrap
(agentrc.py); verdict() needs no core. Stdlib only."""

NAME, VERSION, STEM = 'nrf52-bootloader-hil-run', 1, 'hil_report'
STEPS = ('install', 'dfu', 'copy', 'app-ran', 'reenter', 'copy-b', 'app-ran-b', 'double-reset', 'serial', 'app-ran-c',
         'update-base', 'update')   # hil_test.py's default battery, in order
PROFILE_STEPS = {'default': STEPS}   # each profile's battery (check_build.hil_profiles), in order
STEP_MARK = {True: '✅', False: '❌'}
TYPES = {'pass': bool, 'caveat': str, 'reportPath': str, 'error': str, 'exitCode': int, 'uncovered': list}


def check(body):
  if not isinstance(body.get('results'), list) or not all(isinstance(r, dict) for r in body['results']):
    raise ValueError('results must be a list of objects')
  for k, t in TYPES.items():
    if k in body and type(body[k]) is not t:
      raise ValueError(f'{k} must be {t.__name__}')


def cell(v):
  return ' '.join(str(v).splitlines()).replace('|', '\\|')


def render(core, body):
  lines = [f'HIL run: {"pass" if body.get("pass") else "fail"}']
  for k in ('error', 'caveat'):
    if body.get(k):
      lines.append(f'{k.capitalize()}: {cell(body[k])}')
  for r in body['results']:
    steps = [[cell(s.get('step', '?')), STEP_MARK[bool(s.get('pass'))], cell(s.get('detail', ''))]
             for s in r.get('steps') or [] if isinstance(s, dict)]
    cleanup = r.get('cleanup')
    if isinstance(cleanup, dict) and cleanup:
      steps.append(['cleanup', STEP_MARK[bool(cleanup.get('pass'))], cell(cleanup.get('state', ''))])
    lines += ['', f'### {cell(r.get("board", "?"))}', '']
    if r.get('detail'):
      lines += [cell(r['detail']), '']
    if failed := cleanup_failure(r):
      lines += [cell(failed), '']
    if earlier := previous_failure(r):
      lines += [cell(earlier), '']
    lines.append(core.table(['Step', 'Result', 'Detail'], steps, center={1}) if steps else '_No steps ran._')
  if body.get('uncovered'):
    lines += ['', 'Not covered on this rig:', *(f'- {cell(u)}' for u in body['uncovered'])]
  return '\n'.join(lines) + '\n'


def schema(core):
  return core.BodySchema(name=NAME, version=VERSION, stem=STEM, evidence='firmware-validation', check=check,
                         empty=lambda b: not b['results'], render=lambda b: render(core, b))


def exit_of(body):
  """verdict()'s `code` from a report alone."""
  return body['exitCode']


def cleanup_failure(row):
  """'' unless the row's cleanup failed: then whether the board's cleanup log took it and that the lock was released."""
  c = row.get('cleanup') if isinstance(row.get('cleanup'), dict) else {}
  if c.get('pass') is not False:
    return ''
  logged = ("logged in the board's cleanup log" if c.get('failureLogged') else
            f"NOT logged in the board's cleanup log ({'; '.join(map(str, c.get('endErrors') or ['no record']))})")
  return f'Cleanup failed ({c.get("state")}): {logged}; lock {"released" if c.get("released") else "not released"}'


def previous_failure(row):
  """'' unless the run found the board with open failed cleanups (hil_lock failure_report): the latest, how many."""
  f = row.get('previousCleanupFailure')
  if not isinstance(f, dict):
    return ''
  more = f' ({row["openCleanupFailures"]} open)' if row.get('openCleanupFailures') else ''
  what = f.get('unavailable') or f'{f.get("at")} {f.get("reason")}: {f.get("detail")}'
  return f'Found an earlier failed cleanup on the board{more}: {what}'


def row_of(report, board):
  """`board`'s row of a runner report, None when it has none or there is no report."""
  return next((r for r in (report or {}).get('results', []) if r.get('board') == board), None)


def verdict(code, report, board):
  """The `hil` verdict of one run from the report's row (SKILL.md Verdicts): (verdict, detail). `code` is the runner's
  exit."""
  if report is None:
    return 'blocker', f'run ended without a report (exit {code})'
  row = row_of(report, board)
  if row is None:
    return 'blocker', f'report has no row for {board} (exit {code}): {report.get("caveat") or report.get("error")}'
  cleanup = row.get('cleanup') or {}
  if code in (4, None) or row.get('ran') is None or cleanup.get('pass') is False:
    return 'blocker', f'{cleanup_failure(row) or "cleanup failed or no report"}: {row.get("detail")}'[:400]
  if row.get('occupiedBy') or row.get('recoveryBlocker') and not row.get('locked'):
    return 'blocker', str(row.get('detail'))
  if row.get('locked'):
    return ('blocker' if row.get('recoveryBlocker') else 'not-run'), f'locked: {row.get("detail")}'
  if not row.get('ran'):
    return 'not-run', str(row.get('detail') or report.get('error'))
  failed = next((s for s in row.get('steps', []) if not s.get('pass')), None)
  if row.get('pass') and cleanup.get('pass'):
    return 'pass', ''
  if failed is None:
    return 'blocker', f'aborted: {row.get("detail")}'
  if cleanup.get('state') == 'untouched':
    return 'not-run', f'{failed["step"]}: {failed.get("detail")} (nothing written)'
  return 'fail', f'{failed["step"]}: {failed.get("detail")}'


def run_rows(code, report, requested):
  """One normalised {board, verdict, detail, evidence} row per requested board, from the native verdict()."""
  evidence = [report['reportPath']] if (report or {}).get('reportPath') else []
  return [dict(zip(('verdict', 'detail'), verdict(code, report, b)), board=b, evidence=evidence) for b in requested]


def shared_run(core, code, reading, requested):
  """The core's apply_run_rules over the rows of `reading` (read_document or prepare_pair of a run's report), apart
  from the native verdict, which it can only tighten: `_hil.termination` aborted makes every board a blocker and a
  caveat denies a pass. A report that cannot be read is a blocker for every board."""
  if reading.outcome not in core.READABLE:
    rows = [{'board': b, 'verdict': 'blocker', 'detail': f'report {reading.outcome}: {reading.detail}', 'evidence': []}
            for b in requested]
    return core.apply_run_rules(schema(core), requested, rows, 'completed', '')
  termination = reading.meta['termination']
  return core.apply_run_rules(schema(core), requested, run_rows(code, reading.body, requested), termination,
                              reading.body.get('caveat', ''))
