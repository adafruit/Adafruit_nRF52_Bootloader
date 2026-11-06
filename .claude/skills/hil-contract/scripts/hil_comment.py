#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The HIL workflow's sticky pull request comment: the rig's hil_report.json and the select job's selection rendered
as Markdown, and the check that this run is still the newest for the PR's head before it is posted.

  hil_comment.py render --reports DIR --selection PATH --head SHA --tested SHA --run-url URL --run-id N --attempt N
                        --out FILE
  hil_comment.py fresh --repo R --pr N --head SHA --run-id N --attempt N

`--reports` is the rig job's run root: each `run-*/hil_report.json` in it, the newest per board. A board without
one, or a missing or unreadable `--selection`, renders as such, never as a pass or as no affected boards.
`fresh` exits 0 when the run is the newest attempt of the PR's newest HIL run for its current head, 1 when it is
stale (the reason on stdout), 2 when the API cannot tell. Stdlib only: it runs on a hosted runner.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_run_report as run_report  # noqa: E402

TITLE = '## Hardware-in-the-loop (HIL) Test Report'
STEPS = ('install', 'dfu', 'copy', 'app-ran', 'reenter', 'copy-b', 'app-ran-b')   # hil_test.py's battery
VERDICT = {'pass': '✅ passed', 'fail': '❌ failed', 'blocker': '⛔ blocked', 'not-run': '⚪ not run'}
LEGEND = '✅ passed · ❌ failed · blank not run · cleanup: the candidate reinstalled and a benign app proven running'
RUN_NAME = 'HIL PR #{}'   # hil.yml's run-name: a run's pull_requests lists every PR on its head, not the one it is for


def literal(v, table=False):
  """Report text on one line as an inline code span, so no Markdown, HTML, link or image in it renders; in a table
  cell its pipes are escaped, which GFM unescapes inside the span."""
  text = ' '.join(str(v).splitlines())
  if not text:
    return ''
  fence = '`' * (max(map(len, re.findall('`+', text)), default=0) + 1)
  pad = ' ' if text[0] in '` ' or text[-1] in '` ' else ''   # one space each side is stripped (CommonMark 6.1)
  if table:
    text = text.replace('|', '\\|')
  return f'{fence}{pad}{text}{pad}{fence}'


def board_lines(report, board):
  verdict, detail = run_report.verdict(run_report.exit_of(report), report, board)
  row = run_report.row_of(report, board) or {}
  marks = {s.get('step'): run_report.STEP_MARK[bool(s.get('pass'))] for s in row.get('steps') or []
           if isinstance(s, dict)}
  cleanup = row.get('cleanup') or {}
  clean = run_report.STEP_MARK[bool(cleanup.get('pass'))] if cleanup.get('state') not in (None, 'untouched') else ''
  seconds = sum(s.get('seconds') or 0 for s in row.get('steps') or [] if isinstance(s, dict))
  fw = row.get('firmware') or {}
  lines = [f'**{VERDICT[verdict]}**' + (f' · bootloader {literal(fw["version"])}' if fw.get('version') else ''), '',
           '| Board | ' + ' | '.join(STEPS) + ' | cleanup | steps |',
           '| --- | ' + ' | '.join(':-:' for _ in STEPS) + ' | :-: | --: |',
           '| ' + ' | '.join([literal(board, table=True), *(marks.get(s, '') for s in STEPS), clean,
                              f'{seconds:.0f}s']) + ' |', '', LEGEND, '']
  if verdict != 'pass':
    lines += [f'- {literal(board)}: {literal(detail)}', '']
  return lines


def uncovered(selection, report=None):
  """The selection's uncovered boards and MCU variants, then what the battery itself cannot establish."""
  items = [*((selection or {}).get('uncovered') or []), *((report or {}).get('uncovered') or [])]
  return ['Not covered on this rig:', *(f'- {literal(u)}' for u in items), ''] if items else []


def render(reports, selection, meta):
  """The comment body. `reports` maps a board to its parsed report; `selection` is the parsed file or None; `meta`
  is {head, tested, runUrl, runId, attempt}."""
  lines = [TITLE, '', '### ci.json (ci.lan)', '',
           f'_PR head `{meta["head"][:7]}` · tested `{meta["tested"][:7]}` · '
           f'[run {meta["runId"]}, attempt {meta["attempt"]}]({meta["runUrl"]})_', '']
  boards = None if selection is None else selection.get('boards') or []
  if boards is None:
    lines += ['_Selection unavailable._', '']
  elif not boards:
    return finish(lines + ['_No affected boards on this rig; no HIL run for this push._', ''] + uncovered(selection))
  else:
    lines += ['_Every roster board: the HIL contract or the roster changed._' if selection.get('full')
              else '_Boards the change reaches._', '']
    for board in boards:
      lines += (board_lines(reports[board], board) if board in reports
                else [f'**HIL report unavailable for {literal(board)}: the run ended without one (run incomplete).**',
                      ''])
  report = next(iter(reports.values()), None)
  return finish(lines + uncovered(selection, report) + [f'_[Reports and logs]({meta["runUrl"]})_'])


def finish(lines):
  """The body with every @mention neutralised: report text never pings anyone."""
  while lines and not lines[-1]:
    lines.pop()
  return re.sub(r'@(?=\w)', '@​', '\n'.join(lines)) + '\n'


def fresh(api, repo, pr, head, run_id, attempt):
  """(fresh, reason): this run is the newest attempt of the PR's newest HIL run for its current head. `api(path)`
  returns the parsed JSON of a GitHub REST GET; its errors propagate."""
  now = api(f'repos/{repo}/pulls/{pr}')['head']['sha']
  if now != head:
    return False, f'PR #{pr} head moved to {now[:7]}; this run tested {head[:7]}'
  runs = api(f'repos/{repo}/actions/workflows/hil.yml/runs?event=pull_request&head_sha={head}&per_page=100')
  runs = [r for r in runs['workflow_runs'] if r.get('display_title') == RUN_NAME.format(pr)]   # newest first
  if not runs:
    return False, f'no HIL run of PR #{pr} listed for {head[:7]}'
  if int(runs[0]['id']) != int(run_id):
    return False, f'run {runs[0]["id"]} is newer than run {run_id} for {head[:7]}'
  latest = runs[0]['run_attempt']
  if int(latest) != int(attempt):
    return False, f'run {run_id} attempt {latest} is newer than attempt {attempt}'
  return True, 'fresh'


def gh_api(path):
  r = subprocess.run(['gh', 'api', path], capture_output=True, text=True)
  if r.returncode:
    raise RuntimeError(f'gh api {path}: exit {r.returncode}: {r.stderr.strip()}')
  return json.loads(r.stdout)


def read_reports(root):
  """{board: native body} of the run reports under `root`, the newest run dir per board; unreadable ones skipped."""
  out = {}
  for path in sorted(Path(root).glob('run-*/hil_report.json')):   # run-<time>-<pid>: oldest first
    try:
      body = json.loads(path.read_text(encoding='utf-8'))
      run_report.check(body)
    except (OSError, ValueError, TypeError, AttributeError):
      continue
    for row in body['results']:
      out[row.get('board')] = body
  return out


def read_selection(path):
  try:
    sel = json.loads(Path(path).read_text(encoding='utf-8'))
  except (OSError, ValueError):
    return None
  return sel if isinstance(sel, dict) and isinstance(sel.get('boards'), list) else None


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
  sub = ap.add_subparsers(dest='cmd', required=True)
  r = sub.add_parser('render', help='write the comment body')
  r.add_argument('--reports', required=True, help="the rig job's run root, which may be missing")
  r.add_argument('--selection', required=True, help="the select job's hil-selection.json, which may be missing")
  r.add_argument('--head', required=True, help="the PR's head SHA")
  r.add_argument('--tested', required=True, help='the merge commit the rig tested')
  r.add_argument('--run-url', required=True)
  r.add_argument('--run-id', required=True)
  r.add_argument('--attempt', required=True)
  r.add_argument('--out', required=True)
  f = sub.add_parser('fresh', help='exit 0 when this run may post, 1 stale, 2 API error')
  f.add_argument('--repo', required=True)
  f.add_argument('--pr', required=True)
  f.add_argument('--head', required=True)
  f.add_argument('--run-id', required=True)
  f.add_argument('--attempt', required=True)
  args = ap.parse_args(argv)
  if args.cmd == 'render':
    meta = {'head': args.head, 'tested': args.tested, 'runUrl': args.run_url, 'runId': args.run_id,
            'attempt': args.attempt}
    Path(args.out).write_text(render(read_reports(args.reports), read_selection(args.selection), meta),
                              encoding='utf-8')
    return 0
  try:
    ok, why = fresh(gh_api, args.repo, args.pr, args.head, args.run_id, args.attempt)
  except (RuntimeError, OSError, ValueError, KeyError, TypeError, IndexError) as e:
    print(f'cannot tell whether run {args.run_id} is fresh: {type(e).__name__}: {e}', file=sys.stderr)
    return 2
  print(why)
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())
