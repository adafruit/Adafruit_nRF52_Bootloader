#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The bootloader's build contract (.claude/skills/build/SKILL.md): resolve a change to boards and build each with
CMake + Ninja the way CI (.github/workflows/githubci.yml) does, in a private build dir. The last stdout line is JSON.

  check_build.py --scope PATH...                    boards a change to these paths affects
  check_build.py --base REF [--worktree]            same, from REF...HEAD (plus uncommitted and untracked)
  check_build.py --diff-file FILE                   same, from a file listing one changed path per line
  check_build.py --board B [--board B ...]          these boards
  check_build.py ... --select-only [--hil-config ROSTER]   the selection alone, no toolchain needed
  check_build.py --board B --shared [--hil-config ROSTER]  keep cmake-build/cmake-build-B (flash); with ROSTER the HIL
                                                           record, every HIL profile in cmake-build-B[-PROFILE]
  check_build.py ... -D NAME=VALUE                  a CMake option (DUALBANK_FW, SIGNED_FW, ...), never in a HIL record

Exit: 0 pass, 1 a build or test failed, 2 usage, resolution or a missing prerequisite.
"""
import argparse
import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc  # noqa: E402  the HIL contract's agentrc bootstrap
BOARDS_DIR = ROOT / 'src' / 'boards'
# AGENTS.md's required pair: every firmware build verifies both MCU families' flagship boards
MANDATED = ('feather_nrf52840_express', 'feather_nrf52832')
# one board per MCU variant (nRF52840, nRF52832, nRF52833) plus the HIL rig's DK
REPRESENTATIVES = ('feather_nrf52840_express', 'feather_nrf52832', 'pca10100', 'pca10056')
# ANT (ANT_LICENSE_KEY, SD_NAME=s340) is not offered: the s340 SoftDevice is license-excluded from the repository
OPTIONS = ('DUALBANK_FW', 'DEFAULT_TO_OTA_DFU', 'SIGNED_FW', 'SIGNED_FW_QX', 'SIGNED_FW_QY', 'FORCE_UF2', 'SD_VERSION',
           'USE_NFCT')
# compiler, CMake and make inputs from the environment would make a build CI never makes (MAKEFLAGS=n: no build)
SCRUBBED_ENV = ('CFLAGS', 'CXXFLAGS', 'ASMFLAGS', 'LDFLAGS', 'CPPFLAGS', 'CPATH', 'C_INCLUDE_PATH', 'LIBRARY_PATH',
                'COMPILER_PATH', 'GCC_EXEC_PREFIX', 'CMAKE_TOOLCHAIN_FILE', 'CMAKE_BUILD_TYPE', 'CMAKE_GENERATOR',
                'CMAKE_C_COMPILER_LAUNCHER', 'CMAKE_PREFIX_PATH', 'MAKEFLAGS', 'MAKEOVERRIDES', 'GNUMAKEFLAGS',
                'MFLAGS')
# CMake's false constants (`if(<variable>)` is true for any other value), compared case-insensitively
CMAKE_FALSE = ('', '0', 'OFF', 'NO', 'FALSE', 'N', 'IGNORE', 'NOTFOUND')
# the CMake cache entries a build record keeps: what configured the build
RECORDED_CACHE = ('BOARD', 'CMAKE_BUILD_TYPE', 'CMAKE_GENERATOR', 'CMAKE_OBJCOPY', 'NRFUTIL',
                  'Python_EXECUTABLE', *OPTIONS)
TESTS = ('python3', '-m', 'unittest', 'discover', '-s', '.claude/test', '-p', 'test_*.py')
# the SIGNED_FW profile's published key: test only (hil-contract SKILL.md, Profiles)
SIGNED_FW_TEST_KEY = ROOT / 'test' / 'hil' / 'signed-fw-test-key.json'
MCU_NAMES = {'nrf52': 'nRF52832', 'nrf52833': 'nRF52833', 'nrf52840': 'nRF52840'}

DOCS = re.compile(r'^(README\.md|changelog\.md|CODE_OF_CONDUCT\.md|LICENSE|supported_boards\.md|CLAUDE\.md|AGENTS\.md'
                  r'|\.agents|\.coderabbit\.yaml|\.gitignore|\.gitattributes|\.clang-format|\.github/ISSUE_TEMPLATE/.*'
                  r'|\.github/PULL_REQUEST_TEMPLATE\.md|\.idea/.*|\.vscode/.*|\.PVS-Studio/.*|ozone/.*'
                  r'|\.claude/skills/[^/]+/[^/]+\.md)$')
# (pattern, rule, what it selects); first match wins
RULES = (
  (r'^src/boards/([^/]+)(/|$)', 'board', 'the board'),
  (r'^\.claude/skills/build/scripts/', 'contract-build', 'contract tests, the representatives and every roster board'),
  (r'^(\.claude/skills/hil-contract/scripts/|test/hil/)', 'contract-hil',
   'contract tests, the representatives and every roster board'),
  (r'^\.github/workflows/hil\.yml$', 'contract-hil', 'contract tests, the representatives and every roster board'),
  (r'^(\.claude/test/|\.pre-commit-config\.yaml$)', 'tests', 'contract tests'),
  (r'^\.github/workflows/githubci\.yml$', 'ci', 'contract tests and the mandated boards'),
  (r'^Makefile$', 'make', 'contract tests and a build of the mandated boards through the Makefile wrapper'),
  (r'^tools/build_all\.py$', 'tools', 'the mandated boards'),
  (r'^lib/softdevice/(s\d+_nrf52_[0-9.]+)/', 'softdevice', 'the boards that use this SoftDevice'),
  (r'^(CMakeLists\.txt|cmake/|src/|lib/|linker/|tools/hexmerge\.py$)', 'shared', 'the representatives'),
)


class Refusal(Exception):
  pass


def out(result, code):
  print(json.dumps(result, sort_keys=True))
  return code


def git(*args, check=True):
  env = {k: v for k, v in os.environ.items() if not k.startswith('GIT_')}
  p = subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, text=True, env=env)
  if check and p.returncode:
    raise Refusal(f'git {" ".join(args)}: {p.stderr.strip()}')
  return p.stdout


def all_boards():
  return sorted(p.parent.name for p in BOARDS_DIR.glob('*/board.cmake'))


def board_config(board):
  """(MCU variant, SoftDevice dir name) the CMake build resolves for `board`: board.cmake's settings over the
  CMakeLists.txt defaults."""
  text = (BOARDS_DIR / board / 'board.cmake').read_text()
  sets = dict(re.findall(r'set\((\w+)\s+([^)\s]+)\)', text))
  mcu = sets.get('MCU_VARIANT')
  if mcu not in MCU_NAMES:
    raise Refusal(f'{board}: board.cmake sets no known MCU_VARIANT ({mcu!r})')
  name = 's132' if mcu == 'nrf52' else sets.get('SD_NAME', 's140')
  version = sets.get('SD_VERSION', '7.3.0' if mcu == 'nrf52833' else '6.1.1')
  return mcu, f'{name}_nrf52_{version}'


def scope_paths(p):
  """A scope path as changed paths: a directory is every tracked and untracked file under it, so each selects what
  it would alone; one with none (removed, ignored, inside a submodule) keeps its trailing '/'."""
  rel = os.path.relpath(os.path.abspath(p), ROOT)
  if not (os.path.isdir(p) or p.endswith('/')):
    return [rel]
  files = git('ls-files', '-z', '--cached', '--others', '--exclude-standard', '--', rel, check=False).split('\0')
  return [f for f in files if f] or [rel + '/']


def changed_paths(args):
  if args.scope:
    return list(dict.fromkeys(f for p in args.scope for f in scope_paths(p)))
  if args.diff_file:
    return [l.strip() for l in Path(args.diff_file).read_text().splitlines() if l.strip()]
  paths = []
  for line in git('diff', '--name-status', '--no-renames', f'{args.base}...HEAD').splitlines():
    paths.append(line.split('\t', 1)[1])
  if args.worktree:
    paths += git('diff', '--name-only', '--no-renames', 'HEAD').split()
    paths += git('ls-files', '--others', '--exclude-standard').split()
  return sorted(set(paths))


def classify(path, boards):
  """(rule, reason, selection dict) for one changed path."""
  if DOCS.match(path):
    return 'docs', 'documentation and agent or editor metadata build nothing', {}
  for pattern, rule, what in RULES:
    m = re.match(pattern, path)
    if not m:
      continue
    if rule == 'board':
      board = m.group(1)
      if path == f'src/boards/{board}' and board not in boards:
        continue   # a file beside the boards (boards.c, boards.h) is shared code
      if board not in boards:
        return rule, f'{board} is no board in this tree (removed): nothing to build', {}
      return rule, f'{what}', {'boards': {board}, 'direct': {board}}
    if rule == 'softdevice':
      users = {b for b in boards if board_config(b)[1] == m.group(1)}
      return rule, f'{what}: {", ".join(sorted(users)) or "none"}', {'boards': users, 'direct': users}
    sel = {'contract-build': {'tests': True, 'boards': set(REPRESENTATIVES), 'hil_full': True},
           'contract-hil': {'tests': True, 'boards': set(REPRESENTATIVES), 'hil_full': True},
           'tests': {'tests': True},
           'ci': {'tests': True, 'boards': set(MANDATED)},
           'make': {'tests': True, 'make': set(MANDATED)},
           'tools': {'boards': set(MANDATED)},
           'shared': {'boards': set(REPRESENTATIVES), 'shared': True}}[rule]
    return rule, what, sel
  return 'unclassified', 'not classified: built like shared code (the representatives)', \
      {'boards': set(REPRESENTATIVES), 'shared': True}


def select(paths, boards, roster=None):
  sel = {'boards': set(), 'direct': set(), 'make': set(), 'tests': False, 'shared': False, 'hil_full': False}
  rows = []
  for path in paths:
    rule, reason, s = classify(path, boards)
    rows.append({'path': path, 'rule': rule, 'reason': reason})
    for k, v in s.items():
      sel[k] = sel[k] | v if isinstance(v, set) else (sel[k] or v)
  if sel['boards']:
    sel['boards'] |= set(MANDATED)
  result = {'paths': rows, 'build': {'boards': sorted(sel['boards']), 'make': sorted(sel['make']),
                                     'tests': sel['tests']}}
  if not (sel['boards'] or sel['make'] or sel['tests']):
    result['nothingToBuild'] = [f'{r["path"]}: {r["reason"]}' for r in rows] or ['no changed path']
  if roster is not None:
    result['hil'] = hil_selection(sel, roster, boards)
  return result


def hil_selection(sel, roster, boards):
  rig = [b['name'] for b in roster['boards']]
  full = sel['hil_full'] or sel['shared']
  # only boards a change reaches directly: the mandated pair added to every build is build coverage, not HIL scope
  chosen = set(rig) if full else sel['direct'] & set(rig)
  uncovered = [f'{b}: no roster board on this host' for b in sorted(sel['direct'] - set(rig))]
  if full:
    covered = {board_config(b)[0] for b in rig if b in boards}
    uncovered += [f'{MCU_NAMES[m]}: no roster board on this host' for m in MCU_NAMES if m not in covered]
  return {'boards': sorted(chosen), 'full': full, 'uncovered': uncovered}


def tool_version(cmd):
  try:
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
  except (OSError, subprocess.TimeoutExpired):
    return None
  return (p.stdout or p.stderr).strip().splitlines()[0] if p.returncode == 0 and (p.stdout or p.stderr) else None


def tree_state():
  """(HEAD, `git status --porcelain` text) of this checkout."""
  return git('rev-parse', 'HEAD').strip(), git('status', '--porcelain', '--untracked-files=normal')


def inputs():
  head, status = tree_state()
  subs = {}
  for line in git('submodule', 'status').splitlines():
    state, sha, path = line[0], *line[1:].split()[:2]
    subs[path] = {'commit': sha, 'dirty': state != ' '}
  cc = shutil.which('arm-none-eabi-gcc')
  return {'head': head, 'dirty': bool(status.strip()),
          'dirtyInputs': sorted({l[3:] for l in status.splitlines()}), 'submodules': subs,
          'tools': {'arm-none-eabi-gcc': {'path': cc, 'version': tool_version([cc, '--version']) if cc else None},
                    'cmake': tool_version(['cmake', '--version']), 'ninja': tool_version(['ninja', '--version']),
                    'adafruit-nrfutil': tool_version(['adafruit-nrfutil', 'version']),
                    'python': sys.version.split()[0]}}


def signed_fw(defines):
  """Whether the -D options enable SIGNED_FW, as CMakeLists.txt's `if (SIGNED_FW)` evaluates them."""
  values = [d.split('=', 1)[1].upper() for d in defines if d.split('=', 1)[0] == 'SIGNED_FW']
  return bool(values) and values[-1] not in CMAKE_FALSE and not values[-1].endswith('-NOTFOUND')


def prerequisites(make=False, defines=()):
  missing = [t for t in ('arm-none-eabi-gcc', 'cmake', 'ninja', 'adafruit-nrfutil') if not shutil.which(t)]
  if make and not shutil.which('make'):
    missing.append('make')
  try:
    import intelhex  # noqa: F401  tools/hexmerge.py and uf2conv.py need it
  except ImportError:
    missing.append('python intelhex')
  if missing:
    raise Refusal(f'missing prerequisite(s) on PATH: {", ".join(missing)} (CI installs arm-gcc 12.3.Rel1 and '
                  f'`pip install adafruit-nrfutil intelhex ninja`)')
  needed = ['lib/tinyusb/src', 'lib/nrfx/mdk', 'lib/uf2/utils']
  if signed_fw(defines):
    needed.append('lib/tinycrypt/lib/source')
  for path in needed:
    if not (ROOT / path).is_dir():
      raise Refusal(f'{path} missing: git submodule update --init')


def hil_profiles():
  """name -> the -D options of each configuration a HIL record builds, the default first."""
  try:
    key = json.loads(SIGNED_FW_TEST_KEY.read_text())
    qx, qy = (','.join(f'0x{b:02x}' for b in bytes.fromhex(key[k])) for k in ('qx', 'qy'))
  except (OSError, ValueError, KeyError, TypeError) as e:
    raise Refusal(f'{SIGNED_FW_TEST_KEY.relative_to(ROOT)}: not a key ({e})') from None
  return {'default': [], 'dualbank': ['DUALBANK_FW=ON'],
          'signed': ['SIGNED_FW=ON', f'SIGNED_FW_QX={qx}', f'SIGNED_FW_QY={qy}']}


def sha256(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def expected_artifacts(bdir, board):
  """name -> path of every file a board build must produce, or a list of what is missing or ambiguous."""
  fixed = {'elf': 'bootloader.elf', 'map': 'bootloader.elf.map', 'hex': 'bootloader.hex',
           'hexMbr': 'bootloader_mbr.hex', 'uf2Mbr': 'bootloader_mbr.uf2'}
  globs = {'merged': f'{board}_bootloader-*_s*.hex', 'zip': f'{board}_bootloader-*_s*.zip',
           'update': f'update-{board}_bootloader-*_nosd.uf2'}
  found, problems = {}, []
  for k, name in fixed.items():
    if (bdir / name).is_file():
      found[k] = bdir / name
    else:
      problems.append(f'{name} missing')
  more, more_problems = unique_matches(bdir, globs)
  return {**found, **more}, problems + more_problems


def make_artifacts(bdir, board):
  """name -> path of every file the Makefile wrapper's `all` target must produce under make's names, or a list of
  what is missing or ambiguous."""
  return unique_matches(bdir, {'elf': 'bootloader.elf', 'hexNosd': f'{board}_bootloader-*_nosd.hex',
                               'update': f'update-{board}_bootloader-*_nosd.uf2',
                               'merged': f'{board}_bootloader-*_s*.hex', 'zip': f'{board}_bootloader-*_s*.zip'})


def unique_matches(bdir, globs):
  found, problems = {}, []
  for k, pattern in globs.items():
    hits = [Path(p) for p in glob.glob(str(bdir / pattern))]
    if len(hits) == 1:
      found[k] = hits[0]
    else:
      problems.append(f'{pattern}: {len(hits)} match(es)')
  return found, problems


def cmake_config(bdir):
  """RECORDED_CACHE entries of a configured build dir, the generator and build type included."""
  found = {}
  for line in (bdir / 'CMakeCache.txt').read_text().splitlines():
    m = re.match(r'^(\w+):\w+=(.*)$', line)
    if m and m[1] in RECORDED_CACHE:
      found[m[1]] = m[2]
  return found


def first_error(log):
  for line in Path(log).read_text(errors='replace').splitlines():
    if re.search(r'\berror\b|Error|FAILED|No such file', line):
      return line.strip()[:300]
  return None


def run_logged(cmd, log, verbose):
  env = {k: v for k, v in os.environ.items() if k not in SCRUBBED_ENV}
  with open(log, 'a') as f:
    f.write('$ ' + ' '.join(cmd) + '\n')
    f.flush()
    p = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in p.stdout:
      f.write(line)
      if verbose:
        sys.stderr.write(line)
    return p.wait()


def build_board(board, defines, shared, verbose, profile=None):
  name = f'cmake-build-{board}' if shared else f'cmake-build-agent-{os.getpid()}-{board}'
  bdir = ROOT / 'cmake-build' / (name if profile in (None, 'default') else f'{name}-{profile}')
  shutil.rmtree(bdir, ignore_errors=True)
  bdir.mkdir(parents=True)
  log = bdir / 'check_build.log'
  row = {'board': str(board), 'buildDir': str(bdir.relative_to(ROOT)), 'log': str(log.relative_to(ROOT))}
  if profile:
    row.update(profile=profile, defines=list(defines))
  cmd = ['cmake', '-S', str(ROOT), '-B', str(bdir), '-G', 'Ninja', f'-DBOARD={board}', *(f'-D{d}' for d in defines)]
  rc = run_logged(cmd, log, verbose) or run_logged(['cmake', '--build', str(bdir)], log, verbose)
  if rc:
    row.update(status='failed', firstError=first_error(log) or f'exit {rc}')
    return row
  found, problems = expected_artifacts(bdir, board)
  if problems:
    row.update(status='failed', firstError='; '.join(problems))
    return row
  row['status'] = 'ok'
  row['artifacts'] = {k: str(p.relative_to(ROOT)) for k, p in sorted(found.items())}
  row['config'] = cmake_config(bdir)
  if shared:
    row['sha256'] = {k: sha256(p) for k, p in sorted(found.items())}
    sd = re.search(r'_(s\d+)_([0-9.]+)\.hex$', found['merged'].name)
    sd_hex = ROOT / 'lib' / 'softdevice' / f'{sd[1]}_nrf52_{sd[2]}' / f'{sd[1]}_nrf52_{sd[2]}_softdevice.hex'
    row['softdevice'] = {'name': sd[1], 'version': sd[2], 'hex': str(sd_hex.relative_to(ROOT)),
                         'sha256': sha256(sd_hex)}
  else:
    shutil.rmtree(bdir)
    row['removed'] = True
  return row


def make_board(board, verbose):
  bdir = ROOT / 'cmake-build' / f'make-agent-{os.getpid()}-{board}'
  shutil.rmtree(bdir, ignore_errors=True)
  bdir.mkdir(parents=True)
  log = bdir / 'check_build.log'
  rc = run_logged(['make', '-j', f'BOARD={board}', f'BUILD={bdir}', 'all'], log, verbose)
  row = {'board': board, 'backend': 'make', 'buildDir': str(bdir.relative_to(ROOT))}
  problems = [] if rc else make_artifacts(bdir, board)[1]
  if rc or problems:
    row.update(status='failed', firstError=(first_error(log) or f'exit {rc}') if rc else '; '.join(problems),
               log=str(log.relative_to(ROOT)))
  else:
    row['status'] = 'ok'
    shutil.rmtree(bdir)
  return row


def load_roster(path):
  try:
    data = json.loads(Path(path).read_text())
    if not all(isinstance(b, dict) and b.get('name') for b in data['boards']):
      raise ValueError('an entry without a name')
  except (OSError, ValueError, KeyError, TypeError) as e:
    raise Refusal(f'{path}: not a roster ({e})') from None
  return data


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  src = ap.add_mutually_exclusive_group(required=True)
  src.add_argument('--scope', nargs='+')
  src.add_argument('--base')
  src.add_argument('--diff-file')
  src.add_argument('--board', action='append')
  ap.add_argument('--worktree', action='store_true')
  ap.add_argument('--select-only', action='store_true')
  ap.add_argument('--hil-config')
  ap.add_argument('--shared', action='store_true')
  ap.add_argument('-D', dest='defines', action='append', default=[])
  ap.add_argument('-v', '--verbose', action='store_true')
  args = ap.parse_args(argv)
  try:
    boards = all_boards()
    for d in args.defines:
      if d.split('=', 1)[0] not in OPTIONS or '=' not in d:
        raise Refusal(f'-D {d}: only NAME=VALUE for {", ".join(OPTIONS)}')
    if args.worktree and not args.base:
      raise Refusal('--worktree needs --base')
    if args.shared and not args.board:
      raise Refusal('--shared needs --board')
    roster = load_roster(args.hil_config) if args.hil_config else None
    if args.board:
      unknown = [b for b in args.board if b not in boards]
      if unknown:
        raise Refusal(f'no such board: {", ".join(unknown)}')
      result = {'paths': [], 'build': {'boards': sorted(set(args.board)), 'make': [], 'tests': False}}
    else:
      result = select(changed_paths(args), boards, roster)
    result['defines'] = args.defines
    if args.select_only:
      result['pass'] = True
      return out(result, 0)
    record = bool(args.hil_config and args.shared)
    if record and args.defines:
      raise Refusal('a HIL record builds its profiles: drop -D')
    profiles = hil_profiles() if record else {None: args.defines}
    if result['build']['boards'] or result['build']['make']:
      prerequisites(make=bool(result['build']['make']), defines=[d for ds in profiles.values() for d in ds])
    result['input'] = inputs()
    result['input']['sourceDateEpoch'] = os.environ.get('SOURCE_DATE_EPOCH')
    if record:
      if result['input']['dirty']:
        raise Refusal(f'a HIL record needs a clean tree: {", ".join(result["input"]["dirtyInputs"])}')
      try:
        ident = agentrc.identity()
      except ImportError as e:
        raise Refusal(f'agentrc: {e}') from None
      result['hilRecord'] = {'roster': {'path': args.hil_config, 'sha256': sha256(args.hil_config)}, 'agentrc': ident,
                             'profiles': profiles,
                             'signedFwKey': {'path': str(SIGNED_FW_TEST_KEY.relative_to(ROOT)),
                                             'sha256': sha256(SIGNED_FW_TEST_KEY)}}
  except Refusal as e:
    return out({'pass': False, 'error': str(e)}, 2)

  rows = [build_board(b, defines, args.shared, args.verbose, profile)
          for b in result['build']['boards'] for profile, defines in profiles.items()]
  rows += [make_board(b, args.verbose) for b in result['build']['make']]
  if result['build']['tests']:
    p = subprocess.run(TESTS, cwd=ROOT, capture_output=True, text=True)
    result['tests'] = {'pass': p.returncode == 0, 'tail': (p.stdout + p.stderr).strip().splitlines()[-3:]}
  result['boards'] = rows
  if record:
    head, status = tree_state()
    if head != result['input']['head'] or status.strip():
      return out({**result, 'pass': False, 'error': 'the tree changed while building; build again'}, 2)
    if sha256(args.hil_config) != result['hilRecord']['roster']['sha256']:
      return out({**result, 'pass': False, 'error': f'{args.hil_config} changed while building; build again'}, 2)
    try:
      for script in agentrc.SCRIPTS:
        agentrc.path_of(script)   # every closure file still has the bytes the record's identity took
    except ImportError as e:
      return out({**result, 'pass': False, 'error': f'agentrc changed while building: {e}'}, 2)
  failed = [r for r in rows if r['status'] != 'ok']
  test_failed = result.get('tests', {}).get('pass') is False
  result['pass'] = not failed and not test_failed
  if failed:
    result['firstError'] = f'{failed[0]["board"]}: {failed[0]["firstError"]}'
  elif test_failed:
    result['firstError'] = 'contract tests: ' + ' | '.join(result['tests']['tail'])
  return out(result, 0 if result['pass'] else 1)


if __name__ == '__main__':
  sys.exit(main())
