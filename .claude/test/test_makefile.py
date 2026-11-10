#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tests for the Makefile, the thin CMake wrapper: board checks, what reaches CMake, the commands each target runs
(flash tools stubbed, nothing touches hardware) and the build directory it leaves.

The `Build` class compiles a board and runs only under pytest (`python3 -m pytest .claude/test`) or with
WRAPPER_SLOW_TESTS=1; pre-commit and CI's `unittest discover` skip it."""
import os
import re
import runpy
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SD = ROOT / 'lib/softdevice'
SCRUBBED = ('MAKEFLAGS', 'MFLAGS', 'MAKEOVERRIDES', 'GNUMAKEFLAGS', 'MAKELEVEL', 'BOARD', 'BUILD', 'BIN', 'DEBUG',
            'CFLAGS', 'ASFLAGS', 'LDFLAGS', 'CMAKE_GENERATOR', 'CMAKE_BUILD_TYPE')
HAVE_TOOLS = bool(shutil.which('make') and shutil.which('cmake'))
HAVE_GCC = bool(shutil.which('arm-none-eabi-gcc'))
SLOW = 'pytest' in sys.modules or os.environ.get('WRAPPER_SLOW_TESTS') == '1'
resolve = runpy.run_path(str(ROOT / 'tools/abs_flags.py'))['resolve']


def make(*args, cwd=ROOT, path_prefix=None, home=None):
  env = {k: v for k, v in os.environ.items() if k not in SCRUBBED}
  if path_prefix:
    env['PATH'] = f'{path_prefix}{os.pathsep}{env["PATH"]}'
  if home:
    env['HOME'] = str(home)
  return subprocess.run(['make', '--no-print-directory', *args], cwd=cwd, env=env, capture_output=True, text=True)


def user_checkout(d, makefile_user):
  """A checkout of symlinks to this one's entries, with its own Makefile.user."""
  for entry in ROOT.iterdir():
    if entry.name not in ('Makefile.user', '_build', '_bin'):
      os.symlink(entry, Path(d) / entry.name)
  (Path(d) / 'Makefile.user').write_text(makefile_user)
  return Path(d)


def env_cmake(d):
  """A cmake stub printing the flag variables of its environment, for PATH."""
  stubs = Path(d, 'stubs')
  stubs.mkdir()
  cmake = stubs / 'cmake'
  cmake.write_text('#!/bin/sh\nprintf "CMAKE %s CFLAGS=[%s] ASMFLAGS=[%s] LDFLAGS=[%s]\\n" '
                   '"$1" "$CFLAGS" "$ASMFLAGS" "$LDFLAGS"\n')
  cmake.chmod(0o755)
  return stubs


def stub(d, name):
  """An executable that prints its name and arguments, one per line after the name."""
  p = Path(d) / name
  p.write_text(f'#!/bin/sh\necho {name.upper()} "$@"\n')
  p.chmod(0o755)
  return str(p)


def configure_line(out):
  return next(l[l.index('cmake -S . -B'):] for l in out.splitlines() if 'cmake -S . -B' in l)


class ResolveFlags(unittest.TestCase):
  """tools/abs_flags.py: existing relative path operands get the current directory, quoted for the shell; the rest
  passes byte for byte."""

  def in_dir(self, name, *paths):
    d = tempfile.TemporaryDirectory()
    self.addCleanup(d.cleanup)
    cwd = Path(d.name, name)
    for p in paths:
      (cwd / p).mkdir(parents=True)
    old = os.getcwd()
    os.chdir(cwd)
    self.addCleanup(os.chdir, old)
    return os.getcwd()

  def test_quoted_operand_resolves_whole(self):
    c = self.in_dir('co', 'custom includes')
    self.assertEqual(resolve('-I"custom includes" -include check.h'), f'-I"{c}/custom includes" -include check.h')
    self.assertEqual(resolve("-I 'custom includes'  -Icustom\\ includes"),
                     f"-I '{c}/custom includes'  -I{c}/custom\\ includes")

  def test_directory_with_spaces_is_quoted(self):
    c = self.in_dir('sp ace', 'src')
    self.assertEqual(resolve('-Isrc -I src -I"src" "-Isrc" -Isrc/../src'),
                     f"-I'{c}/'src -I '{c}/'src -I\"{c}/src\" \"-I{c}/src\" -I'{c}/'src/../src")

  def test_shell_characters_in_the_directory_stay_literal(self):
    c = self.in_dir('it\'s "$x" `y`', 'src')
    flags = resolve('-Isrc -I"src" -I\'src\'')
    words = subprocess.run(['sh', '-c', 'printf "%s\\n" ' + flags], capture_output=True, text=True,
                           check=True).stdout.splitlines()
    self.assertEqual(words, [f'-I{c}/src'] * 3)

  def test_options_inside_quoted_values_are_not_options(self):
    self.in_dir('co', 'linker', 'src')
    for flags in ('-DMSG=\'"see -L linker"\' ', '-DA="x -Isrc"', '"-DA -Isrc"', '-DA=\\"-I src\\"x'):
      self.assertEqual(resolve(flags), flags)

  def test_dash_operand_of_I_passes_as_written(self):
    c = self.in_dir('co', '-', 'src')
    self.assertEqual(resolve('-I- -I - -Isrc'), f'-I- -I - -I{c}/src')

  def test_dash_operand_of_other_options_is_a_path(self):
    """Only GCC's -I gives - a meaning of its own; the rest take it as a directory or a file."""
    c = self.in_dir('co', '-')
    self.assertEqual(resolve('-isystem - -iquote- -idirafter - -L- -include - -imacros -'),
                     f'-isystem {c}/- -iquote{c}/- -idirafter {c}/- -L{c}/- -include {c}/- -imacros {c}/-')

  def test_line_continuations_go_as_the_shell_removes_them(self):
    c = self.in_dir('co', 'src/usb')
    self.assertEqual(resolve('-DA=1 \\\n-DB="x\\\ny" -DC=\'x\\\ny\' -Isrc\\\n/usb'),
                     f'-DA=1 -DB="xy" -DC=\'x\\\ny\' -I{c}/src/usb')


@unittest.skipUnless(os.name == 'posix', 'POSIX signals')
class RunCommand(unittest.TestCase):
  """tools/abs_flags.py runs the command and exits with its status, 128+N for a signal as a shell reports it."""

  def test_interrupt_is_left_to_the_command_without_a_traceback(self):
    with tempfile.TemporaryDirectory() as d:
      ready = Path(d, 'ready')
      p = subprocess.Popen([sys.executable, str(ROOT / 'tools/abs_flags.py'), 'sh', '-c',
                            f'touch "{ready}"; exec sleep 30'], start_new_session=True, stderr=subprocess.PIPE, text=True)
      while not ready.exists() and p.poll() is None:
        time.sleep(0.05)
      os.killpg(p.pid, signal.SIGINT)
      _, err = p.communicate(timeout=30)
    self.assertEqual((p.returncode, err), (128 + signal.SIGINT, ''))

  def test_signal_death_exits_as_a_shell_reports_it(self):
    p = subprocess.run([sys.executable, str(ROOT / 'tools/abs_flags.py'), 'sh', '-c', 'kill -TERM $$'])
    self.assertEqual(p.returncode, 128 + signal.SIGTERM)


@unittest.skipUnless(HAVE_TOOLS, 'make and cmake needed')
class Boards(unittest.TestCase):
  def test_missing_board_lists_the_boards_and_fails(self):
    p = make()
    self.assertEqual(p.returncode, 2)
    self.assertIn('BOARD not defined', p.stderr)
    self.assertIn("You must provide a BOARD parameter with 'BOARD='", p.stdout)
    self.assertIn('- pca10056', p.stdout.splitlines())

  def test_invalid_board_lists_the_boards_and_fails(self):
    p = make('BOARD=nope')
    self.assertEqual(p.returncode, 2)
    self.assertIn('Invalid BOARD specified', p.stderr)
    self.assertIn("Unknown board 'nope'", p.stdout)
    self.assertIn('- feather_nrf52832', p.stdout.splitlines())

  def test_unsupported_flasher_and_dfu_without_serial_fail(self):
    self.assertIn('Unsupported flash utility', make('BOARD=pca10056', 'FLASHER=jlink', 'erase').stderr)
    p = make('BOARD=pca10056', 'flash-dfu')
    self.assertEqual(p.returncode, 2)
    self.assertIn('Undefined make flag: SERIAL', p.stderr)


@unittest.skipUnless(HAVE_TOOLS, 'make and cmake needed')
class Configure(unittest.TestCase):
  def test_variables_reach_cmake_as_make_had_them(self):
    p = make('-n', 'BOARD=pca10056', 'BUILD=b dir', 'DUALBANK_FW=1', 'SIGNED_FW=ON', 'USE_NFCT=yes', 'DEBUG=1',
             'SD_VERSION=7.3.0', 'SIGNED_FW_QX=0x1, 0x2', 'SIGNED_FW_QY=0x3', 'NRFUTIL=/x/nrfutil', 'all')
    self.assertEqual(p.returncode, 0, p.stderr)
    lines = p.stdout.splitlines()
    self.assertEqual(lines[0], 'cmake -E rm -f "b dir/CMakeCache.txt"')
    cfg = configure_line(p.stdout)
    for arg in ('-B "b dir"', '-G "Unix Makefiles"', '-DBOARD=pca10056', '-DDUALBANK_FW=ON', '-DSIGNED_FW=OFF', '-DUSE_NFCT=ON',
                '-DCMAKE_BUILD_TYPE=Debug', '"-DSD_VERSION=7.3.0"', '"-DSIGNED_FW_QX=0x1, 0x2"',
                '"-DNRFUTIL:STRING=/x/nrfutil"', '"-DCROSS_COMPILE=arm-none-eabi-"', '-DCMAKE_C_FLAGS_DEBUG=-Os'):
      self.assertIn(arg, cfg)
    self.assertEqual(lines[-1], 'cmake --build "b dir"')

  def test_omitted_variables_take_their_defaults(self):
    cfg = configure_line(make('-n', 'BOARD=pca10056', 'all').stdout)
    self.assertIn('-B "_build/build-pca10056"', cfg)
    for arg in ('-DCMAKE_BUILD_TYPE=MinSizeRel', '-DDUALBANK_FW=OFF', '-DSIGNED_FW=OFF', '-DFORCE_UF2=OFF',
                '-DDEFAULT_TO_OTA_DFU=OFF', '"-DNRFUTIL:STRING=adafruit-nrfutil"'):
      self.assertIn(arg, cfg)
    for name in ('USE_NFCT', 'SD_NAME', 'SD_VERSION', 'SD_HEX', 'SIGNED_FW_QX', 'ANT_LICENSE_KEY', 'TRACE_ETM'):
      self.assertNotIn(f'-D{name}', cfg)
    self.assertIn('-DUSE_NFCT=OFF', configure_line(make('-n', 'BOARD=m60_keyboard', 'USE_NFCT=no', 'all').stdout))

  def test_cmake_generator_overrides_the_default(self):
    self.assertIn('-G "Ninja"', configure_line(make('-n', 'BOARD=pca10056', 'CMAKE_GENERATOR=Ninja', 'all').stdout))

  def test_makefile_user_sets_variables(self):
    with tempfile.TemporaryDirectory() as d:
      for name in ('Makefile', 'src', 'cmake'):
        os.symlink(ROOT / name, Path(d) / name)
      (Path(d) / 'Makefile.user').write_text('BOARD = feather_nrf52832\nDUALBANK_FW = 1\n')
      p = make('-n', 'all', cwd=d)
    self.assertEqual(p.returncode, 0, p.stderr)
    cfg = configure_line(p.stdout)
    for arg in ('-B "_build/build-feather_nrf52832"', '-DBOARD=feather_nrf52832', '-DDUALBANK_FW=ON'):
      self.assertIn(arg, cfg)

  def test_makefile_user_flags_reach_the_configure_environment(self):
    """CMake adds env CFLAGS, ASMFLAGS and LDFLAGS to the toolchain's on a fresh cache; ASFLAGS also take CFLAGS.
    Existing relative paths after -I, -L or -include become absolute: CMake compiles in the build directory."""
    with tempfile.TemporaryDirectory() as d:
      for name in ('Makefile', 'src', 'cmake', 'tools'):
        os.symlink(ROOT / name, Path(d) / name)
      (Path(d) / 'Makefile.user').write_text('CFLAGS += -DX -include src/main.c -Isrc -Inope\nASFLAGS += -DY\n'
                                             'LDFLAGS += -Wl,-z -Lcmake\n')
      p = make('BOARD=pca10056', 'configure', cwd=d, path_prefix=env_cmake(d))
      r = Path(d).resolve()
    self.assertEqual(p.returncode, 0, p.stderr)
    cflags = f'-DX -include {r}/src/main.c -I{r}/src -Inope'
    self.assertIn(f'CMAKE -S CFLAGS=[{cflags}] ASMFLAGS=[-DY {cflags}] LDFLAGS=[-Wl,-z -L{r}/cmake]',
                  p.stdout.splitlines())
    self.assertNotIn('CMAKE_C_FLAGS=', configure_line(p.stdout))

  def test_only_path_operands_change_and_the_rest_passes_as_written(self):
    """Quoted or separate operands resolve too. Operands naming no checkout path (~, $VAR, =sysroot, a header for the
    search path), absolute ones, other words and quoted whitespace pass as written; references to later variables
    and target-specific flags take effect as in the make build."""
    with tempfile.TemporaryDirectory() as d:
      user_checkout(d, 'CFLAGS += -include ~/x.h -D tools -DS=\'"a  b"\' -DPROG=\\"$(NRFJPROG)\\" '
                    '-include "src/main.c" -I \'src\' -I/abs -I=sys -isystem  src -iquote$$HOME -include compile_date.h\n'
                    'all: CFLAGS += -include src/usb/uf2/compile_date.h\nLDFLAGS += -Wl,-z -L linker\n')
      p = make('BOARD=pca10056', 'all', cwd=d, path_prefix=env_cmake(d))
      r = Path(d).resolve()
    self.assertEqual(p.returncode, 0, p.stderr)
    cflags = (f'-include ~/x.h -D tools -DS=\'"a  b"\' -DPROG=\\"nrfjprog\\" -include "{r}/src/main.c" -I \'{r}/src\' '
              f'-I/abs -I=sys -isystem  {r}/src -iquote$HOME -include compile_date.h '
              f'-include {r}/src/usb/uf2/compile_date.h')
    self.assertIn(f'CMAKE -S CFLAGS=[{cflags}] ASMFLAGS=[ {cflags}] LDFLAGS=[-Wl,-z -L {r}/linker]',
                  p.stdout.splitlines())

  @unittest.skipUnless(HAVE_GCC, 'arm-none-eabi-gcc needed')
  def test_makefile_user_flags_compile_as_written_in_cmakes_compiler_check(self):
    """Each flag below fails the compiler check, or its #error, unless it reaches the compiler as written."""
    with tempfile.TemporaryDirectory() as d:
      user_checkout(d, 'CFLAGS += -D tools -DS=\'"a  b"\' -DPROG=\\"$(NRFJPROG)\\" '
                    '-include "src/usb/uf2/compile_date.h"\nconfigure: CFLAGS += -isystem src/usb -include ~/check.h\n')
      home = Path(d, 'home')
      home.mkdir()
      (home / 'check.h').write_text(
        '#ifndef COMPILE_DATE_H\n#error quoted -include\n#endif\n#include <uf2/compile_date.h>\n'
        '#if tools != 1\n#error -D tools\n#endif\n'
        '_Static_assert(sizeof S == sizeof "a  b", "S");\n_Static_assert(sizeof PROG == sizeof "nrfjprog", "PROG");\n')
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'configure', cwd=d, home=home)
    self.assertEqual(p.returncode, 0, p.stdout + p.stderr)

  @unittest.skipUnless(HAVE_GCC, 'arm-none-eabi-gcc needed')
  def test_checkout_relative_flags_resolve_in_cmakes_compiler_check(self):
    with tempfile.TemporaryDirectory() as d:
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'CFLAGS=-Isrc/usb/uf2 -include compile_date.h', 'configure')
    self.assertEqual(p.returncode, 0, p.stdout + p.stderr)

  @unittest.skipUnless(HAVE_GCC, 'arm-none-eabi-gcc needed')
  def test_quoted_operand_with_whitespace_and_quoted_option_text_compile(self):
    with tempfile.TemporaryDirectory() as d:
      user_checkout(d, 'CFLAGS += -I"custom includes" -include check.h -DMSG=\'"see -L linker"\'\n')
      Path(d, 'custom includes').mkdir()
      Path(d, 'custom includes', 'check.h').write_text('_Static_assert(sizeof MSG == sizeof "see -L linker", "MSG");\n')
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'configure', cwd=d)
    self.assertEqual(p.returncode, 0, p.stdout + p.stderr)

  @unittest.skipUnless(HAVE_GCC, 'arm-none-eabi-gcc needed')
  def test_checkout_relative_flags_resolve_in_a_checkout_path_with_spaces(self):
    with tempfile.TemporaryDirectory() as d:
      checkout = Path(d, 'sp ace')
      checkout.mkdir()
      user_checkout(checkout, '')
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'CFLAGS=-Isrc/usb/uf2 -include compile_date.h', 'configure',
               cwd=checkout)
    self.assertEqual(p.returncode, 0, p.stdout + p.stderr)

  def test_python_runs_the_configure_and_can_be_overridden(self):
    def runs(*args):
      return next(l.split()[0] for l in make('-n', 'BOARD=pca10056', *args, 'configure').stdout.splitlines()
                  if 'tools/abs_flags.py cmake' in l)

    self.assertEqual(runs(), 'python3')
    self.assertEqual(runs('PYTHON=python'), 'python')

  def test_python_reaches_cmake_without_shell_expansion(self):
    """The interpreter path reaches CMake as PYTHON ran, whatever shell characters it holds."""
    with tempfile.TemporaryDirectory() as d:
      bin_ = Path(d, 'py $x `y` "q" \'s\' \\b')
      bin_.mkdir()
      os.symlink(sys.executable, bin_ / 'python')
      cmake = Path(d, 'cmake')
      cmake.write_text('#!/bin/sh\nfor a; do printf "ARG[%s]\\n" "$a"; done\n')
      cmake.chmod(0o755)
      quoted = shlex.quote(str(bin_ / 'python'))
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'PYTHON=' + quoted.replace('$', '$$'), 'configure', path_prefix=d)
    self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
    self.assertEqual(p.stdout.splitlines()[-1], f'ARG[-DPython_EXECUTABLE={bin_}/python]')

  @unittest.skipUnless(HAVE_GCC, 'arm-none-eabi-gcc needed')
  def test_python_is_the_interpreter_cmake_runs_the_artifact_scripts_with(self):
    """PYTHON may be a command name: CMake gets the path it runs as, not its own FindPython pick."""
    with tempfile.TemporaryDirectory() as d:
      bin_ = Path(d, 'bin')
      bin_.mkdir()
      os.symlink(sys.executable, bin_ / 'mypython')
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'PYTHON=mypython', 'configure', path_prefix=bin_)
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      cache = Path(d, 'b', 'CMakeCache.txt').read_text()
    self.assertEqual(re.search(r'^_Python_EXECUTABLE:INTERNAL=(.*)$', cache, re.M)[1], f'{bin_}/mypython')

  @unittest.skipUnless(HAVE_GCC, 'arm-none-eabi-gcc needed')
  def test_each_configure_starts_from_an_empty_cache(self):
    with tempfile.TemporaryDirectory() as d:
      tc = Path(d) / 'tc'
      tc.mkdir()
      gcc = Path(shutil.which('arm-none-eabi-gcc')).resolve()
      for tool in gcc.parent.glob('arm-none-eabi-*'):
        os.symlink(tool, tc / tool.name.replace('arm-none-eabi-', 'xx-'))
      build = Path(d) / 'b dir'
      cache = build / 'CMakeCache.txt'
      p = make('BOARD=feather_nrf52840_express', f'BUILD={build}', 'DUALBANK_FW=1', 'SD_VERSION=7.3.0', 'USE_NFCT=yes',
               'TRACE_ETM=1', f'CROSS_COMPILE={tc}/xx-', 'configure')
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      text = cache.read_text()
      for entry in ('DUALBANK_FW:BOOL=ON', 'SD_VERSION:UNINITIALIZED=7.3.0', 'USE_NFCT:UNINITIALIZED=ON',
                    'TRACE_ETM:UNINITIALIZED=1', 'xx-objcopy'):
        self.assertIn(entry, text)
      self.assertEqual(make('BOARD=feather_nrf52840_express', f'BUILD={build}', 'configure').returncode, 0)
      text = cache.read_text()
      self.assertIn('DUALBANK_FW:BOOL=OFF', text)
      self.assertIn('arm-none-eabi-objcopy', text)
      for gone in ('SD_VERSION:', 'USE_NFCT:', 'TRACE_ETM:', 'xx-'):
        self.assertNotIn(gone, text)
      self.assertIn('s140_nrf52_6.1.1_softdevice.hex', (build / 'make_vars.mk').read_text())

  def test_configure_failure_fails_make(self):
    with tempfile.TemporaryDirectory() as d:
      p = make('BOARD=pca10056', f'BUILD={d}/b', 'CROSS_COMPILE=/nonexistent/', 'all')
    self.assertNotEqual(p.returncode, 0)


@unittest.skipUnless(HAVE_TOOLS, 'make and cmake needed')
class FlashWithoutBuild(unittest.TestCase):
  """erase, flash-sd and flash-mbr need no compiler (CROSS_COMPILE points nowhere) and run make's commands."""

  def run_targets(self, *args):
    with tempfile.TemporaryDirectory() as d:
      nrfjprog, pyocd = stub(d, 'nrfjprog'), stub(d, 'pyocd')
      p = make(f'BUILD={d}/b dir', 'CROSS_COMPILE=/nonexistent/', f'NRFJPROG={nrfjprog}', f'PYOCD={pyocd}', *args)
      board_vars = Path(d, 'b dir', 'board_vars.mk')
      return p, board_vars.read_text() if board_vars.exists() else ''

  def test_nrfjprog_commands(self):
    p, _ = self.run_targets('BOARD=pca10056', 'flash-sd', 'flash-mbr', 'erase')
    self.assertEqual(p.returncode, 0, p.stderr)
    out = p.stdout.splitlines()
    self.assertIn(f'NRFJPROG --program {SD}/s140_nrf52_6.1.1/s140_nrf52_6.1.1_softdevice.hex -f nrf52 --sectorerase '
                  '--reset', out)
    self.assertIn(f'NRFJPROG --program {SD}/mbr/hex/mbr_nrf52_2.4.1_mbr.hex -f nrf52 --sectorerase --reset', out)
    self.assertIn('NRFJPROG -f nrf52 --eraseall', out)
    self.assertNotIn('cmake -S', p.stdout)

  def test_aliases_and_pyocd(self):
    p, _ = self.run_targets('BOARD=feather_nrf52832', 'FLASHER=pyocd', 'sd', 'mbr', 'erase')
    self.assertEqual(p.returncode, 0, p.stderr)
    out = p.stdout.splitlines()
    self.assertIn(f'PYOCD flash -t nrf52 {SD}/s132_nrf52_6.1.1/s132_nrf52_6.1.1_softdevice.hex', out)
    self.assertIn(f'PYOCD flash -t nrf52 {SD}/mbr/hex/mbr_nrf52_2.4.1_mbr.hex', out)
    self.assertIn('PYOCD erase -t nrf52 --chip', out)

  def test_softdevice_from_board_cmake_and_overrides(self):
    p, _ = self.run_targets('BOARD=pca10100', 'flash-sd')
    self.assertIn(f'{SD}/s140_nrf52_7.3.0/s140_nrf52_7.3.0_softdevice.hex', p.stdout)
    p, _ = self.run_targets('BOARD=pca10056', 'SD_VERSION=7.3.0', 'flash-sd')
    self.assertIn(f'{SD}/s140_nrf52_7.3.0/s140_nrf52_7.3.0_softdevice.hex', p.stdout)

  def test_sd_name_on_the_command_line_wins_on_nrf52832(self):
    p, _ = self.run_targets('BOARD=feather_nrf52832', 'SD_NAME=s332', 'flash-sd')
    self.assertEqual(p.returncode, 0, p.stderr)
    self.assertIn(f'{SD}/s332_nrf52_6.1.1/s332_nrf52_6.1.1_softdevice.hex', p.stdout)

  def test_relative_sd_hex_resolves_against_the_checkout(self):
    rel = 'lib/softdevice/s140_nrf52_7.3.0/s140_nrf52_7.3.0_softdevice.hex'
    p, board_vars = self.run_targets('BOARD=pca10056', f'SD_HEX={rel}', 'flash-sd')
    self.assertEqual(p.returncode, 0, p.stderr)
    self.assertIn(f'SD_HEX = {ROOT}/{rel}', board_vars.splitlines())
    self.assertIn(f'NRFJPROG --program {rel} -f nrf52 --sectorerase --reset', p.stdout.splitlines())

  def test_flash_tool_failure_fails_make(self):
    with tempfile.TemporaryDirectory() as d:
      failing = Path(d, 'nrfjprog')
      failing.write_text(f'#!/bin/sh\necho "$@" > {d}/called\nexit 1\n')
      failing.chmod(0o755)
      p = make('BOARD=pca10056', 'CROSS_COMPILE=/nonexistent/', f'NRFJPROG={failing}', f'BUILD={d}/b', 'erase')
      self.assertEqual(Path(d, 'called').read_text(), '-f nrf52 --eraseall\n')
    self.assertNotEqual(p.returncode, 0)


@unittest.skipUnless(HAVE_TOOLS, 'make and cmake needed')
class CopyArtifact(unittest.TestCase):
  def test_copies_the_last_builds_artifacts(self):
    with tempfile.TemporaryDirectory() as d:
      build, bin_ = Path(d, 'b dir'), Path(d, 'bin dir')
      build.mkdir()
      names = {'UPDATE_UF2': 'update-x_nosd.uf2', 'MERGED_HEX': 'x_s140_6.1.1.hex', 'MERGED_ZIP': 'x_s140_6.1.1.zip'}
      for n in names.values():
        (build / n).write_text(n)
      (build / 'make_vars.mk').write_text(''.join(f'{k} = {build}/{v}\n' for k, v in names.items()))
      p = make('BOARD=pca10056', f'BUILD={build}', f'BIN={bin_}', 'CROSS_COMPILE=/nonexistent/', 'copy-artifact')
      self.assertEqual(p.returncode, 0, p.stderr)
      self.assertEqual(sorted(x.name for x in bin_.iterdir()), sorted(names.values()))

  def test_without_a_build_fails(self):
    with tempfile.TemporaryDirectory() as d:
      p = make('BOARD=pca10056', f'BUILD={d}/none', 'copy-artifact')
    self.assertEqual(p.returncode, 2)
    self.assertIn('holds no build', p.stderr)


@unittest.skipUnless(SLOW, 'slow: compiles a board; runs under pytest or with WRAPPER_SLOW_TESTS=1')
@unittest.skipUnless(HAVE_TOOLS and HAVE_GCC, 'make, cmake and arm-none-eabi-gcc needed')
class Build(unittest.TestCase):
  def test_targets_build_only_what_they_use(self):
    with tempfile.TemporaryDirectory() as d:
      tools = Path(d, 'tools')
      tools.mkdir()
      nrfjprog, gdb, nrfutil = stub(tools, 'nrfjprog'), stub(tools, 'gdb'), stub(tools, 'nrfutil')
      stub(tools, 'linkermap')
      build, bin_ = Path(d, 'b dir'), Path(d, 'bin dir')
      base = ['BOARD=pca10056', f'BUILD={build}', f'BIN={bin_}', f'NRFJPROG={nrfjprog}', f'GDB={gdb}',
              'BMP_PORT=/dev/bmp']
      missing = 'NRFUTIL=/nonexistent/adafruit-nrfutil'

      def artifacts(pattern):
        return sorted(p.name for p in build.glob(pattern))

      # flash, gdb, linkermap and gdbflash never need the DFU zip, so a missing adafruit-nrfutil does not stop them
      p = make(*base, missing, 'flash')
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      nosd = artifacts('pca10056_bootloader-*_nosd.hex')
      self.assertEqual(len(nosd), 1)
      self.assertIn(f'NRFJPROG --program {build}/{nosd[0]} --sectoranduicrerase -f nrf52 --reset',
                    p.stdout.splitlines())
      self.assertEqual(artifacts('*.zip') + artifacts('*_s140_*.hex'), [])

      p = make(*base, missing, 'gdb', 'linkermap', 'gdbflash', path_prefix=tools)
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      out = p.stdout.splitlines()
      bmp = "GDB -ex target extended-remote /dev/bmp -ex monitor swdp_scan -ex attach 1"
      self.assertIn(f'{bmp} {build}/bootloader.elf', out)
      self.assertIn(f'LINKERMAP -v {build}/bootloader.elf.map', out)
      merged = artifacts('*_s140_6.1.1.hex')
      self.assertEqual(len(merged), 1)
      self.assertIn(f'{bmp} -nx --batch -ex load {build}/{merged[0]} -ex compare-sections -ex kill', out)
      self.assertEqual(artifacts('*.zip'), [])

      # all needs the zip: a missing adafruit-nrfutil fails the build
      self.assertNotEqual(make(*base, missing, 'all').returncode, 0)

      # a relative SD_HEX is the checkout's, though the merge and package commands run in the build directory
      rel = 'lib/softdevice/s140_nrf52_6.1.1/s140_nrf52_6.1.1_softdevice.hex'
      p = make(*base, f'SD_HEX={rel}', 'all')
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      zips = artifacts('*_s140_6.1.1.zip')
      self.assertEqual(len(zips), 1)

      p = make(*base, 'copy-artifact')
      self.assertEqual(p.returncode, 0, p.stderr)
      self.assertEqual(sorted(x.name for x in bin_.iterdir()),
                       sorted([*merged, *zips, *artifacts('update-pca10056_bootloader-*_nosd.uf2')]))

      # flash-dfu builds the zip with the adafruit-nrfutil it is given and sends it over SERIAL
      p = make(*base, f'NRFUTIL={nrfutil}', 'SERIAL=/dev/ttyX', 'flash-dfu')
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      self.assertIn(f'NRFUTIL --verbose dfu serial --package {build}/{zips[0]} -p /dev/ttyX -b 115200 --singlebank '
                    '--touch 1200', p.stdout.splitlines())

      # concurrent goals share one configure and never run two builds in one directory
      self.assertEqual(make(*base, 'clean').returncode, 0)
      p = make('-j2', *base, 'all', 'flash')
      self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
      self.assertEqual(p.stdout.count('cmake -S . -B'), 1)
      self.assertEqual(len(re.findall(r'Building C object \S*/src/main\.c', p.stdout)), 1)
      self.assertEqual(p.stdout.count('Linking C executable bootloader.elf'), 1)
      self.assertIn(f'NRFJPROG --program {build}/{nosd[0]} --sectoranduicrerase -f nrf52 --reset',
                    p.stdout.splitlines())

      p = make(*base, 'clean')
      self.assertEqual(p.returncode, 0, p.stderr)
      self.assertFalse(build.exists() or bin_.exists())


if __name__ == '__main__':
  unittest.main()
