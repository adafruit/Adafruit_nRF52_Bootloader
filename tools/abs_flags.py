#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run a command with the CFLAGS, ASMFLAGS and LDFLAGS of its environment resolved against the current directory.

The make build compiled from the checkout; CMake compiles in its build directory. The operand of a path option, attached
or separate, quoted or not, that names an existing path relative to the current directory gets that directory
prepended, quoted for the shell. Words split as the shell splits them, so an option inside a quoted value is none.
Line continuations go, as the shell removes them; everything else passes as written, quoting and whitespace included,
for the shell running each compile to expand (~, $VAR) as the make build's did.

The command gets a last argument, -DPython_EXECUTABLE= naming the interpreter running this, which no shell expands:
CMake runs the artifact scripts with it (hexmerge.py needs intelhex), as the make build did; FindPython would pick its
own.

usage: abs_flags.py COMMAND [ARG...]"""
import os
import re
import shlex
import signal
import subprocess
import sys

PATH_OPTIONS = ('-I', '-L', '-isystem', '-iquote', '-idirafter', '-include', '-imacros')
ESCAPE = {None: shlex.quote, "'": lambda s: s.replace("'", "'\\''"), '"': lambda s: re.sub(r'[\\"$`]', r'\\\g<0>', s)}


def words(flags, edits):
  """Split flags as the shell does, without expanding: per word, each character of its value with the index in flags
  where text can go before it and the quote it is in. The shell's line continuations go to edits as removals."""
  word, quote, i = None, None, 0
  while i < len(flags):
    c = flags[i]
    if c == '\\' and flags[i + 1:i + 2] == '\n' and quote != "'":
      edits.append((i, 2, ''))
      i += 2
      continue
    if quote is None and c.isspace():
      if word is not None:
        yield word
      word, i = None, i + 1
      continue
    if word is None:
      word = []
    if quote is None and c in '\'"':
      quote = c
    elif c == quote:
      quote = None
    elif c == '\\' and i + 1 < len(flags) and (quote is None or quote == '"' and flags[i + 1] in '\\"$`'):
      word.append((flags[i + 1], i, quote))
      i += 1
    else:
      word.append((c, i, quote))
    i += 1
  if word is not None:
    yield word


def resolve(flags):
  edits, separate = [], None
  for word in words(flags, edits):
    if separate:
      option, operand, separate = separate, word, None
    else:
      value = ''.join(c for c, _, _ in word)
      option = next((o for o in PATH_OPTIONS if value.startswith(o)), None)
      if option is None:
        continue
      operand = word[len(option):]
      if not operand:
        separate = option
        continue
    path = ''.join(c for c, _, _ in operand)
    # GCC's -I- (or -I -) splits the search path; the other options take - as a path
    if (option, path) != ('-I', '-') and not os.path.isabs(path) and os.path.exists(path):
      _, at, quote = operand[0]
      edits.append((at, 0, ESCAPE[quote](f'{os.getcwd()}/')))
  out, last = [], 0
  for at, removed, text in sorted(edits):
    out += [flags[last:at], text]
    last = at + removed
  return ''.join(out) + flags[last:]


def main():
  for name in ('CFLAGS', 'ASMFLAGS', 'LDFLAGS'):
    if name in os.environ:
      os.environ[name] = resolve(os.environ[name])
  # Ctrl-C reaches the command too: wait for it, as a shell does, rather than leave with a traceback. A handler, unlike
  # SIG_IGN, is not inherited.
  signal.signal(signal.SIGINT, lambda *_: None)
  # os.execvp on Windows ends this process without waiting for the command: make would miss its status
  status = subprocess.call([*sys.argv[1:], f'-DPython_EXECUTABLE={sys.executable}'])
  sys.exit(128 - status if status < 0 else status)


if __name__ == '__main__':
  main()
