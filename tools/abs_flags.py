#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run a command with the CFLAGS, ASMFLAGS and LDFLAGS of its environment resolved against the current directory.

The make build compiled from the checkout; CMake compiles in its build directory. The operand of a path option, attached
or separate, quoted or not, that names an existing path relative to the current directory gets that directory
prepended, quoted for the shell. Words split as the shell splits them, so an option inside a quoted value is none.
Everything else passes as written, quoting and whitespace included, for the shell running each compile to expand
(~, $VAR) as the make build's did.

usage: abs_flags.py COMMAND [ARG...]"""
import os
import re
import shlex
import subprocess
import sys

PATH_OPTIONS = ('-I', '-L', '-isystem', '-iquote', '-idirafter', '-include', '-imacros')
ESCAPE = {None: shlex.quote, "'": lambda s: s.replace("'", "'\\''"), '"': lambda s: re.sub(r'[\\"$`]', r'\\\g<0>', s)}


def words(flags):
  """Split flags as the shell does, without expanding: per word, each character of its value with the index in flags
  where text can go before it and the quote it is in."""
  word, quote, i = None, None, 0
  while i < len(flags):
    c = flags[i]
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
  out, last, separate = [], 0, False
  for word in words(flags):
    if separate:
      operand, separate = word, False
    else:
      value = ''.join(c for c, _, _ in word)
      option = next((o for o in PATH_OPTIONS if value.startswith(o)), None)
      if option is None:
        continue
      operand = word[len(option):]
      if not operand:
        separate = True
        continue
    path = ''.join(c for c, _, _ in operand)
    if not os.path.isabs(path) and os.path.exists(path):
      _, at, quote = operand[0]
      out += [flags[last:at], ESCAPE[quote](f'{os.getcwd()}/')]
      last = at
  return ''.join(out) + flags[last:]


def main():
  for name in ('CFLAGS', 'ASMFLAGS', 'LDFLAGS'):
    if name in os.environ:
      os.environ[name] = resolve(os.environ[name])
  # os.execvp on Windows ends this process without waiting for the command: make would miss its status
  sys.exit(subprocess.call(sys.argv[1:]))


if __name__ == '__main__':
  main()
