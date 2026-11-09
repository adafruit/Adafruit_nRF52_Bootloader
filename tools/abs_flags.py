#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run a command with the CFLAGS, ASMFLAGS and LDFLAGS of its environment resolved against the current directory.

The make build compiled from the checkout; CMake compiles in its build directory. The operand of a path option, attached
or separate, quoted or not, that names an existing path relative to the current directory gets that directory
prepended. Everything else passes as written, quoting and whitespace included, for the shell running each compile to
expand (~, $VAR) as the make build's did.

usage: abs_flags.py COMMAND [ARG...]"""
import os
import re
import sys

PATH_OPERAND = re.compile(r'''(?<!\S)(["']?(?:-I|-L|-isystem|-iquote|-idirafter|-include|-imacros)\s*["']?)([^\s"']+)''')


def resolve(flags):
  def absolute(m):
    path = m[2]
    return m[0] if os.path.isabs(path) or not os.path.exists(path) else f'{m[1]}{os.getcwd()}/{path}'

  return PATH_OPERAND.sub(absolute, flags)


def main():
  for name in ('CFLAGS', 'ASMFLAGS', 'LDFLAGS'):
    if name in os.environ:
      os.environ[name] = resolve(os.environ[name])
  os.execvp(sys.argv[1], sys.argv[1:])


if __name__ == '__main__':
  main()
