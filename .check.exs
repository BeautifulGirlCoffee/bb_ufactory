# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

[
  tools: [
    {:credo, "mix credo --strict"},
    # --warnings-as-errors also covers TEST-file compilation, which the
    # compiler task (lib only) cannot see.
    {:excoveralls, "mix coveralls --warnings-as-errors"},
    # Architecture policy (.reach.exs) + strict cross-function smell checks.
    {:reach, "mix reach.check --arch --smells --strict"},
    {:reuse, command: ["docker", "run", "--rm", "-v", "#{File.cwd!()}:/data", "fsfe/reuse", "lint"]}
  ]
]
