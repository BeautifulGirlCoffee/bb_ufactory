# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

[
  tools: [
    {:credo, "mix credo --strict"},
    {:excoveralls, "mix coveralls"},
    {:reuse, command: ["pipx", "run", "reuse", "lint", "-q"]}
  ]
]
