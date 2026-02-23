# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

[
  tools: [
    {:credo, "mix credo --strict"},
    {:excoveralls, "mix coveralls"},
    {:reuse, command: ["docker", "run", "--rm", "-v", "#{File.cwd!()}:/data", "fsfe/reuse", "lint"]}
  ]
]
