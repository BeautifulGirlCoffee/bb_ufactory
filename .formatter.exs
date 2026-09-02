# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

# Used by "mix format". The Spark DSL locals (actuator, joint, limit, ...)
# come from bb's exported .formatter.exs (bb >= 0.26) instead of a
# hand-maintained copy that drifts as the DSL grows.
[
  import_deps: [:bb],
  inputs: ["{mix,.formatter}.exs", "{config,lib,test}/**/*.{ex,exs}"],
  plugins: [Spark.Formatter]
]
