# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

# Architecture policy for `mix reach.check --arch`.
#
# The invariant worth machine-checking: the wire layer (Protocol, Report,
# Registers, Model) is pure encode/decode. It must never reach up into the
# runtime components, message structs, or a TCP socket — that purity is what
# keeps the whole protocol unit-testable without hardware.
[
  layers: [
    protocol: [
      "BB.Ufactory.Protocol",
      "BB.Ufactory.Report",
      "BB.Ufactory.Registers",
      "BB.Ufactory.Model"
    ],
    messages: ["BB.Ufactory.Message.*"],
    errors: ["BB.Error.Protocol.Ufactory.*"],
    components: [
      "BB.Ufactory.Controller",
      "BB.Ufactory.Actuator.*",
      "BB.Ufactory.Sensor.*"
    ],
    robots: ["BB.Ufactory.Robots.*"]
  ],
  deps: [
    forbidden: [
      {:protocol, :components},
      {:protocol, :messages},
      {:protocol, :errors},
      {:messages, :components},
      {:errors, :components}
    ]
  ],
  calls: [
    forbidden: [
      # Only runtime components and the simulator tooling may touch sockets.
      {"BB.Ufactory.Protocol", [":gen_tcp.send", ":gen_tcp.connect", ":gen_tcp.recv"]},
      {"BB.Ufactory.Report", [":gen_tcp.send", ":gen_tcp.connect", ":gen_tcp.recv"]},
      {"BB.Ufactory.Model", [":gen_tcp.send", ":gen_tcp.connect", ":gen_tcp.recv"]}
    ]
  ]
]
