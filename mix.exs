# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.MixProject do
  use Mix.Project

  @moduledoc """
  Beam Bots integration for UFactory xArm robotic arms.
  """

  @version "0.1.0"

  def project do
    [
      aliases: aliases(),
      app: :bb_ufactory,
      consolidate_protocols: Mix.env() == :prod,
      deps: deps(),
      description: @moduledoc,
      dialyzer: dialyzer(),
      docs: docs(),
      elixir: "~> 1.19",
      elixirc_paths: elixirc_paths(Mix.env()),
      package: package(),
      start_permanent: Mix.env() == :prod,
      version: @version
    ]
  end

  defp dialyzer, do: []

  defp package do
    [
      maintainers: ["Holden Oullette"],
      licenses: ["Apache-2.0"],
      links: %{
        "Source" => "https://github.com/BeautifulGirlCoffee/bb_ufactory",
        "Website" => "https://beautifulgirl.coffee"
      }
    ]
  end

  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp docs do
    [
      main: "readme",
      extras:
        ["README.md", "CHANGELOG.md"]
        |> Enum.concat(Path.wildcard("documentation/**/*.{md,livemd,cheatmd}")),
      groups_for_extras: [
        Tutorials: ~r/tutorials\//
      ],
      source_ref: "main",
      source_url: "https://github.com/BeautifulGirlCoffee/bb_ufactory"
    ]
  end

  defp aliases, do: []

  defp deps do
    [
      {:bb, "~> 0.15"},

      # dev/test
      {:credo, "~> 1.7", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.4", only: [:dev, :test], runtime: false},
      {:doctor, "~> 0.22", only: [:dev, :test], runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:ex_doc, ">= 0.0.0", only: [:dev, :test], runtime: false},
      {:git_ops, "~> 2.9", only: [:dev, :test], runtime: false},
      {:igniter, "~> 0.7", only: [:dev, :test], runtime: false},
      {:mimic, "~> 2.3", only: :test, runtime: false},
      {:mix_audit, "~> 2.1", only: [:dev, :test], runtime: false},
      {:sobelow, "~> 0.14", only: [:dev, :test], runtime: false}
    ]
  end

  defp elixirc_paths(env) when env in [:dev, :test], do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]
end
