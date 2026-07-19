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
      test_coverage: [tool: ExCoveralls],
      version: @version
    ]
  end

  def cli do
    [
      preferred_envs: [coveralls: :test, "coveralls.detail": :test, "coveralls.html": :test]
    ]
  end

  # :mix and :ex_unit are needed in the PLT because the library ships a Mix
  # task (Mix.Tasks.BbUfactory.Sim) and an ExUnit case template
  # (BB.Ufactory.SimulatorCase).
  defp dialyzer, do: [plt_add_apps: [:mix, :ex_unit]]

  defp package do
    [
      # Public package owned by the beautifulgirlcoffee organization on
      # hex.pm (published via an org-owned key; the org is the package
      # owner, not a private repository).
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
      # Published docs link to the release tag (created by the publish CI
      # job); "main" would drift as the branch moves past the release.
      source_ref: "v#{@version}",
      source_url: "https://github.com/BeautifulGirlCoffee/bb_ufactory"
    ]
  end

  defp aliases, do: []

  defp deps do
    [
      # 0.22+ required: BB.Message monotonic_time/wall_time envelope,
      # compile-time component behaviour enforcement, Localize-based units.
      {:bb, "~> 0.22"},

      # dev/test
      {:credo, "~> 1.7", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.4", only: [:dev, :test], runtime: false},
      {:doctor, "~> 0.22", only: [:dev, :test], runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:excoveralls, "~> 0.18", only: :test},
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
