# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

import Config

config :git_ops,
  mix_project: Mix.Project.get!(),
  changelog_file: "CHANGELOG.md",
  repository_url: "https://github.com/BeautifulGirlCoffee/bb_ufactory",
  manage_mix_version?: true,
  manage_readme_version: "README.md",
  version_tag_prefix: "v"
