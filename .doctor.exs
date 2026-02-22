# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

%Doctor.Config{
  ignore_modules: [],
  ignore_paths: [],
  min_module_doc_coverage: 0,
  min_module_spec_coverage: 0,
  min_overall_doc_coverage: 0,
  min_overall_spec_coverage: 0,
  raise: true,
  reporter: Doctor.Reporters.Full,
  struct_type_spec_required: true,
  umbrella: false
}
