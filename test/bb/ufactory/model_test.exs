# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ModelTest do
  use ExUnit.Case, async: true

  alias BB.Ufactory.Model

  describe "get/1" do
    test "returns config map with joints, limits, and max_speed_rads for xarm6" do
      config = Model.get(:xarm6)
      assert config.joints == 6
      assert config.max_speed_rads == :math.pi()
      assert length(config.limits) == 6
    end

    test "raises KeyError for unknown model" do
      assert_raise KeyError, fn -> Model.get(:unknown) end
    end
  end

  describe "joint_count/1" do
    test "returns 5 for xarm5" do
      assert Model.joint_count(:xarm5) == 5
    end

    test "returns 6 for xarm6" do
      assert Model.joint_count(:xarm6) == 6
    end

    test "returns 7 for xarm7" do
      assert Model.joint_count(:xarm7) == 7
    end

    test "returns 6 for lite6" do
      assert Model.joint_count(:lite6) == 6
    end

    test "returns 6 for xarm850" do
      assert Model.joint_count(:xarm850) == 6
    end
  end

  describe "max_speed_rads/1" do
    test "returns π for all models" do
      for model <- Model.supported_models() do
        assert Model.max_speed_rads(model) == :math.pi()
      end
    end
  end

  describe "joint_limits/1" do
    test "returns correct number of limit tuples per model" do
      for model <- Model.supported_models() do
        limits = Model.joint_limits(model)
        assert length(limits) == Model.joint_count(model)

        Enum.each(limits, fn {lower, upper} ->
          assert is_float(lower)
          assert is_float(upper)
          assert lower < upper
        end)
      end
    end
  end

  describe "supported_models/0" do
    test "includes all five known models" do
      models = Model.supported_models()
      assert :xarm5 in models
      assert :xarm6 in models
      assert :xarm7 in models
      assert :lite6 in models
      assert :xarm850 in models
      assert length(models) == 5
    end
  end
end
