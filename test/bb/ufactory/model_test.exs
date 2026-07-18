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

    # Values from the official xarm_ros2 URDF descriptions
    # (xarm_description/urdf/<model>/<model>.urdf.xacro), which match the
    # Python SDK per-model configuration. Asserting exact values guards
    # against the copy-paste drift that previously shipped xArm6 limits for
    # every model (inverting xArm7's J4 range, among others).
    @two_pi 2 * :math.pi()
    @pi :math.pi()

    @expected_limits %{
      xarm5: [
        {-@two_pi, @two_pi},
        {-2.059488, 2.094395},
        {-3.926990, 0.191986},
        {-1.692969, @pi},
        {-@two_pi, @two_pi}
      ],
      xarm6: [
        {-@two_pi, @two_pi},
        {-2.059488, 2.094395},
        {-3.926990, 0.191986},
        {-@two_pi, @two_pi},
        {-1.692969, @pi},
        {-@two_pi, @two_pi}
      ],
      xarm7: [
        {-@two_pi, @two_pi},
        {-2.059488, 2.094395},
        {-@two_pi, @two_pi},
        {-0.191986, 3.926990},
        {-@two_pi, @two_pi},
        {-1.692969, @pi},
        {-@two_pi, @two_pi}
      ],
      lite6: [
        {-@two_pi, @two_pi},
        {-2.617990, 2.617990},
        {-0.061087, 5.235988},
        {-@two_pi, @two_pi},
        {-2.164200, 2.164200},
        {-@two_pi, @two_pi}
      ],
      xarm850: [
        {-@two_pi, @two_pi},
        {-2.303835, 2.303835},
        {-4.223697, 0.061087},
        {-@two_pi, @two_pi},
        {-2.164200, 2.164200},
        {-@two_pi, @two_pi}
      ]
    }

    for model <- Map.keys(@expected_limits) do
      test "#{model} limits match the official URDF values" do
        expected = @expected_limits[unquote(model)]
        actual = Model.joint_limits(unquote(model))

        for {{{el, eu}, {al, au}}, idx} <- Enum.with_index(Enum.zip(expected, actual), 1) do
          assert_in_delta al, el, 1.0e-6, "J#{idx} lower limit mismatch"
          assert_in_delta au, eu, 1.0e-6, "J#{idx} upper limit mismatch"
        end
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

  doctest BB.Ufactory.Model
end
