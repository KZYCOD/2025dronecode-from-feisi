# BehaviorTree.CPP InputPort Type Mismatch Fix

## Problem Description

When compiling the codebase, BehaviorTree.CPP reported static assertion errors:

```
static assertion failed: The default value must be either the same of the port or string
```

This error occurred in multiple files when `InputPort` declarations were made without explicit template parameters.

## Root Cause

When declaring `InputPort("param_name", "description")` without template parameters, BehaviorTree.CPP defaults to `AnyTypeAllowed` for the port type. However, when these ports are used with specific typed values via `getInput<T>()`, there's a type mismatch that triggers the static assertion.

## Solution

Add explicit template parameters to all `InputPort` declarations to match the types used in corresponding `getInput<T>()` calls.

### Before (Incorrect):
```cpp
static PortsList providedPorts() {
    return {
        InputPort("object_name", "input object name"),      // Type deduced as AnyTypeAllowed
        InputPort("distance", "target distance"),           // Type deduced as AnyTypeAllowed  
        InputPort("enabled", "action enabled")              // Type deduced as AnyTypeAllowed
    };
}

NodeStatus onStart() override {
    auto name = getInput<std::string>("object_name");     // Expects std::string
    auto dist = getInput<double>("distance");             // Expects double
    auto enabled = getInput<bool>("enabled");             // Expects bool
    // ... static assertion failure occurs here
}
```

### After (Correct):
```cpp
static PortsList providedPorts() {
    return {
        InputPort<std::string>("object_name", "input object name"),    // Explicit std::string type
        InputPort<double>("distance", "target distance"),              // Explicit double type
        InputPort<bool>("enabled", "action enabled")                   // Explicit bool type
    };
}

NodeStatus onStart() override {
    auto name = getInput<std::string>("object_name");     // Types match!
    auto dist = getInput<double>("distance");             // Types match!
    auto enabled = getInput<bool>("enabled");             // Types match!
    // ... compilation succeeds
}
```

## Files Fixed

| File | Lines | Parameters Fixed |
|------|-------|------------------|
| `hit_direct.cpp` | 94-101 | object_name (string), method (int), hit_dist (double), goal_tolerance (double), needle_ang_adapt (double), left_line (int), right_line (int), yaw_err (double) |
| `plannode.cpp` | 33-40 | goal_position (Position3D), goal_ori (Position3D), enabel_planner (bool), planner_ctrl_type (int), goal_src (int), enabel_yaw (bool), enabel_yaw_rate (bool) |
| `land.cpp` | 19-20 | use_speed (int), speed_z (double) |
| `takeoff.cpp` | 18 | is_rc (int) |
| `calobjpos.cpp` | 174 | class_name (string) |
| `plannode_rviz.cpp` | 43-44 | enable_planner (bool), planner_ctrl_type (int) |
| `crossframe.cpp` | 111-113 | object_name (string), ctrl_type (int), ctrl_speed (double) |
| `detectobj.cpp` | 24 | class_name (string) |
| `goalyaw.cpp` | 26-29 | goal_yaw (double), is_set_point (bool), point (Position3D), yaw_rate (double) |
| `hit.cpp` | 84-91 | object_name (string), method (int), hit_dist (double), goal_tolerance (double), needle_ang_adapt (double), left_line (int), right_line (int), yaw_err (double) |

## Type Mapping Reference

| C++ Type | BehaviorTree Usage |
|----------|-------------------|
| `std::string` | `InputPort<std::string>` |
| `int` | `InputPort<int>` |
| `double` | `InputPort<double>` |
| `bool` | `InputPort<bool>` |
| `BT::Position3D` | `InputPort<Position3D>` |

## Verification

All `InputPort()` declarations without explicit template parameters in the mission package have been identified and fixed. The types were determined by analyzing the corresponding `getInput<T>()` calls in the implementation code.

To verify no issues remain:
```bash
# Search for remaining problematic declarations
find . -name "*.cpp" | xargs grep "InputPort(" | grep -v "InputPort<"
```

This should return no results in the mission package source files.