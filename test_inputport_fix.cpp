/*
 * Simple test to demonstrate the InputPort template parameter fix
 * This test shows how InputPort declarations should have explicit template parameters
 * to avoid the "static assertion failed: The default value must be either the same of the port or string" error
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <iostream>

using namespace BT;

// Example Position3D struct (simplified version)
struct Position3D {
    double x, y, z;
    Position3D() : x(0), y(0), z(0) {}
    Position3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

template <>
inline Position3D convertFromString(StringView str) {
    auto parts = splitString(str, ';');
    if (parts.size() != 3) {
        throw RuntimeError("Invalid Position3D format");
    }
    Position3D result;
    result.x = convertFromString<double>(parts[0]);
    result.y = convertFromString<double>(parts[1]);
    result.z = convertFromString<double>(parts[2]);
    return result;
}

class TestAction : public SyncActionNode {
public:
    TestAction(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    // CORRECT way - with explicit template parameters
    static PortsList providedPorts() {
        return {
            InputPort<std::string>("object_name", "Object to target"),
            InputPort<int>("method", "Action method"),
            InputPort<double>("distance", "Target distance"),
            InputPort<bool>("enabled", "Action enabled"),
            InputPort<Position3D>("position", "Target position")
        };
    }

    NodeStatus tick() override {
        auto name = getInput<std::string>("object_name");
        auto method = getInput<int>("method");
        auto distance = getInput<double>("distance");
        auto enabled = getInput<bool>("enabled");
        auto position = getInput<Position3D>("position");

        if (!name || !method || !distance || !enabled || !position) {
            return NodeStatus::FAILURE;
        }

        std::cout << "Action executed with:\n";
        std::cout << "  Object: " << name.value() << "\n";
        std::cout << "  Method: " << method.value() << "\n";
        std::cout << "  Distance: " << distance.value() << "\n";
        std::cout << "  Enabled: " << enabled.value() << "\n";
        std::cout << "  Position: (" << position.value().x << ", " 
                  << position.value().y << ", " << position.value().z << ")\n";

        return NodeStatus::SUCCESS;
    }
};

/*
// INCORRECT way (this would cause the static assertion error):
class BadTestAction : public SyncActionNode {
public:
    BadTestAction(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {}

    static PortsList providedPorts() {
        return {
            InputPort("object_name", "Object to target"),  // No template parameter!
            InputPort("method", "Action method"),           // No template parameter!
            InputPort("distance", "Target distance"),       // No template parameter!
            InputPort("enabled", "Action enabled"),         // No template parameter!
            InputPort("position", "Target position")        // No template parameter!
        };
    }
    // ... rest would be the same
};
*/

int main() {
    BehaviorTreeFactory factory;
    factory.registerNodeType<TestAction>("TestAction");

    auto tree_xml = R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="TestTree">
                <TestAction object_name="target1" 
                           method="1" 
                           distance="5.0" 
                           enabled="true" 
                           position="1.0;2.0;3.0"/>
            </BehaviorTree>
        </root>
    )";

    try {
        auto tree = factory.createTreeFromText(tree_xml);
        std::cout << "Tree created successfully!\n";
        
        auto status = tree.tickWhileRunning();
        std::cout << "Tree execution status: " << toStr(status) << "\n";
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}