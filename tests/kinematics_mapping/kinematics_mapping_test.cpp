#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>

using LimbMap  = std::map<std::string,std::vector<std::string>>;
using LimbList = std::vector<LimbMap>;

LimbList loadLimbsDefinition(const YAML::Node& root)
{
    LimbList limbs_definition;

    if (!root["limbs"] || !root["limbs"].IsSequence()) {
        throw std::runtime_error("YAML must contain a 'limbs' sequence");
    }

    for (const auto& limb_node : root["limbs"]) {
        LimbMap limb_def;

        // limb short name, e.g. "LF", "RF", ...
        const std::string limb_name = limb_node["name"].as<std::string>();
        limb_def["name"].push_back(limb_name);

        // type: e.g. "leg"
        if (limb_node["type"]) {
            limb_def["type"].push_back(limb_node["type"].as<std::string>());
        }

        // --- joints: build "LF_HAA", "LF_HFE", "LF_KFE", ...
        const YAML::Node& joints = limb_node["chain"]["joints"];
        if (joints && joints.IsSequence()) {
            for (const auto& j : joints) {
                // j["name"] is e.g. "HAA", "HFE", "KFE"
                std::string j_name = j["name"].as<std::string>();
                std::string j_name_urdf = j["urdf"].as<std::string>();
                std::string j_direction = j["direction"].as<std::string>();
                limb_def["dls_joints_name"].push_back(limb_name + "_" + j_name);
                limb_def["urdf_joints_name"].push_back(j_name_urdf);
                limb_def["joints_direction"].push_back(j_direction);
            }
        }

        // --- links: build "LF_ASSEMBLY", "LF_UPPERLERG", "LF_LOWERLEG", "LF_FOOT", ...
        const YAML::Node& links = limb_node["chain"]["links"];
        if (links && links.IsSequence()) {
            for (const auto& l : links) {
                std::string l_name = l["name"].as<std::string>();
                std::string l_name_urdf = l["urdf"].as<std::string>();
                limb_def["dls_links_name"].push_back(limb_name + "_" + l_name);
                limb_def["urdf_links_name"].push_back(l_name_urdf);
            }
        }

        limbs_definition.push_back(std::move(limb_def));
    }

    return limbs_definition;
}

int main()
{
    try {
        YAML::Node kinematics_mapping = YAML::LoadFile("/usr/include/aliengo_description/kinematics/kinematics.yaml");
        LimbList limbs_definition = loadLimbsDefinition(kinematics_mapping);

        // Print to verify structure
        for (const auto& limb : limbs_definition) {
            std::cout << "---- limb ----\n";
            for (const auto& kv : limb) {
                std::cout << kv.first << " : ";
                for (const auto& v : kv.second) {
                    std::cout << v << " ";
                }
                std::cout << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

// // format used for creating limbs
// std::vector<std::map<std::string,std::vector<std::string>>> limbs_definition {
//     {
//         {"name", {"LF"}},
//         {"dls_joints_name", {"LF_HAA", "LF_HFE", "LF_KFE"}},
//         {"dls_links_name",  {"LF_ASSEMBLY", "LF_UPPERLEG", "LF_LOWERLEG"}},
//         {"type", {"leg"}}
//     },
//     {
//         {"name", {"RF"}},
//         {"joints", {"RF_HAA", "RF_HFE", "RF_KFE"}},
//         {"links",  {"RF_ASSEMBLY", "RF_UPPERLEG", "RF_LOWERLEG"}},
//         {"type", {"leg"}}
//     },
//     {
//         {"name", {"LH"}},
//         {"joints", {"LH_HAA", "LH_HFE", "LH_KFE"}},
//         {"links",  {"LH_ASSEMBLY", "LH_UPPERLEG", "LH_LOWERLEG"}},
//         {"type", {"leg"}}
//     },
//     {
//         {"name", {"RH"}},
//         {"joints", {"RH_HAA", "RH_HFE", "RH_KFE"}},
//         {"links",  {"RH_ASSEMBLY", "RH_UPPERLEG", "RH_LOWERLEG"}},
//         {"type", {"leg"}}
//     }
// };
