#ifndef BEHAVIOR_GUI_H
#define BEHAVIOR_GUI_H

#include "seed.h"
#include <nlohmann/json.hpp>

using namespace seed; //this is not needed to compile, byt most IDEs require it

class GUIBehavior : public Behavior {
public:
    GUIBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

protected:

    std::string saveWMtoJSON(WM_node *root)
    {

        wm_lock();

        if (WM == nullptr || dead())
        {
            wm_unlock();
            return "";
        }

        using nlohmann::json;
        json j;
        j["wm"] = json::array();

        std::function<void(WM_node *)> recurse = [&](WM_node *n)
        {
            if (!n)
                return "";

            json b;

            b["name"] = n->instance;
            b["schema"] = n->name;
            b["id"] = n->id;

            b["abstract"] = n->abstract;
            b["expanded"] = n->expanded;
            b["background"] = n->background;
            b["teleological"] = n->teleological;
            b["goalCount"] = n->goalCount;
            b["amplified"] = n->amplified;

            b["is_sequential"] = n->is_sequential;
            b["in_sequence"] = n->in_sequence;
            b["sequentialReleaser"] = n->sequentialReleaser;

            double emph = 0.0;
            try
            {
                emph = n->emphasis(true);
            }
            catch (...)
            {
            }
            b["emphasis"] = emph;

            b["fading"] = n->fading;

            json contrib = json::object();
            for (auto &p : n->contribution)
            {
                if (p.second != nullptr)
                    contrib[p.first] = *(p.second);
            }
            b["contribution"] = contrib;

            json weights_json = json::object();
            try
            {
                auto weights = seed::WMV.get<std::unordered_map<std::string, double *>>(n->name + ".weights");
                for (auto &w : weights)
                    if (w.second != nullptr)
                        weights_json[w.first] = *(w.second);
            }
            catch (...)
            {
            }
            b["weights"] = weights_json;

            bool rel = false, goal = false;
            try
            {
                rel = n->releaserStatus();
            }
            catch (...)
            {
            }
            try
            {
                goal = n->goalStatus();
            }
            catch (...)
            {
            }

            b["releaser"] = rel;
            b["goal"] = goal;

            b["releaser_formulae"] = n->releaser;
            b["goal_formulae"] = n->goal;

            bool truth = false;
            try
            {
                truth = seed::WMV.get<bool>(n->instance);
            }
            catch (...)
            {
            }
            b["truth"] = truth;

            b["is_working"] = n->isWorking();
            b["is_branch_released"] = n->isBranchReleased();
            b["is_awake"] = n->isAwake(n->instance);

            b["father"] = n->father ? n->father->instance : "none";

            std::vector<std::string> sons;
            for (auto *child : n->son)
                if (child)
                    sons.push_back(child->instance);
            b["sons"] = sons;

            j["wm"].push_back(b);

            for (auto *child : n->son)
                recurse(child);
        };

        recurse(root);

        wm_unlock();

        j["timestamp"] = std::time(nullptr);

        try
        {
            std::ofstream f(SEED_HOME_PATH + "/log/seed_wm.json");
            f << std::setw(2) << j;
            f.close();
        }
        catch (...)
        {
            std::cerr << "Cannot write JSON" << std::endl;
        }


        return j.dump();
    }

    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pb;
};

#endif	/* BEHAVIOR_GUI_H */

