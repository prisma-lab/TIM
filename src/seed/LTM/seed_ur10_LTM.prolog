% Reuse the console, ROS interface and sequencing behaviors from the test LTM.
:- multifile schema/4.
:- ensure_loaded('seed_test_LTM.prolog').

% A releaser enables command publication until the parent goal is satisfied.
% The plugin manager reports these states from controller results and
% fresh joint feedback. A failed controller action inhibits automatic retries.
schema(open_gripper, [
    [rosAct(open_gripper,ur10_gripper,ur10/gripper/command,0.25),0,[-gripper.failed]]
], [gripper.open], []).

schema(close_gripper, [
    [rosAct(close_gripper,ur10_gripper,ur10/gripper/command,0.25),0,[-gripper.failed]]
], [gripper.closed], []).

schema(gripper_demo, [
    [hardSequence([close_gripper,open_gripper]),0,["TRUE"]]
], [hardSequence([close_gripper,open_gripper]).done], []).

% Numeric poses stay below SEED. The symbolic Target selects a pose topic.
schema(move_a_b(Target), [
    [rosAct(move_a_b(Target),ur10_manipulation,ur10/primitives/command,0.25),0,[-manipulation.failed]]
], [arm.at(Target)], []).

schema(pick, [
    [rosAct(pick,ur10_manipulation,ur10/primitives/command,0.25),0,[-manipulation.failed]]
], [object.held], []).

schema(place, [
    [rosAct(place,ur10_manipulation,ur10/primitives/command,0.25),0,[-manipulation.failed]]
], [object.placed], []).

% SEED has a plan behavior, too. Given domain&problem.pddl, it automatically generates this hardSequence
% Assuming that the components are already implemented as schemas, like above, the rest just executes.
schema(pick_place_demo, [
    [hardSequence([move_a_b(pick),pick,move_a_b(place),place]),0,["TRUE"]]
], [hardSequence([move_a_b(pick),pick,move_a_b(place),place]).done], []).
