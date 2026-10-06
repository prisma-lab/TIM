% Hardware service backend: gripper-only pick/place, explicit arm travel.
:- multifile schema/4.
:- ensure_loaded('seed_test_LTM.prolog').

schema(move(Location), [
    [rosAct(move(Location),ur10_services,ur10/hardware/primitives/command,0.25),0,[-manipulation.failed]]
], [arm.at(Location)], []).

schema(pick, [
    [rosAct(pick,ur10_services,ur10/hardware/primitives/command,0.25),0,[-manipulation.failed]]
], [gripper.closed], []).

schema(place, [
    [rosAct(place,ur10_services,ur10/hardware/primitives/command,0.25),0,[-manipulation.failed]]
], [gripper.open], []).

hardware_service_steps([
    move(pick_approach), move(pick), pick,
    move(pick_approach), move(place_approach), move(place),
    place, move(place_approach)
]).

schema(hardware_pick_place_demo, [[hardSequence(Steps),0,["TRUE"]]],
    [hardSequence(Steps).done], []) :- hardware_service_steps(Steps).
