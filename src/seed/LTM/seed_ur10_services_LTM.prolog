% Hardware service backend: gripper-only pick/place, explicit arm travel.
:- multifile schema/4.
:- ensure_loaded('seed_test_LTM.prolog').

schema(move_a_b(Location), [
    [rosAct(move_a_b(Location),ur10_services,ur10/hardware/primitives/command,0.25),0,[-manipulation.failed]]
], [arm.at(Location)], []).

schema(pick(Object,Location), [
    [rosAct(pick(Object,Location),ur10_services,ur10/hardware/primitives/command,0.25),0,[-manipulation.failed]]
], [object.held(Object)], []).

schema(place(Object,Location), [
    [rosAct(place(Object,Location),ur10_services,ur10/hardware/primitives/command,0.25),0,[-manipulation.failed]]
], [object.placed(Object,Location)], []).

hardware_service_steps([
    move_a_b(pick_approach), move_a_b(pick), pick(workpiece,pick),
    move_a_b(pick_approach), move_a_b(place_approach), move_a_b(place),
    place(workpiece,place), move_a_b(place_approach)
]).

schema(hardware_pick_place_demo, [[hardSequence(Steps),0,["TRUE"]]],
    [hardSequence(Steps).done], []) :- hardware_service_steps(Steps).
