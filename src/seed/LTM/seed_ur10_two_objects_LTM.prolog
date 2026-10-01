% Separate simulation profile. Run: ros2 run seed seed ur10_two_objects
:- multifile schema/4.
:- ensure_loaded('seed_test_LTM.prolog').

% Object and Location are symbolic parameters. Numeric poses stay in ROS.
schema(move_a_b(Location), [
    [rosAct(move_a_b(Location),ur10_two_objects,ur10/two_objects/command,0.25),0,[-manipulation.failed]]
], [arm.at(Location)], []).

schema(pick(Object,Location), [
    [rosAct(pick(Object,Location),ur10_two_objects,ur10/two_objects/command,0.25),0,[-manipulation.failed]]
], [object.held(Object)], []).

schema(place(Object,Location), [
    [rosAct(place(Object,Location),ur10_two_objects,ur10/two_objects/command,0.25),0,[-manipulation.failed]]
], [object.placed(Object,Location)], []).

two_object_steps([
    move_a_b(red_pick), pick(red_connector,red_pick),
    move_a_b(red_place), place(red_connector,red_place),
    move_a_b(blue_pick), pick(blue_peg,blue_pick),
    move_a_b(blue_place), place(blue_peg,blue_place)
]).

schema(two_objects_demo, [[hardSequence(Steps),0,["TRUE"]]],
    [hardSequence(Steps).done], []) :- two_object_steps(Steps).
