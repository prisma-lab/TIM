%% NOTE:
% in this version it is possible to have a good invesion of the stack task.
% An inverted stack has been added to this LTM and it has been successfully executed

:-op(500,xfy,.).
:-op(600,fx,-).

%this is to allow natural language commands to be used as tasks!
:-op(550,xfy,\).

%SWI-prolog: in SWI the '.' operator is already used and constrained to take "dict" arguments 
%		..we have to replace it!
:-op(500,xfy,*.*).
.(L, R, L *.* R).

%this means that the following list is a sequence!
:-op(500,fx,$).


%allow schemata to be asserted
:-dynamic schema/4.


%%% %%% %%% LTM utils %%% %%% %%%

%get the level of abstraction of a schema (ie. max_tree_depth-1)
absLevel(S,0):-schema(S,[]). %concrete
absLevel(S,N):-schema(S,L), maxAbsLevel(L,LN),N is LN+1.
%maxAbsLevel([],0).
maxAbsLevel([[SS,_,_]|Rest],RN):-maxAbsLevel(Rest,RN),absLevel(SS,SSN),RN>=SSN.
maxAbsLevel([[SS,_,_]|_],SSN):-absLevel(SS,SSN).

%get the list of sub-schemata SSL for the schema S (ie. discard releasers and rtms)
subSchemaList([[SS,_,_]|Rest],[SS|SSRest]):-subSchemaList(Rest,SSRest).
subSchemaList([[SS,_,_]],[SS]).
subSchemaList(S,SSL):-schema(S,List),subSchemaList(List,SSL).



%Server

getSemantics(X):-
	yield([],_),
	schema(X,SemDef,_),
	yield(SemDef,_),!.

%added 02/12/2020 in SEED 4.0
getGoal(X):-
	yield([],_),
	schema(X,_,[],_),!,false.

getGoal(X):-
	yield([],_),
	schema(X,_,Goal,_),
	yield(Goal,_),!.
	
%added 08/04/2022 in SEED 5.0
getRegulations(X):-
	yield([],_),
	schema(X,_,_,[]),!,false.
	
getRegulations(X):-
	yield([],_),
	schema(X,_,_,Regulations),
	yield(Regulations,_),!.
	

% Execute schema only ONCE (i.e., forget it on accomplishment)
schema( once(Instance), [[hardSequence([Instance, iiwaWait(iiwaOnce,1), forget(once(Instance))]), 0, ["TRUE"]]], [], [] ).


% Structure of a Schema:
%	a schema has name, list of sub-schemata and goal, all within the schema()
%	predicate. The name can countains parameters that are propagated to the 
%	list and the goal (unified). Each sub-schema countains a default emphasis value
%	and a list of enabling variables (releaer). If the goal is not present, the schema
%	is not teleological.
%		NOTE: the goal within the schema predicate was added 02/12/2020 in SEED 4.0.
%		NOTE: the regulations within the schema predicate was added 08/04/2022 in SEED 5.0.
% eg:
%	schema( name(Param), [[subschema1(Param),emph,[releaserList]], [subschema2...]...], goal, regulations ).

%% ALIVE SCHEMA

%alive: this is the actual SEED loaded by default in the WM. The subnodes of alive are loaded on start 
schema(alive, [
	[inputStream,0,["TRUE"]],
	[rosStream,0,["TRUE"]],
	[show(alive),0,["TRUE"]],
	[memory,0,["TRUE"]],
	[wsg50Manager,0,["TRUE"]],
	[iiwaManager,0,["TRUE"]],
	%[plan,0,["TRUE"]],
	[tfobserver,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	[],
	[] ).
	
%% ABSTRACT SCHEMATA

%add q to kill SEED
schema( q, [[forget(alive),0, ["TRUE"]]], [], [] ).

% IIWA tasks (abstract)

schema( iiwa\teach\Act, [
	[iiwaGo(teach),1,["TRUE"]],
	[iiwaTeach(Act),0,[teach.near]] ] , 
	[Act.known], 
	[] ).

schema( iiwa\try\Act\M, [
	[iiwaExe(Act,M),0,[Act.known]],
	[iiwa\teach\Act,0,[-Act.known]] ], 
	[iiwaExe(Act,M).done], 
	[] ).

schema(k, [[set(teaching.done,true),0,["TRUE"]]], [], [] ).

% --




%% CONCRETE SCHEMATA

%forget(X): remove the node X from the WM
schema(forget(_), [], [], [] ).

%remember(X,Y): add the node X to the Y node in WM
schema(remember(_,_), [], [], [] ).
%remember(X): add the node X to the "alive" node in WM (default)
schema(remember(_), [], [], [] ).

%inputStream: enable commands from keyboard
schema(inputStream, [], [], [] ).

%listing: plot all nodes of the WM
schema(listing, [], [], [] ).

%show(X): publish on the topic /seed_SEEDNAME/show the image of the WM (graphviz based)
schema(show(_), [], [], [] ).
%show(X,less): publish a compact version of the WM (only enabled nodes are plot)
schema(show(_,less), [], [], [] ).

%gui: open the Graphical User Interface of the SEED (QT based)
schema(gui, [], [], [] ).

%requestStream: standard anchestor for the requested nodes
schema(requestStream, [], [], [] ).

%test: behavior used for node-testing
schema(test, [], [], [] ).

%rosStream: interface with the ROS environment (enables topics and services)
schema(rosStream, [], [], [] ).

%joyStream: interface with the Joypad
schema(joyStream, [], [], [] ).

%ltm(X): post a SwiProlog Query on this file
schema(ltm(_), [], [], [] ).

%set(X,V,P): set the WM variable X to a value V with a specific period P
schema(set(_,_,_), [], [], [] ).
%set(X,V): set the WM variable X to a value V only once
schema(set(_,_), [], [], [] ).

%get(X,T,P): get the WM variable X of type T with a specific period P
schema(get(_,_,_), [], [], [] ).
%get(X,T): get the WM variable X of type T only once
schema(get(_,_), [], [], [] ).

%timer(X,V,W): set the WM variable X to a value V after W seconds (waiting time)
schema(timer(V,true,_), [], [V], [] ).
schema(timer(V,false,_), [], [-V], [] ).
schema(timer(_,_,_), [], [], [] ).

%compete(X,T,V,P): compete to write on a variable X of type T the value V every period P
schema(compete(_,_,_,_), [], [], [] ).
%compete(X,T,V): compete to write on a variable X of type T the value V only once
schema(compete(_,_,_), [], [], [] ).

%solve(X,T,P): solve the competition every P seconds for a variable X of type T (result is plotted in blue)
schema(solve(_,_,_), [], [], [] ).
%solve(X,T): solve the competition only once for a variable X of type T (result is plotted in blue)
schema(solve(_,_), [], [], [] ).

% Added in SEED 6.0
%solve(X,T,R,P): solve the competition every P seconds for a variable X of type T, publish the result on topic R
schema(rosSolve(_,_,_,_), [], [], [] ).
%solve(X,T,R): solve the competition only once for a variable X of type T, publish the result on topic R
schema(rosSolve(_,_,_), [], [], [] ).

%rosAct(ACTION_ID, ACTOR, TOPIC, RATE)
schema(rosAct(_,_,_,_), [], [], [] ).
%rosAct(ACTION_ID, ACTOR, TOPIC)
schema(rosAct(_,_,_), [], [], [] ).

%state(V,R): set to TRUE the value of a variable V and communicate it through the topic R
schema(rosState(_,_), [], [], [] ).

%template(MSG): template behavior used to show how to add behaviors in SEED
schema(template(_), [], [], [] ).


%hardSequence(TASK_LIST): sequential behavior used to implement HARD sequence in SEED
schema(hardSequence(_,ID), [], [hardSequence(ID).done], [] ).

%softSequence(TASK_LIST): sequential behavior used to implement SOFT sequence in SEED
schema(softSequence(_,ID), [], [softSequence(ID).done], [] ).



% IIWA tasks (concrete)
schema(iiwaManager, [], [], [] ).

schema(iiwaGo(F1,F2), [], [iiwaGo(F1,F2).done], [] ).

schema(iiwaTeach(Motion), [], [iiwaTeach(Motion).done], [] ).

schema(iiwaExe(Motion,MXY), [], [iiwaExe(Motion,MXY).done], [] ).

schema(iiwaWrite(Word,Frame), [], [iiwaWrite(Word,Frame).done], [] ).

schema(iiwaInsert(Obj1,Frame), [], [inserted(Obj1,Frame)], [] ).

schema(iiwaExtract(Obj1,Frame), [], [extracted(Obj1,Frame)], [] ).


%INVERSE interfaces

%Interface with BOUN planner
schema(plan, [], [], [] ).

%tfObserver
schema(tfobserver, [], [], [] ).



% IIWA tasks (abstract)

schema(iiwaWait(Event), [
	[timer(Event.done,true,3),0,["TRUE"] ] ],
	[Event.done],
	[]).

schema(iiwaWait(Event,T), [
	[timer(Event.done,true,T),0,["TRUE"] ] ],
	[Event.done],
	[]).

schema(iiwaTest1, [
	[hardSequence([%iiwaGo(robot_bottle.1),iiwaWait(w1),
			iiwaGo(robot_bottle.2),iiwaWait(w2),
			iiwaGo(robot_bottle.3),iiwaWait(w3),
			wsg50Grasp(bottle),
			iiwaGo(test1_final)
				]), 0, ["TRUE"] ] ], 
	[], []).
	
schema(iiwaTest2, [
	[hardSequence([%iiwaGo(robot_bottle.1),iiwaWait(w1),
			iiwaGo(robot_bottle_cup.2),iiwaWait(w5),
			iiwaGo(robot_bottle_cup.3),iiwaWait(w6),
			iiwaWait(pouring)]), 0, ["TRUE"] ] ], 
	[], []).


schema(iiwaTest3, [
	[hardSequence([%iiwaGo(robot_bottle_bowl.1),iiwaWait(w4),
			iiwaGo(robot_bottle_bowl.2),iiwaWait(w5),
			iiwaGo(robot_bottle_bowl.3),iiwaWait(w6),
			wsg50Release(bottle)]), 0, ["TRUE"] ] ], 
	[], []).



%example IIWA-blocksworld domain:
%	NOTE: BOUN planner lowercases everithing!
%		iiwaUnstack -> iiwaunstack
%		iiwaStack -> iiwastack
%		iiwaPick -> iiwapick
%		iiwaPut -> iiwaput

%schema(iiwaTower(Obj1,Obj2,Obj3), [
%	[hardSequence([iiwastack(Obj2,Obj1),iiwastack(Obj3,Obj2)]), 0, ["TRUE"] ] ], 
%	[], []).

schema(iiwaunstack(Obj1,Obj2), [
	[hardSequence([iiwaDetach(Obj1,Obj2),iiwaWait(w0)],iiwaunstack.Obj1.Obj2), 0, ["TRUE"] ] ], 
	[], []).

schema(iiwastack(Obj1,Obj2), [
	[hardSequence([iiwaWait(w0),iiwaAttach(Obj1,Obj2)],iiwastack.Obj1.Obj2), 0, ["TRUE"] ] ], 
	[on(Obj1,Obj2)], []).

schema(iiwapick(Obj1,Obj2), [
	[hardSequence([iiwaGo(any,Obj1.pre),iiwaGo(Obj1.pre,Obj1),iiwaWait(w1),wsg50Grasp(Obj1),iiwaGo(Obj1,home),timer(Obj1.taken,true,0.1)],iiwapick.Obj1.Obj2), 0, ["TRUE"] ] ], 
	[Obj1.taken], []).
%schema(iiwaPick(Obj1,Obj2), [
%	[hardSequence([iiwaGo(Obj1.pre),iiwaGo(Obj1),iiwaWait(w1),wsg50Grasp(Obj1),iiwaGo(Obj2.pre),iiwaGo(home),timer(Obj1.taken,true,0.1)]), 0, ["TRUE"] ] ], 
%	[Obj1.taken], []).
	
schema(iiwaput(Obj1,Obj2), [
	[hardSequence([iiwaGo(any,Obj2.pre),iiwaGo(Obj2.pre,Obj2.over),iiwaWait(w3),wsg50Release(Obj1),iiwaGo(home),timer(on(Obj1,Obj2),true,0.1)],iiwaput.Obj1.Obj2), 0, ["TRUE"] ] ], 
	[on(Obj1,Obj2)], []).



schema(iiwaAttach(Obj1,Obj2), [
	[hardSequence([iiwaGo(any,Obj2.pre),iiwaWait(w2),iiwaInsert(Obj1,Obj2.over),iiwaWait(w3),wsg50Release(Obj1),iiwaGo(Obj2.over,home),timer(on(Obj1,Obj2),true,0.1)],iiwaAttach.Obj1.Obj2), 0, ["TRUE"] ] ], 
	[on(Obj1,Obj2)], []).
	
schema(iiwaDetach(Obj1,Obj2), [
	[hardSequence([iiwaGo(any,Obj1.pre),iiwaGo(Obj1.pre,Obj1),iiwaWait(w2),wsg50Grasp(Obj1),iiwaExtract(Obj1,Obj2.pre),iiwaWait(w3),iiwaGo(Obj2.pre,home),timer(Obj1.taken,true,0.1)],iiwaDetach.Obj1.Obj2), 0, ["TRUE"] ] ], 
	[Obj1.taken], []).


% --


% vision
schema(visionStream, [], [], [] ).


% WSG50 tasks (concrete)

schema(wsg50Manager, [], [], [] ).

schema(wsg50Grasp(Obj), [], [gripper.hold(Obj)], [] ).

schema(wsg50Release(Obj), [], [-gripper.hold(Obj)], [] ).

% --





% -- TASKS AUTOMATICALLY GENERATED FROM INVERSION PROCESS!!

% result of invert(iiwaStack(brick21,brick20),INV) modifying  
%	1. iiwaGo(home,brick21) -> iiwaGo(home,rnd)
%	2. task GOAL set as GOAL of the last action in sequence
schema(-iiwaPick(brick21),
	[[hardSequence([
		timer(brick21.taken,false,0.1),
		iiwaGo(home,rnd), %iiwaGo(home,brick21),
		wsg50Release(brick21),
		iiwaWait(w1),
		iiwaGo(brick21,brick21.pre),
		iiwaGo(brick21.pre,home)]),0,["TRUE"]]],[iiwaGo(brick21.pre, home).done],[]).

schema(-iiwaAttach(brick21,brick20),
	[[hardSequence([
		timer(on(brick21,brick20),false,0.1),
		iiwaGo(home,brick20.over),
		wsg50Grasp(brick21),
		iiwaWait(w3),
		iiwaExtract(brick21,brick20.over),
		iiwaWait(w2),
		iiwaGo(brick20.pre,home)]),0,["TRUE"]]],[iiwaGo(brick20.pre, home).done],[]).


schema(-iiwaStack(brick21,brick20),
	[[hardSequence([
		-iiwaAttach(brick21,brick20),
		iiwaWait(w0),
		-iiwaPick(brick21)]),0,["TRUE"]]],[iiwaGo(brick21.pre, home).done],[]).

% --






% INVERSION OPERATOR (-)

%schema(-S_dir, Sub, Goal, Reg) :- inverse(S_dir, S_inv), schema(S_inv, Sub, Goal, Reg).

%schema(-S_dir, Sub_inv, Goal, Reg) :- schema(S_dir, Sub_dir, Goal, Reg), apply_inversion(Sub_dir, Sub_inv).

%schema(-S_dir, [[teach(-S_dir), 0, ["TRUE"]]], Goal, Reg) :- schema(S_dir, [], Goal, Reg).



apply_inversion([], []).

apply_inversion([[Sch_dir, _, _]|Rest_dir], Sub_inv):- inverse(Sch_dir, Sch_inv), apply_inversion(Rest_dir,Rest_inv), append(Rest_inv, [[Sch_inv,0,["TRUE"]]], Sub_inv).

%apply_inversion([[Sch_dir, Emp_dir, Rel_dir]|Rest_dir], Sub_inv):- schema(-Sch_dir, New_sub_inv, _, _), write(-Sch_dir), write("inverted as"), write(New_sub_inv), apply_inversion(Rest_dir,Rest_inv), append(Rest_inv, [[-Sch_dir,Emp_dir,Rel_dir]], Sub_inv).


% INVERTED TASKS DATABASE

inverse(S_dir, -S_dir):- schema(-S_dir,_,_,_).

inverse(wsg50Grasp(Obj),wsg50Release(Obj)).
inverse(wsg50Release(Obj),wsg50Grasp(Obj)).

inverse(iiwaGo(any,F2),iiwaGo(F2,home)).

inverse(iiwaGo(home,F2),iiwaGo(F2,any)).

inverse(iiwaGo(F1,F2),iiwaGo(F2,F1)).

inverse(iiwaInsert(Obj1,Obj2),iiwaExtract(Obj1,Obj2)).
inverse(iiwaExtract(Obj1,Obj2), iiwaInsert(Obj1,Obj2)).

inverse(timer(Var,true,T),timer(Var,false,T)).
inverse(timer(Var,false,T),timer(Var,true,T)).

inverse(iiwaWait(X),iiwaWait(X)).

inverse(hardSequence([]), hardSequence([])).
inverse(hardSequence([H_dir|R_dir]), hardSequence(Seq_inv)) :- inverse(H_dir, H_inv), inverse(hardSequence(R_dir), hardSequence(R_inv)), append(R_inv,[H_inv],Seq_inv).


inverse(S_dir, S_inv) :- invert(S_dir, S_inv).


invert(S_dir, -S_dir) :- schema(S_dir, Sub_dir, _, _), apply_inversion(Sub_dir, Sub_inv), assertz(schema(-S_dir,Sub_inv,[],[])), write("NEW: "), write(schema(-S_dir,Sub_inv,[],[])), nl.

% EXEAMPLE CALL:
% 	invert(iiwaStack(brick21,brick20),INV).










