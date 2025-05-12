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
	%[show(alive),0,["TRUE"]],
	[memory,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	[],
	[] ).
	
%% ABSTRACT SCHEMATA

% test to check if complex formulas are working)
schema(rel\X, [
	[dummy(X),0,[val1(X) >= 1.0, val2(X) < -1.0, bool1(X), val3(X) == val4(X)]] ],
	[],
	[] ).

%Example of a SOFT sequential behavior
% seq(SEQ_NAME): sequential behavior used to implement soft sequence in SEED
schema(soft\sequence, [
	[softSequence([dummy(1),dummy(2),dummy(3)]),0,["TRUE"]] ],
	[],
	[] ).

% DEPRECATED VERSION
%schema(seq(dummy), [
%	[dummy(1),0,[dummy(1).rel]],
%	[dummy(2),0,[dummy(2).rel]],
%	[dummy(3),0,[dummy(3).rel]] ],
%	[],
%	[] ).

%Example of a HARD sequential behavior (HERE BACKTRACKING IS IMPOSSIBLE)
% $ operator is used to implement hard sequence in SEED
schema(hard\sequence, [
	[hardSequence([dummy(1),dummy(2),dummy(3)]),0,["TRUE"]] ],
	[],
	[] ).

% DEPRECATED VERSION
%schema(hardSeq, $[
%	[dummy(1),0,[dummy(1).rel]],
%	[dummy(2),0,[dummy(2).rel]],
%	[dummy(3),0,[dummy(3).rel]] ],
%	[],
%	[] ).

%add q to kill SEED
schema( q, [[forget(alive),0, ["TRUE"]]], [], [] ).

% test for SEED competition
schema(seed\semaphore, [
	[solve(seed.winner,string,1),0,["TRUE"]],
	[semaphore\task,0,["TRUE"]] ],
	[],
	[] ).
	
schema(semaphore\task, [
	[compete(seed.winner,string,yellow,0.1),0,["TRUE"]],
	[compete(seed.winner,string,green,0.1),0.1,[semaphore.green]],
	[compete(seed.winner,string,red,0.1),0.1,[semaphore.red]] ],
	[],
	[semaphore] ).
	
schema(compete(_,string,X,_), [], [], [color(X)] ).
% --


% test for SEED competition and ROS2 interface with act (self-solving) behaviors
schema(ros2\semaphore, [
	[ros2\semaphore\green\for\5,0,["TRUE"]],
	[ros2\semaphore\yellow\for\3,0.1,[green.done]],
	[ros2\semaphore\red\for\5,0,[yellow.done]],
	[ros2\semaphore\restart,0,[red.done]] ],
	[],
	[] ).

schema(ros2\semaphore\COLOR\for\SECONDS, [
	[timer(COLOR.done,true,SECONDS),0,["TRUE"]],
	[rosAct(COLOR,semaphore,semaphore/out,0.1),0,["TRUE"]] ],
	[COLOR.done],
	[] ).

schema(ros2\semaphore\restart, [
	[timer(green.done,false,0.1),0,["TRUE"]],
	[timer(yellow.done,false,0.1),0,[-green.done]],
	[timer(red.done,false,0.1),0,[-yellow.done]] ],
	[],
	[] ).
	
%schema(rosAct(X,_,_,_), [], [], [color(X)] ).
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

%hardSequence(TASK_LIST): sequential behavior used to implement HARD sequence in SEED
schema(hardSequence(T), [], [hardSequence(T).done], [] ).

%softSequence(TASK_LIST): sequential behavior used to implement SOFT sequence in SEED
schema(softSequence(T), [], [softSequence(T).done], [] ).

%dummy(X): dummy behavior, this does nothing, just used for testing
schema(dummy(X), [], [dummy(X).goal], [] ).

%template(MSG): template behavior used to show how to add behaviors in SEED
schema(template(_), [], [], [] ).



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
