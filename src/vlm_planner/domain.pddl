(define (domain clip-assembly)
  (:requirements :strips :typing)
  (:types object location gripper)
  (:predicates
    (on-surface ?obj - object ?loc - location)
    (in-channel ?obj - object ?loc - location)
    (partially-hidden ?obj - object ?loc - location)
    (at-target ?obj - object ?loc - location)
    (holding ?g - gripper ?obj - object)
    (gripper-empty ?g - gripper)
    (gripper-at ?g - gripper ?loc - location))

  (:action slide
    :parameters (?obj - object ?g - gripper ?loc - location)
    :precondition (and (partially-hidden ?obj ?loc) (gripper-empty ?g)
                       (gripper-at ?g ?loc))
    :effect (and (on-surface ?obj ?loc) (not (partially-hidden ?obj ?loc))))

  (:action pick-up
    :parameters (?obj - object ?g - gripper ?loc - location)
    :precondition (and (on-surface ?obj ?loc) (gripper-empty ?g)
                       (gripper-at ?g ?loc))
    :effect (and (holding ?g ?obj) (not (gripper-empty ?g))
                 (not (on-surface ?obj ?loc))))

  (:action transfer
    :parameters (?obj - object ?g - gripper ?from - location ?to - location)
    :precondition (and (holding ?g ?obj) (gripper-at ?g ?from))
    :effect (and (gripper-at ?g ?to) (not (gripper-at ?g ?from))))

  (:action release
    :parameters (?obj - object ?g - gripper ?loc - location)
    :precondition (and (holding ?g ?obj) (gripper-at ?g ?loc))
    :effect (and (at-target ?obj ?loc) (gripper-empty ?g)
                 (not (holding ?g ?obj)))))