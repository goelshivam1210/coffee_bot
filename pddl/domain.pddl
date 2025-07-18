(define (domain coffee-making)
  (:requirements :strips :typing)
  
  (:types
    ;; Cup subtypes
    espresso-cup cappuccino-cup americano-cup - cup
    ;; Button subtypes  
    espresso-button cappuccino-button americano-button cleaning-button - button
    ;; Location subtypes
    dirty-area clean-area coffee-machine serving-counter - location
  )
  
  (:predicates
    ;; Cup states
    (clean ?c - cup)
    (empty ?c - cup)  ; dirty/used but empty
    (full ?c - cup)
    (dirty ?c - cup)  ; explicitly dirty state
    
    ;; Location properties
    (at ?c - cup ?l - location)
    
    ;; Robot properties
    (robot-at ?l - location)
    (holding ?c - cup)
    (hands-free)
  )
  
  ;; Pick up a cup from a location
  (:action pick
    :parameters (?c - cup ?l - location)
    :precondition (and
      (at ?c ?l)
      (robot-at ?l)
      (hands-free)
    )
    :effect (and
      (not (at ?c ?l))
      (holding ?c)
      (not (hands-free))
    )
  )
  
  ;; Place a cup at a location
  (:action place
    :parameters (?c - cup ?l - location)
    :precondition (and
      (holding ?c)
      (robot-at ?l)
    )
    :effect (and
      (at ?c ?l)
      (not (holding ?c))
      (hands-free)
    )
  )
  
  ;; Move robot between locations
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and
      (robot-at ?from)
    )
    :effect (and
      (not (robot-at ?from))
      (robot-at ?to)
    )
  )
  
  ;; Press cleaning button - cleans one cup at a time
  (:action press-cleaning-button
    :parameters (?b - cleaning-button ?c - cup ?da - dirty-area)
    :precondition (and
      (at ?c ?da)
      (robot-at ?da)
      (dirty ?c)
      (empty ?c)
    )
    :effect (and
      (not (dirty ?c))
      (clean ?c)
    )
  )
  
  ;; Move cleaned cup to clean area
  ;;(:action move-cleaned-cup
  ;;  :parameters (?c - cup ?da - dirty-area ?ca - clean-area)
  ;;  :precondition (and
  ;;    (at ?c ?da)
  ;;    (robot-at ?da)
  ;;    (clean ?c)
  ;;    (empty ?c)
  ;;  )
  ;;  :effect (and
  ;;    (not (at ?c ?da))
  ;;    (at ?c ?ca)
  ;;  )
  ;;)
  
  ;; Press espresso button
  (:action press-button-espresso
    :parameters (?b - espresso-button ?c - espresso-cup ?cm - coffee-machine)
    :precondition (and
      (at ?c ?cm)
      (robot-at ?cm)
      (clean ?c)
      (empty ?c)
    )
    :effect (and
      (not (clean ?c))
      (not (empty ?c))
      (full ?c)
    )
  )
  
  ;; Press cappuccino button
  (:action press-button-cappuccino
    :parameters (?b - cappuccino-button ?c - cappuccino-cup ?cm - coffee-machine)
    :precondition (and
      (at ?c ?cm)
      (robot-at ?cm)
      (clean ?c)
      (empty ?c)
    )
    :effect (and
      (not (clean ?c))
      (not (empty ?c))
      (full ?c)
    )
  )
  
  ;; Press americano button
  (:action press-button-americano
    :parameters (?b - americano-button ?c - americano-cup ?cm - coffee-machine)
    :precondition (and
      (at ?c ?cm)
      (robot-at ?cm)
      (clean ?c)
      (empty ?c)
    )
    :effect (and
      (not (clean ?c))
      (not (empty ?c))
      (full ?c)
    )
  )
)