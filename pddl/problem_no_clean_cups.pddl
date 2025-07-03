; Will not have a solution

(define (problem no-clean-cups)
  (:domain coffee-making)
  
  (:objects
    cap-cup1 - cappuccino-cup
    cap-btn - cappuccino-button
    
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Only dirty cups available - should be unsolvable
    (empty cap-cup1)
    (at cap-cup1 loc1)  ; dirty cup at dirty area (not clean)
    
    ;; Robot initial state
    (robot-at loc3)
    (hands-free)
  )
  
  (:goal
    (and
      (full cap-cup1)
      (at cap-cup1 loc4)
    )
  )
)
