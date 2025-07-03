(define (problem minimal-test)
  (:domain coffee-making)
  
  (:objects
    cap-cup1 - cappuccino-cup
    cap-btn - cappuccino-button
    
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Minimal setup - clean cup, robot ready
    (clean cap-cup1)
    (empty cap-cup1)
    (at cap-cup1 loc2)
    
    (robot-at loc2)
    (hands-free)
  )
  
  (:goal
    (full cap-cup1)  ; Just fill the cup, don't require delivery
  )
)
