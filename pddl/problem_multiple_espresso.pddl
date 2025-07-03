(define (problem multiple-same-type)
  (:domain coffee-making)
  
  (:objects
    esp-cup1 esp-cup2 - espresso-cup
    esp-btn - espresso-button
    
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Two clean espresso cups
    (clean esp-cup1)
    (empty esp-cup1)
    (at esp-cup1 loc2)
    
    (clean esp-cup2)
    (empty esp-cup2)
    (at esp-cup2 loc2)
    
    ;; Robot initial state
    (robot-at loc4)
    (hands-free)
  )
  
  (:goal
    (and
      (full esp-cup1)
      (at esp-cup1 loc4)
      (full esp-cup2)
      (at esp-cup2 loc4)
    )
  )
)
