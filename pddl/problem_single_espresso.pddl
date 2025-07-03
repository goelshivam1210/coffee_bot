(define (problem single-espresso)
  (:domain coffee-making)
  
  (:objects
    esp-cup1 - espresso-cup
    esp-btn - espresso-button
    
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Clean cup available
    (clean esp-cup1)
    (empty esp-cup1)
    (at esp-cup1 loc2)
    
    ;; Robot initial state
    (robot-at loc2)
    (hands-free)
  )
  
  (:goal
    (and
      (full esp-cup1)
      (at esp-cup1 loc4)
    )
  )
)
