(define (problem robot-far-from-cups)
  (:domain coffee-making)
  
  (:objects
    amer-cup1 - americano-cup
    amer-btn - americano-button
    
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Clean cup at clean area
    (clean amer-cup1)
    (empty amer-cup1)
    (at amer-cup1 loc2)
    
    ;; Robot starts far away at serving counter
    (robot-at loc4)
    (hands-free)
  )
  
  (:goal
    (and
      (full amer-cup1)
      (at amer-cup1 loc4)
    )
  )
)
