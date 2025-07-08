(define (problem mixed-cup-states)
  (:domain coffee-making)
  
  (:objects
    esp-cup1 esp-cup2 - espresso-cup
    cap-cup1 - cappuccino-cup
    
    esp-btn - espresso-button
    cap-btn - cappuccino-button
    
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Mix of clean and dirty cups - only clean ones should be usable
    (clean esp-cup1)
    (empty esp-cup1)
    (at esp-cup1 loc2)
    
    (empty esp-cup2)  ; dirty cup - unusable
    (dirty esp-cup2)  ; cup is dirty
    (at esp-cup2 loc1)
    
    (clean cap-cup1)
    (empty cap-cup1)
    (at cap-cup1 loc2)
    
    ;; Robot starts at coffee machine
    (robot-at loc3)
    (hands-free)
  )
  
  (:goal
    (and
      (full esp-cup1)
      (at esp-cup1 loc4)
      (full cap-cup1)
      (at cap-cup1 loc4)
    )
  )
)
