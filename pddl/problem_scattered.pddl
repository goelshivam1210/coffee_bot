(define (problem cups-scattered)
  (:domain coffee-making)
  
  (:objects
    esp-cup1 - espresso-cup
    cap-cup1 - cappuccino-cup
    amer-cup1 - americano-cup
    
    esp-btn - espresso-button
    cap-btn - cappuccino-button
    amer-btn - americano-button
    
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Clean cups scattered across different locations
    (clean esp-cup1)
    (empty esp-cup1)
    (at esp-cup1 loc4)  ; at serving counter
    
    (clean cap-cup1)
    (empty cap-cup1)
    (at cap-cup1 loc3)  ; at coffee machine
    
    (clean amer-cup1)
    (empty amer-cup1)
    (at amer-cup1 loc2)  ; at clean area
    
    ;; Robot initial state
    (robot-at loc1)  ; starts at dirty area
    (hands-free)
  )
  
  (:goal
    (and
      (full esp-cup1)
      (at esp-cup1 loc4)
      (full amer-cup1)
      (at amer-cup1 loc4)
    )
  )
)
