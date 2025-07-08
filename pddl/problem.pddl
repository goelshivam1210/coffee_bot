(define (problem make-cappuccino)
  (:domain coffee-making)
  
  (:objects
    ;; Cups (with their types)
    esp-cup1 - espresso-cup
    cap-cup1 - cappuccino-cup
    amer-cup1 - americano-cup
    
    ;; Buttons (with their types)
    esp-btn - espresso-button
    cap-btn - cappuccino-button
    amer-btn - americano-button
    clean-btn - cleaning-button
    
    ;; Locations (with their types)
    loc1 - dirty-area
    loc2 - clean-area
    loc3 - coffee-machine
    loc4 - serving-counter
  )
  
  (:init
    ;; Initial cup states and locations
    (clean cap-cup1)
    (empty cap-cup1)
    (at cap-cup1 loc2)  ; clean cups at clean area
    
    (empty esp-cup1)    ; dirty cup
    (dirty esp-cup1)
    (at esp-cup1 loc1)  ; dirty cups at dirty area
    
    (clean amer-cup1)
    (empty amer-cup1)
    (at amer-cup1 loc2)
    
    
    ;; Robot initial state
    (robot-at loc2)
    (hands-free)
  )
  
  (:goal
    (and
      (full esp-cup1)
      (at esp-cup1 loc4)  ; cappuccino served at counter
    )
  )
)