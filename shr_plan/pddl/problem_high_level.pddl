(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     living_room home outside bedroom - Landmark
     drinking - DrinkingProtocol
     am_meds pm_meds - MedicineProtocol
     nathan - Person
     t1 - Time  ;;t2 t3 t4 t5

    ;; for low level
    reminder_1_msg reminder_2_msg voice_msg - Msg
    first_reminder second_reminder - ReminderAction
    caregiver_call - CallAction
    voice_command - VoiceAction

  )
  (:init
      (priority_1)
      (visible_location home)
      (visible_location living_room)
      (visible_location bedroom)

      (not_visible_location outside)

      ;; add when you need to test the protocl 
      ;;(person_at t1 nathan living_room)
      
      ;;(time_for_drinking_reminder drinking)
      ;;(person_currently_at nathan living_room)
      ;;(robot_at home)
      

      ;;(medicine_location living_room)
      ;;(gym_location living_room)
      ;;(medicine_refill_location living_room)
      ;;(medicine_pharmacy_location living_room)
      ;;(walking_reminder_location living_room)

      ;;(walking_reminder)
      ;;(good_weather walking_reminder)

  )
  (:goal (and (success)  ) )
)