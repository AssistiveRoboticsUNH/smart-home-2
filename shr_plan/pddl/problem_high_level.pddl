(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     home bedroom outside living_room - Landmark
     drinking - DrinkingProtocol
     am_meds pm_meds - MedicineProtocol
     em_trash - EmptyTrashProtocol
     em_dishwasher - EmptyDishwasherProtocol
     morning_wake - MorningWakeProtocol
     shower - ShowerProtocol
     pam_location - PamLocationProtocol
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
      ;;(check_location_wakeup living_room)
      ;;(check_location_wakeup home)

      (not_visible_location outside)

      ;;(time_for_morning_wake_reminder morning_wake)
      (person_currently_at nathan living_room)
      (person_at t1 nathan living_room)
      (robot_at home)


      ;; add when you need to test the protocl 
      ;;(person_at t1 nathan living_room)
      ;;(person_currently_at nathan living_room)
      ;;(robot_at home)
      
      ;;(time_for_drinking_reminder drinking)
      ;;(time_for_empty_trash_reminder em_trash)
      ;;(time_for_empty_dishwasher_reminder em_dishwasher)
      ;;(time_for_morning_wake_reminder morning_wake)
      ;;(time_for_shower_reminder shower)
      ;;(shower_reminder_enabled shower)
      ;;(already_reminded_shower shower)
      ;;(time_for_pam_location_reminder pam_location)
      ;;(pam_outside pam_location)
      ;;(pam_location_reminder_enabled pam_location)


      

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