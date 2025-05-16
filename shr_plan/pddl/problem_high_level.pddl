(define (problem high_level)
  (:domain high_level_domain)
  (:objects
     living_room bedroom kitchen dining home outside - Landmark

     am_meds pm_meds - MedicineProtocol
     coffee_reminder microwave_reminder - VideoReminderProtocol
     trash - OneReminderProtocol

     nathan - Person
     t1 - Time ;;t2 t3 t4 t5

    ;; for low level to be saved
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
      
      (person_currently_at nathan living_room)
      (person_at t1 nathan living_room)
      (robot_at home)

      ;;(time_for_one_reminder trash)
      ;;(one_reminder_enabled trash)

      (time_for_video coffee_reminder)
      (video_reminder_enabled coffee_reminder)

      ;;(medicine_location living_room)
      ;;(gym_location living_room)
      ;;(medicine_refill_location living_room)
      ;;(medicine_pharmacy_location living_room)
      ;;(walking_reminder_location living_room)

  )
  (:goal (and (success)  ) )
)