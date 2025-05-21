(define (problem medicine_reminder)
(:domain shr_domain)
(:objects
    living_room bedroom home outside - Landmark

    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg voice_msg - Msg
    first_reminder - ReminderAction
    voice_command - VoiceAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;; Initial person and robot locations
    ;;(person_at t1 nathan living_room)
    ;;(robot_at home)

    ;; Enabled actions
    (DetectPerson_enabled)
    (GiveReminder_enabled)
    (MakeVoice_enabled)
    (DetectTakingMedicine_enabled)

    ;; Time progression
    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    ;; Person can be at different locations at future times
    (oneof (person_at t2 nathan living_room) (person_at t2 nathan bedroom) (person_at t2 nathan outside) )
    (oneof (person_at t3 nathan living_room) (person_at t3 nathan bedroom) (person_at t3 nathan outside) )
    (oneof (person_at t4 nathan living_room) (person_at t4 nathan bedroom) (person_at t4 nathan outside) )
    (oneof (person_at t5 nathan living_room) (person_at t5 nathan bedroom) (person_at t5 nathan outside) )

    ;; Allow traversal between locations if needed
    (traversable home living_room)
    (traversable living_room home)

    (traversable home bedroom)
    (traversable bedroom home)

    (traversable bedroom living_room)
    (traversable living_room bedroom)

    ;; Define success states
    (message_given_success voice_msg)
    (medicine_taken_success)

    ;; Enforce same location constraint for interactions
    (same_location_constraint)

    ;; Specify required action order
    (reminder_blocks_voice first_reminder voice_command)

    ;; Define valid messages for reminders
    (valid_voice_message voice_command voice_msg)
    (valid_reminder_message first_reminder reminder_1_msg)

    ;; Constraints: Reminders should not be given if Nathan is taking medicine
    (reminder_person_not_taking_medicine_constraint first_reminder nathan)
    (voice_person_not_taking_medicine_constraint voice_command nathan)

    ;; Ensure robot waits only when not outside
    (wait_not_person_location_constraint t1 nathan outside)
    (wait_not_person_location_constraint t2 nathan outside)
    (wait_not_person_location_constraint t3 nathan outside)
    (wait_not_person_location_constraint t4 nathan outside)
    (wait_not_person_location_constraint t5 nathan outside)

    ;; If person is outside, enforce no action
    (noaction_person_location_constraint na1 nathan outside)
    (noaction_person_location_constraint na2 nathan outside)
    (noaction_person_location_constraint na3 nathan outside)

    ;; Ensure the robot can wait only if at home
    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)
)
(:goal (and
        (success))
)
)
