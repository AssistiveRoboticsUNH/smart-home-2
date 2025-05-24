(define (problem video_reminder)
(:domain shr_domain)
(:objects
    dinning kitchen bedroom home outside - Landmark
    ;;living_room bedroom kitchen dining home outside - Landmark
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
    ;;(person_at t1 nathan bedroom)
    ;;(robot_at home)

    ;; Enabled actions
    (DetectPerson_enabled)
    (GiveReminder_enabled)
    (GiveReminder_enabled)
    (MakeVoice_enabled)

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
    ;; Add bedroom location if needed

    (traversable home living_room)
    (traversable living_room home)
    ;;(traversable home bedroom)
    ;;(traversable bedroom home)
    ;;(traversable bedroom living_room)
    ;;(traversable living_room bedroom)

    ;;(play_video)
    (unknown (play_video))
    (DetectPlayVideo_enabled)

    (message_given_success reminder_1_msg)

    ;; Enforce same location constraint for interactions
    (same_location_constraint)

    ;; Specify required action order
    (valid_voice_message voice_command voice_msg)
    ;;(voice_blocks_reminder voice_command first_reminder)

    ;; Define valid messages for reminders
    (valid_reminder_message first_reminder reminder_1_msg)

    ;; Ensure robot waits only when not outside
    (wait_not_person_location_constraint t1 nathan outside)
    (wait_not_person_location_constraint t2 nathan outside)
    (wait_not_person_location_constraint t3 nathan outside)
    (wait_not_person_location_constraint t4 nathan outside)
    (wait_not_person_location_constraint t5 nathan outside)

    (wait_not_person_location_constraint t1 nathan bedroom)
    (wait_not_person_location_constraint t2 nathan bedroom)
    (wait_not_person_location_constraint t3 nathan bedroom)
    (wait_not_person_location_constraint t4 nathan bedroom)
    (wait_not_person_location_constraint t5 nathan bedroom)

    (noaction_person_location_constraint na1 nathan bedroom)
    (noaction_person_location_constraint na2 nathan bedroom)
    (noaction_person_location_constraint na3 nathan bedroom)

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
