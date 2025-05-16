(define (problem one_reminder)
(:domain shr_domain)
(:objects
    ;;living_room kitchen dining home outside - Landmark
    living_room bedroom kitchen dining home outside - Landmark
    nathan - Person
    t1 t2 t3 t4 t5 - Time
    reminder_1_msg - Msg
    first_reminder  - ReminderAction
    w1 w2 w3 w4 w5 - WaitAction
    na1 na2 na3 - NoAction
)
(:init
    ;;(person_at t1 nathan bedroom)
    ;;(robot_at home)
    ;;(robot_at_time t1 home)
    (person_at t1 nathan living_room)
    (robot_at home)

    ;;(no_action)

    (DetectPerson_enabled)
    (GiveReminder_enabled)

    (current_time t1)
    (next_time t1 t2)
    (next_time t2 t3)
    (next_time t3 t4)
    (next_time t4 t5)

    (oneof (person_at t2 nathan living_room) (person_at t2 nathan kitchen) (person_at t2 nathan outside) (person_at t2 nathan dining) (person_at t2 nathan bedroom))
    (oneof (person_at t3 nathan living_room) (person_at t3 nathan kitchen) (person_at t3 nathan outside) (person_at t3 nathan dining) (person_at t3 nathan bedroom))
    (oneof (person_at t4 nathan living_room) (person_at t4 nathan kitchen) (person_at t4 nathan outside) (person_at t4 nathan dining) (person_at t4 nathan bedroom))
    (oneof (person_at t5 nathan living_room) (person_at t5 nathan kitchen) (person_at t5 nathan outside) (person_at t5 nathan dining) (person_at t5 nathan bedroom))


    (traversable home living_room)
    (traversable living_room home)
    (traversable home kitchen)
    (traversable kitchen home)
    (traversable dining home)
    (traversable home dining)
    (traversable dining living_room)
    (traversable living_room dining)
    (traversable dining kitchen)
    (traversable kitchen dining)
    (traversable living_room kitchen)
    (traversable kitchen living_room)

    (traversable home bedroom)
    (traversable bedroom home)
    (traversable bedroom kitchen)
    (traversable kitchen bedroom)
    (traversable dining bedroom)
    (traversable bedroom dining)
    (traversable living_room bedroom)
    (traversable bedroom living_room)


    (same_location_constraint)
    ;;(not_same_location_constraint)

    ;;success states

    (message_given_success reminder_1_msg)
    (person_at_success nathan outside)

    ;; specify valid input argument combinations for all actions
    (valid_reminder_message first_reminder reminder_1_msg)

    ;; specify world state constraints for all actions
    (reminder_person_location_constraint first_reminder nathan bedroom)

    (wait_not_person_location_constraint t1 nathan outside)
    (wait_not_person_location_constraint t2 nathan outside)
    (wait_not_person_location_constraint t3 nathan outside)
    (wait_not_person_location_constraint t4 nathan outside)
    (wait_not_person_location_constraint t5 nathan outside)

    ;; outside or no action will be used
    (noaction_person_location_constraint na1 nathan outside)
    (noaction_person_location_constraint na2 nathan outside)
    (noaction_person_location_constraint na3 nathan outside)


    (wait_robot_location_constraint t1 home)
    (wait_robot_location_constraint t2 home)
    (wait_robot_location_constraint t3 home)
    (wait_robot_location_constraint t4 home)

)
(:goal (and (success)  ) )

)
