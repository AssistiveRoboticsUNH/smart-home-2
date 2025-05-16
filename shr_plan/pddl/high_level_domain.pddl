(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  VideoReminderProtocol
  MedicineProtocol
  OneReminderProtocol
  Landmark
  Time
  Person

  ;; low level
    Msg
    ReminderAction
    CallAction
    VoiceAction
)

(:predicates
  (started)

  (robot_at ?lmr - Landmark)
  (person_at ?t - Time ?p - Person ?lmp - Landmark)
  (person_currently_at ?p - Person ?lmp - Landmark)

  (visible_location ?lmp - Landmark)
  (not_visible_location ?lmp - Landmark)

  (medicine_protocol_enabled ?med - MedicineProtocol)
  (video_reminder_enabled ?vid - VideoReminderProtocol)
  (one_reminder_enabled ?o - OneReminderProtocol)

  ;; medicine
  ;;(medicine_location ?lm - Landmark)
  (time_to_take_medicine ?med - MedicineProtocol)
  (already_took_medicine ?m - MedicineProtocol)
  (already_reminded_medicine ?m - MedicineProtocol)
  (already_called_about_medicine ?m - MedicineProtocol)

  ;; video protocol
  (time_for_video ?v - VideoReminderProtocol)
  (already_showed_video ?v - VideoReminderProtocol)

  ;; one reminders
  (time_for_one_reminder ?o - OneReminderProtocol)
  (already_gave_one_reminder ?o - OneReminderProtocol)

  (low_level_failed)

  ;; priority
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)

  (low_level_failed)
  (dont_use_shutdown)
  (success)

  ;; for low level
  (executed_reminder ?a - ReminderAction)
  (executed_call ?c - CallAction)
  (executed_voice ?a - VoiceAction)
  (message_given ?m - Msg)

)

(:action MoveToLandmark
	:parameters (?from - Landmark ?to - Landmark)
	:precondition (and
	                (robot_at ?from)
	                (started)
	                (visible_location ?from)
                    (visible_location ?to)
                    (not (not_visible_location ?from))
                    (not (not_visible_location ?to))
	          )
	:effect (and (robot_at ?to) (not (robot_at ?from)) )
)

(:action ChangePriority_1_2
	:parameters ()
	:precondition (and
	    (priority_1)
		)
	:effect (and (priority_2) (not (priority_1)))
)
(:action ChangePriority_2_3
	:parameters ()
	:precondition (and
	    (priority_2)
		)
	:effect (and (priority_3) (not (priority_2)))
)
(:action ChangePriority_3_4
	:parameters ()
	:precondition (and
	    (priority_3)
		)
	:effect (and (priority_4) (not (priority_3)))
)
(:action ChangePriority_4_5
	:parameters ()
	:precondition (and
	    (priority_4)
		)
	:effect (and (priority_5) (not (priority_4)))
)

;; to start ros and navigation before the protocol
(:action StartROS
	:parameters ()
	:precondition (;;and
	       ;; will be triggered before it starts a protocol
           ;; (priority_2)
		)
	:effect (and
	            ;;(not (priority_2))
                (started)
          )
)

(:action StartMedicineProtocol
	:parameters (?m - MedicineProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)
        (time_to_take_medicine ?m)
        (visible_location ?dest)
        (not (not_visible_location ?dest))

        (visible_location ?cur)
        (not (not_visible_location ?cur))
        (person_currently_at ?p ?cur)
        (robot_at ?cur)

      ;;(medicine_location ?dest)
      
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
      (started)
	)
	:effect (and
	          (success)
            (not (priority_2))
            (medicine_protocol_enabled ?m)
            (not (low_level_failed))
            (forall (?vid - VideoReminderProtocol) (not (video_reminder_enabled ?vid)) )
            (forall (?one - OneReminderProtocol) (not (one_reminder_enabled ?one)) )
          )
)

(:action ContinueMedicineProtocol
	:parameters (?m - MedicineProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_to_take_medicine ?m)
      (not (already_took_medicine ?m))
      (not (already_reminded_medicine ?m))
      (not (already_called_about_medicine ?m))
      (medicine_protocol_enabled ?m)
		)
	:effect (and (success) (not (priority_2)) )
)

;; video reminder Protocol
(:action StartVideoReminderProtocol
	:parameters (?vid - VideoReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	  (priority_2)
      (robot_at ?cur)

      (time_for_video ?vid)
      (not (already_showed_video ?vid))
      (forall (?vid - VideoReminderProtocol) (not (video_reminder_enabled ?vid)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))

      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (video_reminder_enabled ?vid)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?one - OneReminderProtocol) (not (one_reminder_enabled ?one)) )
          )
)

(:action ContinueVideoReminderProtocol
	:parameters (?vid - VideoReminderProtocol)
	:precondition (and
	  (priority_2)
	  (not (low_level_failed))
      (not (already_showed_video ?vid))
      (video_reminder_enabled ?vid)
      (time_for_video ?vid)
    )
	:effect (and (success) (not (priority_2)) )
)

(:action StartOneReminderProtocol
	:parameters (?o - OneReminderProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	  (priority_2)
      (robot_at ?cur)

      (time_for_one_reminder ?o)
      (not (already_gave_one_reminder ?o))
      (forall (?one - OneReminderProtocol) (not (one_reminder_enabled ?one)) )

      ;; person in visible area
      (person_currently_at ?p ?cur)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))

      (started)

    )
	:effect (and
	          (success)
	          (not (priority_2))
	          (one_reminder_enabled ?o)
	          (not (low_level_failed))
	          (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
              (forall (?vid - VideoReminderProtocol) (not (video_reminder_enabled ?vid)) )

          )
)

(:action ContinueOneReminderProtocol
	:parameters (?on - OneReminderProtocol)
	:precondition (and
	  (priority_2)
	  (not (low_level_failed))
      (not (already_gave_one_reminder ?on))
      (one_reminder_enabled ?on)
      (time_for_one_reminder ?on)
    )
	:effect (and (success) (not (priority_2)) )
)


(:action Idle
	:parameters ()
	:precondition (and
	    (priority_5)
		)
	:effect (and (success)
	              (not (priority_5))
                (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
                (forall (?vid - VideoReminderProtocol) (not (video_reminder_enabled ?vid)) )
                (not (low_level_failed))
          )
)


;; shutdown is supposed to stop ros2 processes
;; it should try to dock if it is not docked
;; triggered when there should be protocol and it has been done

(:action Shutdown
	:parameters ()
	:precondition
	    (and
	        (started)
	        ;; has to be higher priority than idle
            (priority_4)

            ;; CANT SHUTDOWN IF time to do something is true and
            ;; all predicates indicating that they it is done are false
            ;; give F in such case

            ;; need to add a forall for every protocol objects in problem
            ;;; 1
            ;;; forall would give false if one is F
            (forall (?med - MedicineProtocol)
                (not
                    (and
                        (time_to_take_medicine ?med)
                        (not (already_took_medicine ?med))
                        (not (already_reminded_medicine ?med))
                        (not (already_called_about_medicine ?med))
                    )
                )
            )
            ;;; 2
            (forall (?vid - VideoReminderProtocol)
                (not
                    (and
                        (time_for_video ?vid)
                        (not (already_showed_video ?vid))
                    )
                )
            )
	    )
	:effect (and (success)
	            (not (priority_4))
                (not (low_level_failed))
                (not started)
          )
)


)

