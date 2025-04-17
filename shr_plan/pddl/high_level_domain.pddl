(define (domain high_level_domain)

(:requirements
  :strips
  :typing
)

(:types
  DrinkingProtocol
  MedicineProtocol
  EmptyDishwasherProtocol
  EmptyTrashProtocol
  Landmark
  Time
  Person

  ;; low level from types in low level domain
  ;; was added because kb doesnt store intermediate success for the low level
  ;; this enable the low level to continue form where it stopped by adding to high level
  ;; kb only keeps predicates that are part of the high level
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

   ;; drinking reminder
  (drinking_protocol_enabled ?dr - DrinkingProtocol)
  (time_for_drinking_reminder ?dr - DrinkingProtocol)
  (already_reminded_drinking ?dr - DrinkingProtocol)

  ;; medicine
  (medicine_protocol_enabled ?med - MedicineProtocol)
  (time_to_take_medicine ?med - MedicineProtocol)
  (already_took_medicine ?m - MedicineProtocol)
  (already_reminded_medicine ?m - MedicineProtocol)
  (already_called_about_medicine ?m - MedicineProtocol)


;; empty trash reminder
  (empty_trash_protocol_enabled ?etr - EmptyTrashProtocol)
  (time_for_empty_trash_reminder ?etr - EmptyTrashProtocol)
  (already_reminded_empty_trash ?etr - EmptyTrashProtocol)

;; empty dishwasher reminder
  (empty_dishwasher_protocol_enabled ?etd - EmptyDishwasherProtocol)
  (time_for_empty_dishwasher_reminder ?etd - EmptyDishwasherProtocol)
  (already_reminded_empty_dishwasher ?etd - EmptyDishwasherProtocol)

  (low_level_failed)

  ;; priority
  ;; all protocols should have a priority higher than that of idle
  (priority_1)
  (priority_2)
  (priority_3)
  (priority_4)
  (priority_5)
  (priority_6)
 

  (dont_use_shutdown)
  (success)

  ;; for low level domain
  ;; these should include actions that need to be done only once
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

(:action ChangePriority_5_6
	:parameters ()
	:precondition (and
	    (priority_5)
		)
	:effect (and (priority_6) (not (priority_5)))
)

;; to start ros and navigation before the protocol
(:action StartROS
	:parameters ()
	:precondition (;;and
        ;;(priority_2)
	    ;; will be triggered before it starts a protocol
		)
	:effect (and
                (started)
                ;;(not (priority_2))
          )
)

(:action StartDrinkingProtocol
	:parameters (?d - DrinkingProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	  ;; all protocols should have a priority higher than that of idle
	  (priority_2)
      (time_for_drinking_reminder ?d)

      (visible_location ?dest)
      ;;(visible_location ?cur)

      (person_currently_at ?p ?cur)
      (robot_at ?cur)

      (not (already_reminded_drinking ?d))

      (forall (?dr - DrinkingProtocol) (not (drinking_protocol_enabled ?dr)) )
      (started)
		)
	:effect (and
	        (success)
            (not (priority_2))
            (drinking_protocol_enabled ?d)
            (not (low_level_failed))
            ;; for every protocol in types it has to have a forall to disable other protocols before starting this one
            (forall (?medicine_protocol - MedicineProtocol) (not (medicine_protocol_enabled ?medicine_protocol)) )
            (forall (?empty_trash_protocol - EmptyTrashProtocol) (not (empty_trash_protocol_enabled ?empty_trash_protocol)) )
            (forall (?empty_dishwasher_protocol - EmptyDishwasherProtocol) (not (empty_dishwasher_protocol_enabled ?empty_dishwasher_protocol)) )
          )
)

(:action ContinueDrinkingProtocol
	:parameters (?d - DrinkingProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_for_drinking_reminder ?d)
      (not (already_reminded_drinking ?d))
      (drinking_protocol_enabled ?d)
		)
	:effect (and (success) (not (priority_2)) )
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
            (forall (?drinking_protocol - DrinkingProtocol) (not (drinking_protocol_enabled ?drinking_protocol)) )
            (forall (?empty_trash_protocol - EmptyTrashProtocol) (not (empty_trash_protocol_enabled ?empty_trash_protocol)) )
            (forall (?empty_dishwasher_protocol - EmptyDishwasherProtocol) (not (empty_dishwasher_protocol_enabled ?empty_dishwasher_protocol)) )

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

(:action StartEmptyTrashProtocol
	:parameters (?e - EmptyTrashProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)
      (time_for_empty_trash_reminder ?e)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))
      (person_currently_at ?p ?cur)
      (robot_at ?cur)

      (not (already_reminded_empty_trash ?e))
      (forall (?etr - EmptyTrashProtocol) (not (empty_trash_protocol_enabled ?etr)) )
      (started)
		)
	:effect (and
	          (success)
            (not (priority_2))
            (empty_trash_protocol_enabled ?e)
            (not (low_level_failed))
            (forall (?drinking_protocol - DrinkingProtocol) (not (drinking_protocol_enabled ?drinking_protocol)) )
            (forall (?medicine_protocol - MedicineProtocol) (not (medicine_protocol_enabled ?medicine_protocol)) )
            (forall (?empty_dishwasher_protocol - EmptyDishwasherProtocol) (not (empty_dishwasher_protocol_enabled ?empty_dishwasher_protocol)) )
          )
)

(:action ContinueEmptyTrashProtocol
	:parameters (?e - EmptyTrashProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_for_empty_trash_reminder ?e)
      (not (already_reminded_empty_trash ?e))
      (empty_trash_protocol_enabled ?e)
		)
	:effect (and (success) (not (priority_2)) )
)

(:action StartEmptyDishwaserProtocol
	:parameters (?e - EmptyDishwasherProtocol ?p - Person ?cur - Landmark ?dest - Landmark)
	:precondition (and
	    (priority_2)
      (time_for_empty_dishwasher_reminder ?e)
      (visible_location ?dest)
      (not (not_visible_location ?dest))
      (visible_location ?cur)
      (not (not_visible_location ?cur))
      (person_currently_at ?p ?cur)
      (robot_at ?cur)

      (not (already_reminded_empty_dishwasher ?e))
      (forall (?etd - EmptyDishwasherProtocol) (not (empty_dishwasher_protocol_enabled ?etd)) )
      (started)
		)
	:effect (and
	          (success)
            (not (priority_2))
            (empty_dishwasher_protocol_enabled ?e)
            (not (low_level_failed))
            (forall (?drinking_protocol - DrinkingProtocol) (not (drinking_protocol_enabled ?drinking_protocol)) )
            (forall (?medicine_protocol - MedicineProtocol) (not (medicine_protocol_enabled ?medicine_protocol)) )
            (forall (?empty_trash_protocol - EmptyTrashProtocol) (not (empty_trash_protocol_enabled ?empty_trash_protocol)) )
          )
)

(:action ContinueEmptyDishWasherProtocol
	:parameters (?e - EmptyDishwasherProtocol)
	:precondition (and
	    (priority_2)
	    (not (low_level_failed))
      (time_for_empty_dishwasher_reminder ?e)
      (not (already_reminded_empty_dishwasher ?e))
      (empty_dishwasher_protocol_enabled ?e)
		)
	:effect (and (success) (not (priority_2)) )
)

(:action Idle
	:parameters ()
	:precondition (and
	    (priority_6)
		)
	:effect (and (success)
	             (not (priority_6))
	             (not (low_level_failed))

	             ;; for every protocol in types it has to have a forall to disable other protocols before starting this one
                (forall (?dr - DrinkingProtocol) (not (drinking_protocol_enabled ?dr)) )
                ;; ADD CHANGES HERE
                (forall (?med - MedicineProtocol) (not (medicine_protocol_enabled ?med)) )
                (forall (?etr - EmptyTrashProtocol) (not (empty_trash_protocol_enabled ?etr)) )
                (forall (?etd - EmptyDishwasherProtocol) (not (empty_dishwasher_protocol_enabled ?etd)) )


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
            (priority_5)

            ;; CANT SHUTDOWN IF time to do something is true and
            ;; all predicates indicating that they it is done are false
            ;; give F in such case

            ;; need to add a forall for every protocol objects in problem
            ;;; 1
            ;;; forall would give false if one is F

            ;; ADD CHANGES HERE
            ;; for all has to be done for all protocol types
            ;;;(forall (?med - MedicineProtocol)
            ;;;    (not
            ;;;        (and
            ;;;            (time_to_take_medicine ?med)
            ;;;            (not (already_took_medicine ?med))
            ;;;            (not (already_reminded_medicine ?med))
            ;;;            (not (already_called_about_medicine ?med))
            ;;;        )
            ;;;    )
            ;;;)

            ;;; 1
            (forall (?dr - DrinkingProtocol)
                (not
                    (and
                        (time_for_drinking_reminder ?dr)
                        (not (already_reminded_drinking ?dr))
                    )
                )
            )

	    )
	:effect (and (success)
	            (not (priority_5))
                (not (low_level_failed))
                (not started)
          )
)


)